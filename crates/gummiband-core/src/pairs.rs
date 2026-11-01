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
}

impl Default for PairConfig {
    fn default() -> Self {
        PairConfig {
            skew: 0.1,
            zdiff: None,
            all: false,
            room: None,
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
            ends.push(PairEnd { p, n });
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

fn merge(board: &Board, all: bool) -> Merged {
    let mut b = board.clone();
    let mut pairs = Vec::new();
    let mut skipped = Vec::new();
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
            let mid = (b.pins[e.p].pos + b.pins[e.n].pos) * 0.5;
            for i in b.pins[e.n].obstacles.clone() {
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
fn offset_side(elems: &[Elem], d: f64) -> Vec<Elem> {
    let c = remove_loops(offset_curve(elems, d), 6.0 * d.abs() + 0.5);
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
            .map(|e| (real.pins[e.p].pos + real.pins[e.n].pos) * 0.5)
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
            // at the far end the pads lie ahead: seen from them the tracks arrive
            let exit_a = exit_of(real, ea, u0);
            let exit_b = exit_of(real, eb, u1 * -1.0);
            let (left_p, left_p_end) = (exit_a.p_left, !exit_b.p_left);
            let twisted = left_p != left_p_end;
            let via_joints: Vec<usize> = (0..joints.len()).filter(|&j| joints[j].x.is_finite()).collect();
            if twisted && via_joints.is_empty() {
                rep.note = "polarity is crossed and the wide track has no via".to_string();
                unfinished.push(rep.p.clone());
                unfinished.push(rep.n.clone());
                continue;
            }
            // where both legs have room for the tracks to change sides; best where the band
            // turns back, there is no detour around the vias there
            let swap_room = |&j: &usize| {
                let (a, b) = (&legs[j].elems, &legs[j + 1].elems);
                let back = tangent(a.last().unwrap(), true).dot(tangent(&b[0], false)) < 0.0;
                elems_len(a).min(elems_len(b)) + if back { 1.0 } else { 0.0 }
            };
            let swap_at = twisted
                .then(|| {
                    via_joints
                        .iter()
                        .copied()
                        .max_by(|a, b| swap_room(a).total_cmp(&swap_room(b)))
                })
                .flatten();
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
                let (full_l, full_r) = (offset_side(&leg.elems, s), offset_side(&leg.elems, -s));
                let mut start_trim = reach_start.unwrap_or(0.0);
                if reach_start.is_none() {
                    let base = exit_trim(&exit_a, u0, s, w, clr);
                    let sides = [
                        (ln_start, net_l, full_l.as_slice()),
                        (rn_start, net_r, full_r.as_slice()),
                    ];
                    let (more, t) = best_exit(&around, &exit_a, sides, base, w, clr, leg.layer);
                    let out = exit_a.x * (exit_a.ext + more);
                    lead_l = vec![ln_start + out];
                    lead_r = vec![rn_start + out];
                    start_trim = t;
                }
                let mut end_trim = disc + 0.02;
                let mut exits = None;
                if k + 1 == legs.len() {
                    let base = exit_trim(&exit_b, u1, s, w, clr);
                    let (tl, tr) = (pad_target(real, eb, left_is_p), pad_target(real, eb, !left_is_p));
                    let (bl, br) = (reversed(&full_l), reversed(&full_r));
                    let sides = [(tl, net_l, bl.as_slice()), (tr, net_r, br.as_slice())];
                    let (more, t) = best_exit(&around, &exit_b, sides, base, w, clr, leg.layer);
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
                let t = if back { (ua - ub).norm() } else { (ua + ub).norm() };
                let n = left_of(t);
                let h = (via_d + via_gap) / 2.0;
                let swap = swap_at == Some(k);
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
                reach_start = Some(disc + 0.02);
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
    /// Index of the path.
    path: usize,
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
}

fn track_len(t: &Track) -> f64 {
    t.pts.windows(2).map(|w| w[0].dist(w[1])).sum()
}

/// How far from the ends of a path a clearance problem is worth giving up the wide track.
const REPAIR_REACH: f64 = 6.0;

/// Next to the pads and vias the two tracks do not always fit, the wide track knew nothing of
/// them. Where a path has clearance problems near one of its ends, take its tracks and vias
/// away up to the last problem: single tracks connect the rest. Returns the nets concerned.
fn repair(real: &Board, geo: &mut Geometry, paths: &[PairPath], skip: &[usize]) -> (Vec<String>, Vec<Cut>) {
    let viol = crate::check::check(real, geo);
    let mut names = Vec::new();
    let mut cuts = Vec::new();
    let mut gone_vias: Vec<usize> = Vec::new();
    for (pi, path) in paths.iter().enumerate() {
        if skip.contains(&pi) {
            continue;
        }
        let hits: Vec<Point> = viol
            .iter()
            .filter(|v| [path.p, path.n].iter().any(|&x| v.nets.0 == x || v.nets.1 == Some(x)))
            .map(|v| v.at)
            .collect();
        if hits.is_empty() {
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
    let top = 2.0 * w;
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
            let span = 2.0 * h + top;
            if from + span + 0.3 > len {
                continue;
            }
            let p0 = a + u * from;
            let p1 = p0 + u * h + out * h;
            let p2 = p1 + u * top;
            let p3 = p2 + u * h - out * h;
            // around the bump: its copper and the clearance on the far side
            let free = [(p0, p1), (p1, p2), (p2, p3)]
                .iter()
                .all(|&(x, y)| around.slack(net, x, y, w, clr, layer) >= 0.02)
                && around.board.edge_dist(p1) >= edge
                && around.board.edge_dist(p2) >= edge;
            if free {
                elems.push(Elem::Line(cur, p0));
                elems.push(Elem::Line(p0, p1));
                elems.push(Elem::Line(p1, p2));
                elems.push(Elem::Line(p2, p3));
                cur = p3;
                gained += h * BUMP_GAIN;
                from += span + 2.0 * top;
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
fn best_exit(
    around: &Around,
    ex: &Exit,
    sides: [(Point, NetId, &[Elem]); 2],
    base: f64,
    w: f64,
    clr: f64,
    layer: LayerId,
) -> (f64, f64) {
    let near = around.near(sides[0].0, 8.0);
    let mut best = (0.0, base);
    let mut room = f64::NEG_INFINITY;
    for f in [1.0, 1.3, 1.7, 2.2, 3.0] {
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
    let merged = merge(board, cfg.all);
    let (sub, mut res) = route_board(&merged.board, sub_cfg, base, search);
    let real = if res.offsets.is_empty() {
        board.clone()
    } else {
        board.with_offsets(&res.offsets)
    };
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
    let Split {
        geo,
        mut reports,
        unfinished,
        mut paths,
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
    // a repaired end that finds no way keeps the two tracks it had: completion comes first
    let split_geo = geo;
    let mut keep: Vec<usize> = Vec::new();
    let (geo_done, fin) = loop {
        let mut geo = split_geo.clone();
        let (repaired, cuts) = repair(&real, &mut geo, &paths, &keep);
        let mut names = unfinished.clone();
        names.extend(repaired);
        names.sort();
        names.dedup();
        if names.is_empty() {
            break (geo, None);
        }
        let mut b = real.only_nets(&names, &Default::default());
        let kept_from = b.obstacles.len();
        lock_geometry(&mut b, &geo);
        let cut_map = cut_nets(&mut b, &cuts, kept_from);
        let (_, mut fin) = route_board(&b, sub_cfg, base, &finish);
        let failed: Vec<usize> = fin
            .stats
            .unrouted
            .iter()
            .filter_map(|u| {
                cuts.iter()
                    .enumerate()
                    .find(|(i, _)| u.contains(&format!("~cut{i} from ")))
            })
            .map(|(_, c)| c.path)
            .collect();
        if !failed.is_empty() {
            keep.extend(failed);
            continue;
        }
        for t in fin.geometry.tracks.iter_mut() {
            t.net = cut_map.get(&t.net).copied().unwrap_or(t.net);
        }
        for v in fin.geometry.vias.iter_mut() {
            v.net = cut_map.get(&v.net).copied().unwrap_or(v.net);
        }
        break (geo, Some(fin));
    };
    let mut geo = geo_done;
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

    #[test]
    fn a_pair_becomes_two_parallel_tracks_of_equal_length() {
        let b = pair_board();
        assert_eq!(merge(&b, true).pairs.len(), 1);
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
        assert_eq!(rep.len(), 1);
        assert!(rep[0].wide && rep[0].note.is_empty(), "{:?}", rep[0]);
        assert!((rep[0].length_p - rep[0].length_n).abs() <= 0.1, "{:?}", rep[0]);
        assert_eq!(res.geometry.tracks.iter().filter(|t| t.net == 0).count(), 1);
        assert_eq!(res.geometry.tracks.iter().filter(|t| t.net == 1).count(), 1);
        assert!(crate::check::check(&b, &res.geometry).is_empty());
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
