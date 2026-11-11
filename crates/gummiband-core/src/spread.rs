//! Laying the tracks of a squeezed place that has room.
//!
//! [`crate::passage`] says of a finding of the check whether the tracks can give way there.
//! Where they can, the copper is laid again here, by construction and as one group:
//!
//! The passage gets a frame: x runs along the tracks at the finding, y across them (the cut of
//! the passage). Every free track of the route that runs along x through a window around the
//! findings of the spot is a **run**, as far along the track as it keeps that direction, to
//! `REACH` beside the window. Everything else is fixed: pads, holes, rule areas, the edge,
//! vias, kept copper, the exit pieces of pads, tracks of pairs and of nets with a length to
//! keep, the other parts of the tracks, and the points where a run is held (its ends, its
//! joints with other tracks, its vias).
//!
//! The runs are laid from one side outwards. The first keeps the clearance and [`MARGIN`] to
//! what is fixed on that side, the next to the one before, and so on. On the other side a run
//! stays below every fixed thing by what the runs still to come need between them: their
//! widths, their clearances and a margin each. Between those two bounds a run is a taut
//! string: a line where nothing is in its way and an arc where it passes something round (a
//! via, the corner of a pad, the arc of the run before it, which gives the runs of a bundle
//! arcs around one centre). A run that lies within its bounds stays as it is; one that does
//! not is laid again where it is out of them and a little beyond, and joins its old copper
//! there. Held points do not move.
//!
//! Where the bounds leave a run less than it needs, the margin goes down from [`MARGIN`] to
//! the largest one that fits, the same for every gap: with little room to spare the room is
//! shared out evenly. Below [`LEAST`] the passage is not laid.
//!
//! Where a held point is nearer to something than the margin asks (two exit pieces at the
//! least pitch have the clearance to each other and nothing more), the run goes on straight
//! from it without getting nearer until it is far enough, and meets its arc there with a
//! corner; on that stretch the margin gives way, the clearance does not (`straight_on`).
//!
//! The group is laid from both sides, and the side with the larger margin, then with fewer
//! tracks to lay again, wins. [`spread`] takes the result only if the own check finds nothing
//! new on the whole board and the worst finding of the spot is gone; otherwise all tracks keep
//! their copper.

use std::f64::consts::PI;

use crate::board::{Board, NetId, ObstacleKind};
use crate::check::{self, Violation};
use crate::geom::{Point, Rect, Shape, arc_from_three, arc_polyline, point_seg_dist, pt};
use crate::passage::{self, Kind, Passage};
use crate::realize::{Elem, Geometry, Track, flatten};

/// What a laid piece keeps on top of every clearance, in mm.
pub const MARGIN: f64 = 0.002;
/// The least margin a passage is laid with: below it the check of the file may see it as too
/// near (KiCad compares to about half a micrometre).
pub const LEAST: f64 = 0.0005;
/// How far beyond the pieces of the findings of a spot the window reaches along the tracks, and
/// how far from the worst finding a track can be a run, in mm.
const AHEAD: f64 = 0.3;
const ACROSS: f64 = 2.5;
/// How far beside the window a track is still a run, in mm.
const REACH: f64 = 2.0;
/// The stations along the tracks at which the bounds are taken, in mm apart at most.
const GRID: f64 = 0.025;
/// How far beyond the place where it is out of its bounds a run is laid again, in mm.
const JOIN: f64 = 0.3;
/// A piece runs along the passage if its direction is within 60 degrees of it.
const SLANT: f64 = 0.5;
/// An arc that leaves its chord by less than this is a corner, in mm.
const FLAT: f64 = 0.0002;
const EPS: f64 = 1e-7;

/// Why a passage was not laid.
#[derive(Debug, Clone, PartialEq)]
pub enum Why {
    /// No track of the route runs along the passage at the finding.
    NoRun,
    /// The tracks do not lie in one order along the passage.
    Tangle,
    /// Laid from either side, a track gets less than it needs somewhere: by how much, and
    /// where. The classifier saw room on its cut; there is none beside it.
    NoRoom { missing: f64, at: Point },
    /// Every track lies within its bounds: what is too near is held (an end, a joint, a via).
    Held,
    /// The check finds something new afterwards (how many), or the finding is still there.
    Check { new: usize, still: bool },
}

impl std::fmt::Display for Why {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Why::NoRun => write!(f, "no track runs along the passage"),
            Why::Tangle => write!(f, "the tracks are not in one order"),
            Why::NoRoom { missing, at } => write!(
                f,
                "{:.1} um missing beside the cut at ({:.3}, {:.3})",
                missing * 1e3,
                at.x,
                at.y
            ),
            Why::Held => write!(f, "what is too near is held"),
            Why::Check { new, still } => write!(
                f,
                "the check: {new} new{}",
                if *still { ", the finding still there" } else { "" }
            ),
        }
    }
}

/// A group of tracks laid again.
#[derive(Debug, Clone)]
pub struct Laid {
    pub geo: Geometry,
    /// The tracks with new copper.
    pub tracks: Vec<usize>,
    /// The margin every gap got.
    pub margin: f64,
    /// The stretches that are lines from station to station instead of lines and arcs.
    pub rough: usize,
}

/// A spot that kept its copper.
#[derive(Debug, Clone)]
pub struct Back {
    pub what: String,
    pub at: Point,
    pub pieces: usize,
    pub why: Why,
}

/// What [`spread`] did.
#[derive(Debug, Clone, Default)]
pub struct Report {
    /// Spots where the tracks can give way that were tried.
    pub tried: usize,
    /// Of them laid and taken.
    pub laid: usize,
    /// Tracks with new copper, each counted per group.
    pub tracks: usize,
    /// The least margin a group was laid with.
    pub margin: Option<f64>,
    /// Stretches of them that are lines from station to station instead of lines and arcs.
    pub rough: usize,
    /// The spots that kept their copper, and why.
    pub back: Vec<Back>,
}

/// The frame of a passage: `x` along the tracks, `y` across them.
#[derive(Clone, Copy)]
struct Frame {
    o: Point,
    u: Point,
    n: Point,
}

impl Frame {
    fn new(o: Point, n: Point) -> Frame {
        Frame { o, u: pt(n.y, -n.x), n }
    }
    fn to(&self, p: Point) -> Point {
        let d = p - self.o;
        pt(d.dot(self.u), d.dot(self.n))
    }
    fn from(&self, l: Point) -> Point {
        self.o + self.u * l.x + self.n * l.y
    }
}

/// A centre line in the frame.
#[derive(Clone, Copy, Debug)]
enum Form {
    Seg(Point, Point),
    Arc {
        c: Point,
        rad: f64,
        a: Point,
        b: Point,
        sa: f64,
        sweep: f64,
    },
}

impl Form {
    fn of(f: &Frame, e: &Elem) -> Form {
        match *e {
            Elem::Line(a, b) => Form::Seg(f.to(a), f.to(b)),
            Elem::Arc(a, m, b) => {
                let (a, m, b) = (f.to(a), f.to(m), f.to(b));
                match arc_from_three(a, m, b) {
                    Some((c, rad, sa, sweep)) => Form::Arc {
                        c,
                        rad,
                        a,
                        b,
                        sa,
                        sweep,
                    },
                    None => Form::Seg(a, b),
                }
            }
        }
    }

    fn ends(&self) -> (Point, Point) {
        match *self {
            Form::Seg(a, b) | Form::Arc { a, b, .. } => (a, b),
        }
    }

    fn mid(&self) -> Point {
        match *self {
            Form::Seg(a, b) => a.lerp(b, 0.5),
            Form::Arc { c, rad, sa, sweep, .. } => c + pt(rad, 0.0).rotate(sa + sweep / 2.0),
        }
    }

    /// Seen from the other side: y turned round.
    fn flipped(&self) -> Form {
        let m = |p: Point| pt(p.x, -p.y);
        match *self {
            Form::Seg(a, b) => Form::Seg(m(a), m(b)),
            Form::Arc {
                c,
                rad,
                a,
                b,
                sa,
                sweep,
            } => Form::Arc {
                c: m(c),
                rad,
                a: m(a),
                b: m(b),
                sa: -sa,
                sweep: -sweep,
            },
        }
    }

    /// The distance from p to it.
    fn dist(&self, p: Point) -> f64 {
        match *self {
            Form::Seg(a, b) => point_seg_dist(p, a, b),
            Form::Arc {
                c,
                rad,
                a,
                b,
                sa,
                sweep,
                ..
            } => {
                let mut d = crate::math::atan2((p - c).y, (p - c).x) - sa;
                let turn = 2.0 * PI;
                if sweep >= 0.0 {
                    d = d.rem_euclid(turn);
                } else {
                    d = -((-d).rem_euclid(turn));
                }
                if d.abs() <= sweep.abs() {
                    (p.dist(c) - rad).abs()
                } else {
                    p.dist(a).min(p.dist(b))
                }
            }
        }
    }

    /// The x range it covers, widened by `by`.
    fn reach(&self, by: f64) -> (f64, f64) {
        match *self {
            Form::Seg(a, b) => (a.x.min(b.x) - by, a.x.max(b.x) + by),
            Form::Arc { c, rad, .. } => (c.x - rad - by, c.x + rad + by),
        }
    }
}

/// A circle a taut track lies on: the centre of something round and the distance it keeps.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Round {
    c: Point,
    rho: f64,
}

/// What a bound is made of at a station.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Bind {
    /// Nothing a track can turn around: the inside of an arc.
    None,
    Round(Round),
    /// The straight side of something, between its two round ends.
    Line(Round, Round),
}

/// The upper edge, at `x`, of everything within `rho` of the form, and what it is made of.
/// `caps`: the form ends there (a piece of a longer line has no round end of its own).
fn top(form: &Form, caps: [bool; 2], rho: f64, x: f64) -> Option<(f64, Bind)> {
    let mut best: Option<(f64, Bind)> = None;
    let mut offer = |y: f64, b: Bind| {
        if best.is_none_or(|o| y > o.0) {
            best = Some((y, b));
        }
    };
    let (a, b) = form.ends();
    for (p, on) in [(a, caps[0]), (b, caps[1])] {
        let d = x - p.x;
        if on && d.abs() < rho {
            offer(p.y + (rho * rho - d * d).sqrt(), Bind::Round(Round { c: p, rho }));
        }
    }
    match *form {
        Form::Seg(a, b) => {
            let e = b - a;
            let len = e.len();
            if len > 1e-12 {
                let mut nu = e.perp() * (1.0 / len);
                if nu.y < 0.0 {
                    nu = -nu;
                }
                let (a2, b2) = (a + nu * rho, b + nu * rho);
                if (b2.x - a2.x).abs() > 1e-12 && (x - a2.x) * (x - b2.x) <= 0.0 {
                    let t = (x - a2.x) / (b2.x - a2.x);
                    let (first, second) = if a.x <= b.x { (a, b) } else { (b, a) };
                    offer(
                        a2.y + (b2.y - a2.y) * t,
                        Bind::Line(Round { c: first, rho }, Round { c: second, rho }),
                    );
                }
            }
        }
        Form::Arc { c, rad, sa, sweep, .. } => {
            let within = |p: Point| {
                let mut d = crate::math::atan2((p - c).y, (p - c).x) - sa;
                if sweep >= 0.0 {
                    while d < -1e-9 {
                        d += 2.0 * PI;
                    }
                    while d > 2.0 * PI {
                        d -= 2.0 * PI;
                    }
                    d <= sweep + 1e-9
                } else {
                    while d > 1e-9 {
                        d -= 2.0 * PI;
                    }
                    while d < -2.0 * PI {
                        d += 2.0 * PI;
                    }
                    d >= sweep - 1e-9
                }
            };
            let d = x - c.x;
            let outer = rad + rho;
            if d.abs() < outer {
                let p = pt(x, c.y + (outer * outer - d * d).sqrt());
                // the copper is at the point of the arc itself, straight below
                if within(c + (p - c) * (rad / outer)) {
                    offer(p.y, Bind::Round(Round { c, rho: outer }));
                }
            }
            let inner = rad - rho;
            if inner > 0.0 && d.abs() < inner {
                let p = pt(x, c.y - (inner * inner - d * d).sqrt());
                if within(c + (p - c) * (rad / inner)) {
                    offer(p.y, Bind::None);
                }
            }
        }
    }
    best
}

/// The lower edge of the same.
fn bottom(form: &Form, caps: [bool; 2], rho: f64, x: f64) -> Option<(f64, Bind)> {
    let m = |r: Round| Round {
        c: pt(r.c.x, -r.c.y),
        rho: r.rho,
    };
    top(&form.flipped(), caps, rho, x).map(|(y, b)| {
        (
            -y,
            match b {
                Bind::None => Bind::None,
                Bind::Round(r) => Bind::Round(m(r)),
                Bind::Line(p, q) => Bind::Line(m(p), m(q)),
            },
        )
    })
}

/// Something that does not move.
struct Feat {
    form: Form,
    /// The ends of the whole side a piece is of, ascending in x.
    whole: (Point, Point),
    caps: [bool; 2],
    /// Half its width (0: an outline).
    r: f64,
    kind: Kind,
    net: Option<NetId>,
    obstacle: Option<usize>,
}

/// A track of the route where it runs along the passage.
struct Run {
    ti: usize,
    net: NetId,
    /// The elements of the track in the span, first and last.
    e: (usize, usize),
    /// The track runs against x there.
    rev: bool,
    /// The span in x.
    span: (f64, f64),
    /// The old path in the span, ascending in x, and the widths of the pieces between.
    v: Vec<Point>,
    w: Vec<f64>,
    /// The arcs of the old path, as x ranges: a run is laid again over a whole arc or not at
    /// all of it.
    arcs: Vec<(f64, f64)>,
    /// Places inside the span where something of its net ends on it.
    held: Vec<f64>,
    /// Stations of the span, first and last, and per station from the first: the old path,
    /// the widest piece there, and whether the run is held.
    k: (usize, usize),
    x0: Vec<f64>,
    wk: Vec<f64>,
    pin: Vec<bool>,
    corner: Vec<bool>,
    /// The fixed things below with the distance to keep without margin and half width, and
    /// those above with the number of margins that go on top.
    below: Vec<(usize, f64)>,
    above: Vec<(usize, f64, f64)>,
}

impl Run {
    /// The old path at x (clamped to the span).
    fn at(&self, x: f64) -> f64 {
        let v = &self.v;
        if x <= v[0].x {
            return v[0].y;
        }
        let i = v.partition_point(|p| p.x < x);
        if i >= v.len() {
            return v[v.len() - 1].y;
        }
        let (p, q) = (v[i - 1], v[i]);
        if q.x - p.x < 1e-12 {
            return q.y;
        }
        p.y + (q.y - p.y) * (x - p.x) / (q.x - p.x)
    }

    /// The width of the old path at x: the wider piece at a joint.
    fn width(&self, x: f64) -> f64 {
        let x = x.clamp(self.v[0].x, self.v[self.v.len() - 1].x);
        self.v
            .windows(2)
            .zip(&self.w)
            .filter(|(p, _)| p[0].x - 1e-9 <= x && x <= p[1].x + 1e-9)
            .map(|(_, w)| *w)
            .fold(0.0, f64::max)
    }

    /// The width of a new piece from x1 to x2: that of the old piece at its middle.
    fn width_between(&self, x1: f64, x2: f64) -> f64 {
        let mid = ((x1 + x2) / 2.0).clamp(self.v[0].x, self.v[self.v.len() - 1].x);
        self.v
            .windows(2)
            .zip(&self.w)
            .find(|(v, _)| v[0].x <= mid && mid <= v[1].x)
            .map_or(self.w[0], |(_, w)| *w)
    }

    fn has(&self, x: f64) -> bool {
        self.span.0 - 1e-9 <= x && x <= self.span.1 + 1e-9
    }
}

/// The index ranges of the points of each element of a track, or None if the points are not
/// those of the elements.
fn ranges(t: &Track) -> Option<Vec<(usize, usize)>> {
    let mut out = Vec::with_capacity(t.elems.len());
    let mut at = 0usize;
    for (i, e) in t.elems.iter().enumerate() {
        let n = match *e {
            Elem::Line(..) => 2,
            Elem::Arc(a, m, b) => arc_polyline(a, m, b, 0.001).len(),
        };
        let (a, b) = match *e {
            Elem::Line(a, b) | Elem::Arc(a, _, b) => (a, b),
        };
        if i == 0 {
            if t.pts.first() != Some(&a) {
                return None;
            }
        } else if t.pts.get(at) != Some(&a) {
            return None;
        }
        let end = at + n - 1;
        if t.pts.get(end) != Some(&b) {
            return None;
        }
        out.push((at, end));
        at = end;
    }
    (at + 1 == t.pts.len()).then_some(out)
}

fn insert(list: &mut Vec<f64>, x: f64, tol: f64) {
    let i = list.partition_point(|s| *s < x);
    let near = |j: usize| list.get(j).is_some_and(|s| (s - x).abs() < tol);
    if near(i) || (i > 0 && near(i - 1)) {
        return;
    }
    list.insert(i, x);
}

fn station(list: &[f64], x: f64) -> usize {
    let i = list.partition_point(|s| *s < x);
    if i == 0 {
        0
    } else if i >= list.len() {
        list.len() - 1
    } else if x - list[i - 1] <= list[i] - x {
        i - 1
    } else {
        i
    }
}

/// Where a taut string touches.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Side {
    Pin,
    Lo,
    Hi,
}

/// The shortest way from the first station to the last between `lo` and `hi` (equal at both
/// ends): the stations where it touches a bound, in order, the ends included.
fn taut(x: &[f64], lo: &[f64], hi: &[f64]) -> Vec<(usize, Side)> {
    let n = x.len();
    let mut out = vec![(0, Side::Pin)];
    if n < 2 {
        return out;
    }
    let mut apex = (0usize, lo[0]);
    let (mut up, mut dn) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut ui, mut di) = (0usize, 0usize);
    let mut k = 1;
    while k < n {
        let dx = x[k] - x[apex.0];
        let (su, sl) = ((hi[k] - apex.1) / dx, (lo[k] - apex.1) / dx);
        if su < up {
            if su < dn {
                apex = (di, lo[di]);
                out.push((di, Side::Lo));
                k = di + 1;
                (up, dn) = (f64::INFINITY, f64::NEG_INFINITY);
                continue;
            }
            up = su;
            ui = k;
        }
        if sl > dn {
            if sl > up {
                apex = (ui, hi[ui]);
                out.push((ui, Side::Hi));
                k = ui + 1;
                (up, dn) = (f64::INFINITY, f64::NEG_INFINITY);
                continue;
            }
            dn = sl;
            di = k;
        }
        k += 1;
    }
    out.push((n - 1, Side::Pin));
    out
}

/// A thing a taut string passes: a point it goes through, or a circle it lies on (q > 0: the
/// circle is below, on the right of the way; q < 0: above).
#[derive(Clone, Copy, Debug, PartialEq)]
struct Node {
    c: Point,
    q: f64,
}

/// The line that leaves node a and arrives at node b, touching both.
fn tangent(a: Node, b: Node) -> Option<(Point, Point)> {
    let d = b.c - a.c;
    let l = d.len();
    if l < 1e-9 {
        return None;
    }
    let k = (a.q - b.q) / l;
    // (two circles that touch from inside, as the arc of a track and its round end do, have
    // the one line at the point where they touch)
    if k.abs() > 1.0 + 1e-6 {
        return None;
    }
    let k = k.clamp(-1.0, 1.0);
    let e = d * (1.0 / l);
    let nu = e * k + e.perp() * (1.0 - k * k).sqrt();
    Some((a.c + nu * a.q, b.c + nu * b.q))
}

/// The lines and arcs through the nodes, in the frame. None: they do not make a way.
fn string(nodes: &[Node]) -> Option<Vec<Elem>> {
    // two circles about one centre on one side (a thing seen from two stations with two widths
    // of the track) are the larger one
    let mut all: Vec<Node> = Vec::with_capacity(nodes.len());
    for &n in nodes {
        match all.last_mut() {
            Some(l) if l.c.dist(n.c) < 1e-9 && l.q * n.q > 0.0 => l.q = if n.q.abs() > l.q.abs() { n.q } else { l.q },
            _ => all.push(n),
        }
    }
    let mut nodes = all;
    'again: loop {
        let mut touch: Vec<(Point, Point)> = Vec::with_capacity(nodes.len());
        // the circles met with a corner, by the index of the line that meets them
        let mut met: Vec<usize> = Vec::new();
        for i in 0..nodes.len() - 1 {
            let (a, b) = (nodes[i], nodes[i + 1]);
            if let Some(t) = tangent(a, b) {
                touch.push(t);
                continue;
            }
            met.push(i);
            // A point nearer to a circle than its radius (a held point with less than the
            // margin to something): no line from it touches the circle. The way goes on
            // straight, as it comes or else without getting nearer, and meets the circle with
            // a corner.
            let (point, circle, leaves) = match (a.q == 0.0, b.q == 0.0) {
                (true, false) => (a.c, b, true),
                (false, true) => (b.c, a, false),
                _ => return None,
            };
            let other = if leaves { i.checked_sub(1) } else { Some(i + 2) }
                .and_then(|j| nodes.get(j))
                .filter(|n| n.q == 0.0 && n.c.dist(point) > 1e-9);
            let r = point - circle.c;
            if r.len() < 1e-9 {
                return None;
            }
            let dir = match other {
                Some(n) if leaves => (point - n.c).norm(),
                Some(n) => (n.c - point).norm(),
                None => pt(r.y, -r.x).norm() * circle.q.signum(),
            };
            // from the point along the way (against it, if the way arrives there) to the circle
            let along = if leaves { dir } else { -dir };
            let m = r.dot(along);
            let s = -m + (m * m - (r.dot(r) - circle.q * circle.q)).max(0.0).sqrt();
            let meet = point + along * s;
            touch.push(if leaves { (point, meet) } else { (meet, point) });
        }
        // a circle the way turns around the wrong way is not in its way
        for i in 1..nodes.len() - 1 {
            let n = nodes[i];
            if n.q == 0.0 {
                continue;
            }
            let (tin, tout) = (touch[i - 1].1, touch[i].0);
            if (tin - n.c).cross(tout - n.c) * n.q > 1e-12 {
                // (where the way met it with a corner, it still goes on straight to there)
                if met.contains(&(i - 1)) {
                    nodes[i] = Node { c: tin, q: 0.0 };
                } else if met.contains(&i) {
                    nodes[i] = Node { c: tout, q: 0.0 };
                } else {
                    nodes.remove(i);
                }
                continue 'again;
            }
        }
        let mut out = Vec::new();
        let mut at = nodes[0].c;
        let line = |at: &mut Point, to: Point, out: &mut Vec<Elem>| {
            if at.dist(to) > 1e-9 {
                out.push(Elem::Line(*at, to));
                *at = to;
            }
        };
        for i in 1..nodes.len() {
            let (tin, n) = (touch[i - 1].1, nodes[i]);
            if i + 1 == nodes.len() || n.q == 0.0 {
                line(&mut at, tin, &mut out);
                continue;
            }
            let tout = touch[i].0;
            let rho = n.q.abs();
            let (a, b) = (tin - n.c, tout - n.c);
            let angle = crate::math::atan2(a.cross(b).abs(), a.dot(b));
            // an arc that leaves its chord by less than FLAT is a corner, where its tangents meet
            let half = crate::math::cos(angle / 2.0);
            if rho * (1.0 - half) < FLAT {
                line(&mut at, n.c + (a + b).norm() * (rho / half), &mut out);
                continue;
            }
            line(&mut at, tin, &mut out);
            let mid = n.c + (a + b).norm() * rho;
            out.push(Elem::Arc(tin, mid, tout));
            at = tout;
        }
        // lines on one line are one
        let mut joined: Vec<Elem> = Vec::with_capacity(out.len());
        for e in out {
            if let (Elem::Line(b, c), Some(&Elem::Line(a, b0))) = (e, joined.last())
                && b0 == b
                && (b - a).dot(c - b) > 0.0
                && point_seg_dist(b, a, c) < 1e-9
            {
                *joined.last_mut().unwrap() = Elem::Line(a, c);
                continue;
            }
            joined.push(e);
        }
        return Some(joined);
    }
}

/// A stretch on which a run goes on straight from a held point: from `x.0` to `x.1`, on the
/// line through `p` with slope `slope`.
struct Straight {
    p: Point,
    slope: f64,
    x: (f64, f64),
}

impl Straight {
    fn at(&self, x: f64) -> Option<f64> {
        (x >= self.x.0 - 1e-9 && x <= self.x.1 + 1e-9).then_some(self.p.y + (x - self.p.x) * self.slope)
    }
}

/// Where a held point of a run is nearer to something than the run is to keep (two exit pieces
/// side by side have the clearance to each other and no margin), the run cannot keep that
/// there. It goes on straight from the point without getting nearer, until it is far enough
/// from everything the point is too near to: the stretches of that line before and behind each
/// of `held`. `things`: the forms with the distance to keep.
fn straight_on(things: &[(Form, f64)], held: &[Point]) -> Vec<Straight> {
    let mut out = Vec::new();
    for &p in held {
        let near: Vec<&(Form, f64)> = things
            .iter()
            .filter(|(f, rho)| {
                let d = f.dist(p);
                d < rho - EPS && d > 1e-9
            })
            .collect();
        // the line across the way to the nearest point of the thing it is deepest in
        let Some((form, _)) = near
            .iter()
            .max_by(|a, b| (a.1 - a.0.dist(p)).total_cmp(&(b.1 - b.0.dist(p))))
        else {
            continue;
        };
        let foot = match *form {
            Form::Seg(a, b) => {
                let e = b - a;
                let t = if e.dot(e) < 1e-18 {
                    0.0
                } else {
                    ((p - a).dot(e) / e.dot(e)).clamp(0.0, 1.0)
                };
                a + e * t
            }
            Form::Arc { c, rad, a, b, .. } => {
                let on = c + (p - c).norm() * rad;
                if (form.dist(on)) < 1e-9 {
                    on
                } else if p.dist(a) <= p.dist(b) {
                    a
                } else {
                    b
                }
            }
        };
        let r = (p - foot).norm();
        let mut t = pt(r.y, -r.x);
        if t.x < 0.0 {
            t = -t;
        }
        if t.x < 0.2 {
            continue;
        }
        // how far along the line each way until all of them are far enough
        let reach = |sign: f64| -> f64 {
            let inside = |s: f64| near.iter().any(|(f, rho)| f.dist(p + t * (sign * s)) < rho - EPS);
            let step = 0.005;
            let mut s = 0.0;
            while s < 2.0 && inside(s + step) {
                s += step;
            }
            let (mut a, mut b) = (s, s + step);
            for _ in 0..30 {
                let m = (a + b) / 2.0;
                if inside(m) {
                    a = m;
                } else {
                    b = m;
                }
            }
            b
        };
        out.push(Straight {
            p,
            slope: t.y / t.x,
            x: (p.x - reach(-1.0) * t.x, p.x + reach(1.0) * t.x),
        });
    }
    out
}

/// y of a way of lines and arcs at x (the way goes on in x).
fn height(way: &[Point], x: f64) -> f64 {
    let i = way.partition_point(|p| p.x < x);
    if i == 0 {
        return way[0].y;
    }
    if i >= way.len() {
        return way[way.len() - 1].y;
    }
    let (p, q) = (way[i - 1], way[i]);
    if q.x - p.x < 1e-12 {
        return q.y;
    }
    p.y + (q.y - p.y) * (x - p.x) / (q.x - p.x)
}

/// The points of lines and arcs, fine enough to measure.
fn fine(elems: &[Elem]) -> Vec<Point> {
    let mut pts: Vec<Point> = Vec::new();
    for e in elems {
        let p = match *e {
            Elem::Line(a, b) => vec![a, b],
            Elem::Arc(a, m, b) => arc_polyline(a, m, b, 1e-5),
        };
        for q in p {
            if pts.last().is_none_or(|l| l.dist(q) > 1e-12) {
                pts.push(q);
            }
        }
    }
    pts
}

/// The fixed things around a passage and the runs through it, in the frame of one side.
struct Scene<'a> {
    board: &'a Board,
    frame: Frame,
    at: Point,
    feats: Vec<Feat>,
    runs: Vec<Run>,
    stations: Vec<f64>,
    /// The runs from the first side outwards, and for each run those right below it somewhere.
    order: Vec<usize>,
    lower: Vec<Vec<usize>>,
    /// Where along the tracks the runs are looked at: the findings of the spot are there.
    window: (f64, f64),
}

impl<'a> Scene<'a> {
    fn asked(&self, net: NetId, kind: Kind, other: Option<NetId>, obstacle: Option<usize>) -> f64 {
        let c = passage::asked(self.board, net, kind, other, obstacle);
        // a clearance of a custom courtyard rule holds for all of them there
        self.board
            .courtyard_clearance_at(self.at, c)
            .map_or(c, |min| c.min(min))
    }

    /// The scene of passage `p` with its window from `w.0` to `w.1` along the tracks.
    fn build(board: &'a Board, geo: &Geometry, p: &Passage, frame: Frame, w: (f64, f64)) -> Result<Scene<'a>, Why> {
        let layer = p.layer;
        let spot = frame.to(p.at);
        // the tracks: a run where one runs along the passage through the window
        let mut runs: Vec<Run> = Vec::new();
        let mut rest: Vec<(Form, f64, Kind, NetId)> = Vec::new();
        for (ti, t) in geo.tracks.iter().enumerate() {
            if t.layer != layer || t.pts.len() < 2 {
                continue;
            }
            let kind = if t.origin.is_some() { Kind::Kept } else { Kind::Track };
            let forms: Vec<Form> = t.elems.iter().map(|e| Form::of(&frame, e)).collect();
            let rg = ranges(t);
            // an exit piece, from the pin of a pad to its exit point, belongs to the pad
            let stub = t.seg == u32::MAX
                && t.elems.len() == 1
                && board
                    .pins
                    .iter()
                    .any(|p| p.net == t.net && (p.pos.dist(t.pts[0]) < 1e-9 || p.pos.dist(t.pts[1]) < 1e-9));
            // a track of a pair or of a net with a length to keep has its form from there
            let bound = board.in_diff_pair(t.net) || board.match_rules.iter().any(|r| r.nets.contains(&t.net));
            let free = t.origin.is_none() && t.pattern.is_none() && !t.resting && !stub && !bound;
            let run = rg
                .as_ref()
                .filter(|_| free)
                .and_then(|rg| Self::run(t, ti, &frame, &forms, rg, spot, w));
            let width = |i: usize| -> f64 {
                match (&rg, t.elems[i]) {
                    (Some(rg), _) => t.pts[rg[i].0..=rg[i].1]
                        .windows(2)
                        .map(|p| t.piece_width(p[0], p[1]))
                        .fold(0.0, f64::max),
                    (None, Elem::Line(a, b) | Elem::Arc(a, _, b)) => t.piece_width(a, b),
                }
            };
            for (i, f) in forms.iter().enumerate() {
                if !run.as_ref().is_some_and(|r| r.e.0 <= i && i <= r.e.1) {
                    rest.push((*f, width(i) / 2.0, kind, t.net));
                }
            }
            if let Some(mut run) = run {
                // where something of its net ends on it, it is held
                let ends = geo
                    .tracks
                    .iter()
                    .enumerate()
                    .filter(|(tj, o)| *tj != ti && o.net == t.net && o.layer == layer && o.pts.len() >= 2)
                    .flat_map(|(_, o)| [o.pts[0], *o.pts.last().unwrap()])
                    .chain(geo.vias.iter().filter(|v| v.net == t.net).map(|v| v.at))
                    .chain(board.pins.iter().filter(|p| p.net == t.net).map(|p| p.pos));
                for e in ends {
                    let l = frame.to(e);
                    if l.x > run.span.0 + 1e-6
                        && l.x < run.span.1 - 1e-6
                        && run.v.windows(2).any(|p| point_seg_dist(l, p[0], p[1]) < 1e-6)
                    {
                        run.held.push(l.x);
                    }
                }
                runs.push(run);
            }
        }
        if runs.is_empty() {
            return Err(Why::NoRun);
        }
        // what does not move, as far around the runs as anything can matter to them
        let mut around = Rect::empty();
        runs.iter().flat_map(|r| &r.v).for_each(|p| around.add(*p));
        let around = around.grow(ACROSS);
        let reach = |f: &Form| {
            let (a, b) = f.ends();
            let mut bb = Rect::empty();
            bb.add(a);
            bb.add(b);
            if let Form::Arc { c, rad, .. } = *f {
                bb.add(c);
                bb = bb.grow(rad);
            }
            bb.overlaps(&around)
        };
        let mut feats: Vec<Feat> = Vec::new();
        // a long straight side is taken in pieces: what lies between it and a run differs
        // along it
        let mut add = |form: Form, r: f64, kind: Kind, net: Option<NetId>, obstacle: Option<usize>| match form {
            Form::Seg(a, b) if a.dist(b) > 0.2 => {
                let n = (a.dist(b) / 0.2).ceil() as usize;
                for i in 0..n {
                    let piece = Form::Seg(a.lerp(b, i as f64 / n as f64), a.lerp(b, (i + 1) as f64 / n as f64));
                    if reach(&piece) {
                        feats.push(Feat {
                            form: piece,
                            whole: if a.x <= b.x { (a, b) } else { (b, a) },
                            caps: [i == 0, i + 1 == n],
                            r,
                            kind,
                            net,
                            obstacle,
                        });
                    }
                }
            }
            form => {
                if reach(&form) {
                    let (a, b) = form.ends();
                    feats.push(Feat {
                        form,
                        whole: if a.x <= b.x { (a, b) } else { (b, a) },
                        caps: [true, true],
                        r,
                        kind,
                        net,
                        obstacle,
                    });
                }
            }
        };
        for (oi, ob) in board.obstacles.iter().enumerate() {
            let Some(kind) = passage::cut_kind(&ob.kind) else {
                continue;
            };
            if !(ob.layers.has(layer) || ob.kind == ObstacleKind::Hole) {
                continue;
            }
            match &ob.shape {
                Shape::Circle { c, r } => {
                    let c = frame.to(*c);
                    add(Form::Seg(c, c), *r, kind, ob.net, Some(oi));
                }
                Shape::Capsule { a, b, r } => add(Form::Seg(frame.to(*a), frame.to(*b)), *r, kind, ob.net, Some(oi)),
                Shape::Polygon(poly) => {
                    for (a, b) in poly.edges() {
                        add(Form::Seg(frame.to(a), frame.to(b)), 0.0, kind, ob.net, Some(oi));
                    }
                }
            }
        }
        for v in geo.vias.iter().filter(|v| v.drill > 0.0) {
            let kind = if v.origin.is_some() { Kind::Kept } else { Kind::Via };
            let (a, b) = (frame.to(v.at - v.half), frame.to(v.at + v.half));
            add(
                Form::Seg(a, b),
                v.diameter / 2.0 - v.half.len(),
                kind,
                Some(v.net),
                None,
            );
        }
        for (a, b) in board.outline.iter().flat_map(|o| o.edges()) {
            add(Form::Seg(frame.to(a), frame.to(b)), 0.0, Kind::Edge, None, None);
        }
        for (f, r, kind, net) in rest {
            add(f, r, kind, Some(net), None);
        }
        // a run is held at the ends of its span and at its joints: fixed for the others
        for r in &runs {
            let xs = [r.span.0, r.span.1].into_iter().chain(r.held.iter().copied());
            for x in xs {
                let c = pt(x, r.at(x));
                feats.push(Feat {
                    form: Form::Seg(c, c),
                    whole: (c, c),
                    caps: [true, true],
                    r: r.width(x) / 2.0,
                    kind: Kind::Track,
                    net: Some(r.net),
                    obstacle: None,
                });
            }
        }
        // the stations: every corner of a run, every end of a fixed thing, and a grid
        let (first, last) = runs.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), r| {
            (a.min(r.span.0), b.max(r.span.1))
        });
        let mut stations: Vec<f64> = Vec::new();
        for r in &runs {
            for x in r.v.iter().map(|p| p.x).chain(r.held.iter().copied()) {
                insert(&mut stations, x, 1e-6);
            }
        }
        for f in &feats {
            let (a, b) = f.form.ends();
            let mut xs = vec![a.x, b.x];
            if let Form::Arc { c, .. } = f.form {
                xs.push(c.x);
            }
            for x in xs {
                if x > first && x < last {
                    insert(&mut stations, x, 1e-4);
                }
            }
        }
        let n = ((last - first) / GRID).ceil() as usize;
        for i in 1..n {
            insert(&mut stations, first + (last - first) * i as f64 / n as f64, GRID / 3.0);
        }
        for r in &mut runs {
            r.k = (station(&stations, r.span.0), station(&stations, r.span.1));
            let xs = &stations[r.k.0..=r.k.1];
            r.x0 = xs.iter().map(|&x| r.at(x)).collect();
            r.wk = xs.iter().map(|&x| r.width(x)).collect();
            r.pin = xs
                .iter()
                .map(|&x| r.held.iter().any(|h| (h - x).abs() < 1e-6))
                .collect();
            r.pin[0] = true;
            *r.pin.last_mut().unwrap() = true;
            r.corner = xs.iter().map(|&x| r.v.iter().any(|p| (p.x - x).abs() < 1e-6)).collect();
        }
        // the order across: from the first side outwards
        let mut lower: Vec<Vec<usize>> = vec![Vec::new(); runs.len()];
        for k in 0..stations.len() {
            let mut here: Vec<(f64, usize)> = runs
                .iter()
                .enumerate()
                .filter(|(_, r)| r.k.0 <= k && k <= r.k.1)
                .map(|(i, r)| (r.x0[k - r.k.0], i))
                .collect();
            here.sort_by(|a, b| a.0.total_cmp(&b.0));
            // right below a run: the next one of another net, and those of that net with it
            // (two tracks of one net may lie on each other, they have no order)
            for (at, &(_, i)) in here.iter().enumerate() {
                let mut net = None;
                for &(_, j) in here[..at].iter().rev() {
                    let n = runs[j].net;
                    if n == runs[i].net {
                        continue;
                    }
                    if net.is_some_and(|m| m != n) {
                        break;
                    }
                    net = Some(n);
                    if !lower[i].contains(&j) {
                        lower[i].push(j);
                    }
                }
            }
        }
        let mut order: Vec<usize> = Vec::new();
        let mut done = vec![false; runs.len()];
        while order.len() < runs.len() {
            let Some(i) = (0..runs.len()).find(|&i| !done[i] && lower[i].iter().all(|&j| done[j])) else {
                return Err(Why::Tangle);
            };
            done[i] = true;
            order.push(i);
        }
        let mut scene = Scene {
            board,
            frame,
            at: p.at,
            feats,
            runs,
            stations,
            order,
            lower,
            window: w,
        };
        scene.bounds();
        Ok(scene)
    }

    /// The run of track `t` through the window, if it has one: the elements around the one
    /// nearest to the spot that go on along x.
    fn run(
        t: &Track,
        ti: usize,
        frame: &Frame,
        forms: &[Form],
        rg: &[(usize, usize)],
        spot: Point,
        w: (f64, f64),
    ) -> Option<Run> {
        let local: Vec<Point> = t.pts.iter().map(|p| frame.to(*p)).collect();
        // the direction an element goes in x, if all of it goes one way and along the passage
        let way = |i: usize| -> Option<f64> {
            let pts = &local[rg[i].0..=rg[i].1];
            let dir = (pts[pts.len() - 1].x - pts[0].x).signum();
            // (a steep piece that goes across by less than the width of the track is a jog in
            // it, no turn away from the passage)
            let jog =
                matches!(t.elems[i], Elem::Line(a, b) if (pts[pts.len() - 1].y - pts[0].y).abs() < t.piece_width(a, b));
            pts.windows(2)
                .all(|p| {
                    let d = p[1] - p[0];
                    d.x * dir > 0.0 && (jog || d.x.abs() >= SLANT * d.len())
                })
                .then_some(dir)
        };
        let inside = |i: usize| {
            let (a, b) = forms[i].ends();
            a.x.max(b.x) > w.0 + 1e-9 && a.x.min(b.x) < w.1 - 1e-9
        };
        let dist = |i: usize| {
            local[rg[i].0..=rg[i].1]
                .windows(2)
                .map(|p| point_seg_dist(spot, p[0], p[1]))
                .fold(f64::INFINITY, f64::min)
        };
        let seed = (0..forms.len())
            .filter(|&i| way(i).is_some() && inside(i))
            .map(|i| (dist(i), i))
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .filter(|s| s.0 <= ACROSS)?
            .1;
        let dir = way(seed)?;
        let (mut e0, mut e1) = (seed, seed);
        // on along the track as far as it goes that way, to REACH beside the window: the ends
        // of a run are held, and what is held should be far from what is too near
        let near = |i: usize| {
            let (a, b) = forms[i].ends();
            a.x.max(b.x) > w.0 - REACH && a.x.min(b.x) < w.1 + REACH
        };
        while e0 > 0 && way(e0 - 1) == Some(dir) && near(e0 - 1) {
            e0 -= 1;
        }
        while e1 + 1 < forms.len() && way(e1 + 1) == Some(dir) && near(e1 + 1) {
            e1 += 1;
        }
        // ascending in x
        let mut v: Vec<Point> = local[rg[e0].0..=rg[e1].1].to_vec();
        let mut wd: Vec<f64> = t.pts[rg[e0].0..=rg[e1].1]
            .windows(2)
            .map(|p| t.piece_width(p[0], p[1]))
            .collect();
        if dir < 0.0 {
            v.reverse();
            wd.reverse();
        }
        let n = v.len();
        let span = (v[0].x, v[n - 1].x);
        if span.1 - span.0 < 1e-3 {
            return None;
        }
        let arcs = (e0..=e1)
            .filter(|&i| matches!(forms[i], Form::Arc { .. }))
            .map(|i| {
                let (a, b) = (local[rg[i].0], local[rg[i].1]);
                (a.x.min(b.x), a.x.max(b.x))
            })
            .collect();
        Some(Run {
            ti,
            net: t.net,
            e: (e0, e1),
            rev: dir < 0.0,
            span,
            v,
            w: wd,
            arcs,
            held: Vec::new(),
            k: (0, 0),
            x0: Vec::new(),
            wk: Vec::new(),
            pin: Vec::new(),
            corner: Vec::new(),
            below: Vec::new(),
            above: Vec::new(),
        })
    }

    /// For every run the fixed things below it and above it, with the distance each asks: below
    /// its own clearance, above also what the runs between the two need.
    fn bounds(&mut self) {
        for i in 0..self.runs.len() {
            let r = &self.runs[i];
            let (mut below, mut above) = (Vec::new(), Vec::new());
            for (fi, f) in self.feats.iter().enumerate() {
                if f.net == Some(r.net) && f.kind != Kind::Hole {
                    continue;
                }
                let mid = f.form.mid();
                let x = mid.x.clamp(r.span.0, r.span.1);
                let y = r.at(x);
                let d = mid.y - y;
                if d.abs() < 1e-9 {
                    continue;
                }
                if d < 0.0 {
                    let c = self.asked(r.net, f.kind, f.net, f.obstacle);
                    if c > 0.0 && -d - f.r - c - r.width(x) / 2.0 < 0.5 {
                        below.push((fi, f.r + c));
                    }
                    continue;
                }
                // the runs between, from this one outwards
                let mut between: Vec<(f64, usize)> = self
                    .runs
                    .iter()
                    .enumerate()
                    .filter(|(j, o)| *j != i && o.net != r.net && f.net != Some(o.net) && o.has(x))
                    .map(|(j, o)| (o.at(x), j))
                    .filter(|(oy, _)| *oy > y && *oy < mid.y)
                    .collect();
                between.sort_by(|a, b| a.0.total_cmp(&b.0));
                let (mut total, mut gaps, mut prev) = (0.0, 0.0, r.net);
                for (_, j) in &between {
                    let o = &self.runs[*j];
                    let c = self.asked(prev, Kind::Track, Some(o.net), None);
                    total += c + o.width(x);
                    gaps += if c > 0.0 { 1.0 } else { 0.0 };
                    prev = o.net;
                }
                let c = self.asked(prev, f.kind, f.net, f.obstacle);
                if c <= 0.0 {
                    continue;
                }
                total += c + f.r;
                gaps += 1.0;
                if d - total - r.width(x) / 2.0 < 0.5 {
                    above.push((fi, total, gaps));
                }
            }
            self.runs[i].below = below;
            self.runs[i].above = above;
        }
    }
}

/// A run with its new copper: the elements in the frame that replace its old ones from
/// `from` to `to` in x.
struct New {
    from: f64,
    to: f64,
    elems: Vec<Elem>,
}

/// One laying from one side with one margin.
struct Try {
    /// The tracks with new copper.
    tracks: Vec<(usize, Track)>,
    /// The stretches laid as lines from station to station: lines and arcs did not fit there.
    rough: usize,
}

impl Scene<'_> {
    /// The bounds of run `i` at its stations with `margin` on every clearance, and what they are
    /// made of: below what is fixed there and the runs right below it (as `tracks` has them, else
    /// as they lie), above what is fixed there less what the runs between need.
    fn limits(
        &self,
        geo: &Geometry,
        i: usize,
        margin: f64,
        tracks: &[(usize, Track)],
    ) -> (Vec<f64>, Vec<f64>, Vec<Bind>, Vec<Bind>) {
        let xs = &self.stations;
        let r = &self.runs[i];
        let n = r.k.1 - r.k.0 + 1;
        let mut under: Vec<(Form, f64)> = Vec::new();
        for &j in &self.lower[i] {
            let o = &self.runs[j];
            let c = self.asked(r.net, Kind::Track, Some(o.net), None);
            if c <= 0.0 {
                continue;
            }
            let t = tracks
                .iter()
                .find(|(ti, _)| *ti == o.ti)
                .map_or(&geo.tracks[o.ti], |(_, t)| t);
            let rg = ranges(t);
            // the elements of the run only: the rest of the track is among the fixed things
            let last = t.elems.len() - (geo.tracks[o.ti].elems.len() - o.e.1);
            for (ei, e) in t.elems.iter().enumerate().take(last + 1).skip(o.e.0) {
                let f = Form::of(&self.frame, e);
                let (a, b) = f.reach(1.0);
                if b < r.span.0 || a > r.span.1 {
                    continue;
                }
                let w = match (&rg, *e) {
                    (Some(rg), _) => t.pts[rg[ei].0..=rg[ei].1]
                        .windows(2)
                        .map(|p| t.piece_width(p[0], p[1]))
                        .fold(0.0, f64::max),
                    (None, Elem::Line(a, b) | Elem::Arc(a, _, b)) => t.piece_width(a, b),
                };
                under.push((f, w / 2.0 + c));
            }
        }
        let (mut lo, mut hi) = (vec![f64::NEG_INFINITY; n], vec![f64::INFINITY; n]);
        let (mut lob, mut hib) = (vec![Bind::None; n], vec![Bind::None; n]);
        // every thing with the distance the run keeps to it with the margin and without
        struct Thing {
            form: Form,
            caps: [bool; 2],
            whole: Option<(Point, Point)>,
            rho: f64,
            bare: f64,
            up: bool,
        }
        let mut things: Vec<Thing> = Vec::new();
        for &(fi, base) in &r.below {
            let f = &self.feats[fi];
            things.push(Thing {
                form: f.form,
                caps: f.caps,
                whole: Some(f.whole),
                rho: base + margin,
                bare: base,
                up: false,
            });
        }
        for (f, base) in &under {
            things.push(Thing {
                form: *f,
                caps: [true, true],
                whole: None,
                rho: base + margin,
                bare: *base,
                up: false,
            });
        }
        for &(fi, base, gaps) in &r.above {
            let f = &self.feats[fi];
            things.push(Thing {
                form: f.form,
                caps: f.caps,
                whole: Some(f.whole),
                rho: base + gaps * margin,
                bare: base,
                up: true,
            });
        }
        // where the run goes on straight from a held point that is too near to something
        let held: Vec<(Point, f64)> = (0..n)
            .filter(|&k| r.pin[k])
            .map(|k| (pt(xs[r.k.0 + k], r.x0[k]), r.wk[k] / 2.0))
            .collect();
        let straight: Vec<Straight> = held
            .iter()
            .flat_map(|&(p, half)| {
                let forms: Vec<(Form, f64)> = things
                    .iter()
                    .map(|t| (t.whole.map_or(t.form, |(a, b)| Form::Seg(a, b)), t.rho + half))
                    .collect();
                straight_on(&forms, &[p])
            })
            .collect();
        for k in 0..n {
            let (x, half) = (xs[r.k.0 + k], r.wk[k] / 2.0);
            let line: Vec<f64> = straight.iter().filter_map(|s| s.at(x)).collect();
            // the straight side of a thing is that of all of it, not of the piece
            let side = |b: Bind, whole: Option<(Point, Point)>| match (b, whole) {
                (Bind::Line(p, _), Some((a, b))) => Bind::Line(Round { c: a, rho: p.rho }, Round { c: b, rho: p.rho }),
                (b, _) => b,
            };
            for t in &things {
                let rho = t.rho + half;
                let (a, b) = t.form.reach(rho);
                if x < a || x > b {
                    continue;
                }
                if !t.up {
                    let Some((mut y, mut b)) = top(&t.form, t.caps, rho, x) else {
                        continue;
                    };
                    // on a straight stretch the margin gives way, the clearance does not
                    if let Some(l) = line.iter().copied().reduce(f64::min)
                        && l < y
                    {
                        (y, b) = match top(&t.form, t.caps, t.bare + half, x) {
                            Some((bare, bind)) if bare > l + EPS => (bare, bind),
                            _ => (l, Bind::None),
                        };
                    }
                    if y > lo[k] {
                        (lo[k], lob[k]) = (y, side(b, t.whole));
                    }
                } else {
                    let Some((mut y, mut b)) = bottom(&t.form, t.caps, rho, x) else {
                        continue;
                    };
                    if let Some(l) = line.iter().copied().reduce(f64::max)
                        && l > y
                    {
                        (y, b) = match bottom(&t.form, t.caps, t.bare + half, x) {
                            Some((bare, bind)) if bare < l - EPS => (bare, bind),
                            _ => (l, Bind::None),
                        };
                    }
                    if y < hi[k] {
                        (hi[k], hib[k]) = (y, side(b, t.whole));
                    }
                }
            }
        }
        (lo, hi, lob, hib)
    }

    /// Lays the runs from the first side outwards with `margin` on every clearance. Err: what
    /// a run lacks at most, and where.
    fn lay(&self, geo: &Geometry, margin: f64) -> Result<Try, (f64, Point)> {
        let xs = &self.stations;
        let mut new: Vec<Vec<New>> = (0..self.runs.len()).map(|_| Vec::new()).collect();
        let mut tracks: Vec<(usize, Track)> = Vec::new();
        let mut worst: Option<(f64, Point)> = None;
        let mut rough = 0;
        // where each run is out of its bounds as the copper lies
        let before: Vec<Vec<bool>> = (0..self.runs.len())
            .map(|i| {
                let r = &self.runs[i];
                let (lo, hi, ..) = self.limits(geo, i, margin, &[]);
                (0..lo.len())
                    .map(|k| r.x0[k] < lo[k] - EPS || r.x0[k] > hi[k] + EPS)
                    .collect()
            })
            .collect();
        for &i in &self.order {
            let r = &self.runs[i];
            let n = r.k.1 - r.k.0 + 1;
            let (lo, hi, lob, hib) = self.limits(geo, i, margin, &tracks);
            let was = &before[i];
            // where it is out of its bounds, in the window
            let x = |k: usize| xs[r.k.0 + k];
            // a place where it was out of them is looked at as a whole if part of it is in the
            // window: a new stretch cannot end where the old copper is too near to something
            let mut live: Vec<bool> = (0..n)
                .map(|k| x(k) >= self.window.0 - 1e-9 && x(k) <= self.window.1 + 1e-9)
                .collect();
            for k in 1..n {
                live[k] |= was[k] && live[k - 1];
            }
            for k in (0..n - 1).rev() {
                live[k] |= was[k] && live[k + 1];
            }
            let seen = |k: usize| live[k];
            let out = |k: usize| r.x0[k] < lo[k] - EPS || r.x0[k] > hi[k] + EPS;
            // beside the window only where a run laid before it has pushed it out
            let dirty: Vec<bool> = (0..n).map(|k| !r.pin[k] && out(k) && (seen(k) || !was[k])).collect();
            for k in (0..n).filter(|&k| !r.pin[k] && (seen(k) || !was[k])) {
                if lo[k] > hi[k] + EPS && worst.is_none_or(|w| lo[k] - hi[k] > w.0) {
                    worst = Some((lo[k] - hi[k], self.frame.from(pt(x(k), r.x0[k]))));
                }
            }
            if worst.is_some() || !dirty.contains(&true) {
                continue;
            }
            // It is laid again from the corner before such a place to the corner behind it, and
            // at least JOIN beyond: never past a place where it is held, and beside the window
            // not into a place where it was out of its bounds before.
            let free = |j: usize| seen(j) || !out(j) || !was[j];
            let mut stretches: Vec<(usize, usize)> = Vec::new();
            let mut k = 0;
            while k < n {
                if !dirty[k] {
                    k += 1;
                    continue;
                }
                let mut b = k;
                while b + 1 < n && dirty[b + 1] {
                    b += 1;
                }
                let (mut a, mut b2) = (k, b);
                while a > 0 && !r.pin[a] && free(a - 1) && (x(k) - x(a) < JOIN || !r.corner[a]) {
                    a -= 1;
                }
                while b2 + 1 < n && !r.pin[b2] && free(b2 + 1) && (x(b2) - x(b) < JOIN || !r.corner[b2]) {
                    b2 += 1;
                }
                loop {
                    let (xa, xb) = (x(a), x(b2));
                    let grow = r
                        .arcs
                        .iter()
                        .find(|(p, q)| (*p < xa - 1e-6 && xa < *q - 1e-6) || (*p < xb - 1e-6 && xb < *q - 1e-6));
                    let Some(&(p, q)) = grow else { break };
                    while a > 0 && x(a) > p + 1e-6 {
                        a -= 1;
                    }
                    while b2 + 1 < n && x(b2) < q - 1e-6 {
                        b2 += 1;
                    }
                }
                match stretches.last_mut() {
                    Some(last) if last.1 >= a => last.1 = last.1.max(b2),
                    _ => stretches.push((a, b2)),
                }
                k = b + 1;
            }
            for (a, b) in stretches {
                // held at both ends and at every joint between
                let mut pins: Vec<usize> = (a..=b).filter(|&k| k == a || k == b || r.pin[k]).collect();
                pins.dedup();
                let mut nodes: Vec<Node> = Vec::new();
                let mut plain: Vec<Point> = Vec::new();
                for p in pins.windows(2) {
                    let (p, q) = (p[0], p[1]);
                    let m = q - p + 1;
                    let (mut l, mut h): (Vec<f64>, Vec<f64>) = (
                        (p..=q).map(|k| lo[k].min(hi[k])).collect(),
                        (p..=q).map(|k| hi[k].max(lo[k].min(hi[k]))).collect(),
                    );
                    (l[0], h[0]) = (r.x0[p], r.x0[p]);
                    (l[m - 1], h[m - 1]) = (r.x0[q], r.x0[q]);
                    let sx: Vec<f64> = (p..=q).map(x).collect();
                    let touched = taut(&sx, &l, &h);
                    for (ti, &(k, side)) in touched.iter().enumerate() {
                        if ti == 0 && !nodes.is_empty() {
                            continue;
                        }
                        // where the two bounds meet there is one way: that along the lower one
                        // (that along the upper one where only that one is something round)
                        let tight = h[k] - l[k] < 5e-4;
                        let side = match side {
                            Side::Hi if tight && lob[p + k] != Bind::None => Side::Lo,
                            Side::Lo if tight && lob[p + k] == Bind::None && hib[p + k] != Bind::None => Side::Hi,
                            s => s,
                        };
                        let at = pt(sx[k], if side == Side::Hi { h[k] } else { l[k] });
                        plain.push(at);
                        let (bind, sign) = match side {
                            Side::Pin => (Bind::None, 0.0),
                            Side::Lo => (lob[p + k], 1.0),
                            Side::Hi => (hib[p + k], -1.0),
                        };
                        let push = |nodes: &mut Vec<Node>, n: Node| {
                            if nodes.last() != Some(&n) {
                                nodes.push(n);
                            }
                        };
                        let round = |c: Round| Node {
                            c: c.c,
                            q: sign * c.rho,
                        };
                        match bind {
                            Bind::None => push(&mut nodes, Node { c: at, q: 0.0 }),
                            Bind::Round(c) => push(&mut nodes, round(c)),
                            // along a straight side: around the ends of it that lie on the way
                            Bind::Line(c, d) => {
                                let on = |c: Round| c.c.x > x(a) && c.c.x < x(b);
                                if nodes.last() != Some(&round(d)) {
                                    if on(c) {
                                        push(&mut nodes, round(c));
                                    }
                                    if on(d) {
                                        push(&mut nodes, round(d));
                                    }
                                }
                            }
                        }
                    }
                }
                // lines and arcs through what it touches, if they stay within the bounds;
                // else the lines from station to station
                let exact = string(&nodes).filter(|e| {
                    let way = fine(e);
                    way.windows(2).all(|p| p[1].x > p[0].x)
                        && (a..=b).all(|k| {
                            let y = height(&way, x(k));
                            r.pin[k] || k == a || k == b || (y >= lo[k] - 3e-4 && y <= hi[k].max(lo[k]) + 3e-4)
                        })
                });
                if exact.is_none() {
                    rough += 1;
                }
                let elems = exact.unwrap_or_else(|| plain.windows(2).map(|p| Elem::Line(p[0], p[1])).collect());
                new[i].push(New {
                    from: x(a),
                    to: x(b),
                    elems,
                });
            }
            if !new[i].is_empty() {
                tracks.push((r.ti, self.track(&geo.tracks[r.ti], r, &new[i])));
            }
        }
        match worst {
            Some(w) => Err(w),
            None => Ok(Try { tracks, rough }),
        }
    }

    /// The track of a run with its new stretches in place of the old copper there.
    fn track(&self, old: &Track, r: &Run, new: &[New]) -> Track {
        let f = &self.frame;
        let turned = |e: Elem| match e {
            Elem::Line(a, b) => Elem::Line(b, a),
            Elem::Arc(a, m, b) => Elem::Arc(b, m, a),
        };
        // the old elements of the span, each going on in x, with where it begins and ends
        let mut span: Vec<(Elem, f64, f64)> = (r.e.0..=r.e.1)
            .map(|ei| {
                let e = if r.rev { turned(old.elems[ei]) } else { old.elems[ei] };
                let (a, b) = match e {
                    Elem::Line(a, b) | Elem::Arc(a, _, b) => (a, b),
                };
                (e, f.to(a).x, f.to(b).x)
            })
            .collect();
        if r.rev {
            span.reverse();
        }
        // (element, width, laid here)
        let mut out: Vec<(Elem, f64, bool)> = Vec::new();
        // the old copper from x1 to x2: whole elements, and the part of a line that is cut
        let keep = |x1: f64, x2: f64, out: &mut Vec<(Elem, f64, bool)>| {
            for &(e, ea, eb) in &span {
                if eb.min(x2) - ea.max(x1) < 1e-9 {
                    continue;
                }
                let (a, b) = match e {
                    Elem::Line(a, b) | Elem::Arc(a, _, b) => (a, b),
                };
                let w = old.piece_width(a, b);
                if ea >= x1 - 1e-9 && eb <= x2 + 1e-9 {
                    out.push((e, w, false));
                } else {
                    let at = |x: f64| f.from(pt(x, r.at(x)));
                    let p = if ea >= x1 - 1e-9 { a } else { at(x1) };
                    let q = if eb <= x2 + 1e-9 { b } else { at(x2) };
                    out.push((Elem::Line(p, q), w, true));
                }
            }
        };
        let mut at = f64::NEG_INFINITY;
        for n in new {
            keep(at, n.from, &mut out);
            // the places where the old width changes: a new line is cut there
            let cuts: Vec<f64> =
                r.v.windows(2)
                    .zip(r.w.windows(2))
                    .filter(|(p, w)| (w[0] - w[1]).abs() > 1e-12 && p[1].x > n.from + 1e-6 && p[1].x < n.to - 1e-6)
                    .map(|(p, _)| p[1].x)
                    .collect();
            for e in &n.elems {
                let mut pieces = vec![*e];
                if let Elem::Line(p, q) = *e {
                    let inside: Vec<f64> = cuts
                        .iter()
                        .copied()
                        .filter(|c| *c > p.x + 1e-6 && *c < q.x - 1e-6)
                        .collect();
                    if !inside.is_empty() {
                        let mut from = p;
                        pieces.clear();
                        for c in inside {
                            let m = p.lerp(q, (c - p.x) / (q.x - p.x));
                            pieces.push(Elem::Line(from, m));
                            from = m;
                        }
                        pieces.push(Elem::Line(from, q));
                    }
                }
                for piece in pieces {
                    let (p, q) = match piece {
                        Elem::Line(p, q) | Elem::Arc(p, _, q) => (p, q),
                    };
                    let w = r.width_between(p.x, q.x);
                    out.push((
                        match piece {
                            Elem::Line(p, q) => Elem::Line(f.from(p), f.from(q)),
                            Elem::Arc(p, m, q) => Elem::Arc(f.from(p), f.from(m), f.from(q)),
                        },
                        w,
                        true,
                    ));
                }
            }
            at = n.to;
        }
        keep(at, f64::INFINITY, &mut out);
        // the span begins and ends where it did, and every element begins where the one before
        // it ends, to the last digit
        let ends = |e: Elem| match e {
            Elem::Line(a, b) | Elem::Arc(a, _, b) => (a, b),
        };
        if let (Some(first), Some(last)) = (span.first(), span.last()) {
            let (a, b) = (ends(first.0).0, ends(last.0).1);
            if let Some(Elem::Line(p, _) | Elem::Arc(p, _, _)) = out.first_mut().map(|e| &mut e.0) {
                *p = a;
            }
            if let Some(Elem::Line(_, q) | Elem::Arc(_, _, q)) = out.last_mut().map(|e| &mut e.0) {
                *q = b;
            }
        }
        for i in 1..out.len() {
            let end = match out[i - 1].0 {
                Elem::Line(_, b) | Elem::Arc(_, _, b) => b,
            };
            match &mut out[i].0 {
                Elem::Line(a, _) | Elem::Arc(a, _, _) => *a = end,
            }
        }
        out.retain(|(e, ..)| match *e {
            Elem::Line(a, b) => a.dist(b) > 1e-9,
            Elem::Arc(..) => true,
        });
        if r.rev {
            out.reverse();
            out.iter_mut().for_each(|e| e.0 = turned(e.0));
        }
        let mut t = old.clone();
        t.elems = old.elems[..r.e.0]
            .iter()
            .copied()
            .chain(out.iter().map(|e| e.0))
            .chain(old.elems[r.e.1 + 1..].iter().copied())
            .collect();
        t.pts = flatten(&t.elems);
        // the widths: every new piece is as wide as the old copper was there
        let pieces: Vec<(Point, Point, f64)> = out
            .iter()
            .filter(|e| e.2)
            .flat_map(|(e, w, _)| {
                let pts = flatten(&[*e]);
                let chord = (pts[0], pts[pts.len() - 1], *w);
                pts.windows(2)
                    .map(|p| (p[0], p[1], *w))
                    .chain([chord])
                    .collect::<Vec<_>>()
            })
            .collect();
        t.fitted
            .retain(|(z, _)| !pieces.iter().any(|(a, b, _)| z.touches_seg(*a, *b)));
        for (a, b, w) in pieces {
            if (t.piece_width(a, b) - w).abs() > 1e-12 {
                let mut z = Rect::empty();
                z.add(a.lerp(b, 0.5));
                t.fitted.push((z.grow(1e-6), w));
            }
        }
        t
    }
}

/// The best laying of a scene: with the full margin if it fits, else with the largest that does.
fn best(scene: &Scene, geo: &Geometry) -> Result<(Try, f64), (f64, Point)> {
    match scene.lay(geo, MARGIN) {
        Ok(t) => Ok((t, MARGIN)),
        Err(full) => {
            let mut found = match scene.lay(geo, LEAST) {
                Ok(t) => (t, LEAST),
                Err(_) => return Err(full),
            };
            let (mut lo, mut hi) = (LEAST, MARGIN);
            for _ in 0..6 {
                let mid = (lo + hi) / 2.0;
                match scene.lay(geo, mid) {
                    Ok(t) => {
                        found = (t, mid);
                        lo = mid;
                    }
                    Err(_) => hi = mid,
                }
            }
            Ok(found)
        }
    }
}

/// Lays the tracks of passage `p` again where they are too near (see the module). `violations`
/// are the findings of the check the passage was taken from.
pub fn lay(board: &Board, geo: &Geometry, violations: &[Violation], p: &Passage) -> Result<Laid, Why> {
    if p.crossing || p.dir.len() < 0.5 {
        return Err(Why::NoRun);
    }
    let key = |n: (NetId, Option<NetId>)| {
        let k = n.1.unwrap_or(u32::MAX);
        (n.0.min(k), n.0.max(k))
    };
    let mut results: Vec<Result<(Try, f64, Frame), Why>> = Vec::new();
    for n in [p.dir, -p.dir] {
        let frame = Frame::new(p.from, n);
        // the window: as far along the tracks as the pieces of the findings of the spot go
        let (mut w0, mut w1) = (0.0f64, 0.0f64);
        for v in violations
            .iter()
            .filter(|v| !v.is_width() && key(v.nets) == key(p.nets) && v.at.dist(p.at) < 2.0)
        {
            let piece = geo
                .tracks
                .iter()
                .filter(|t| t.net == v.nets.0 && t.layer == p.layer)
                .flat_map(|t| t.pts.windows(2))
                .find(|w| w[0].lerp(w[1], 0.5).dist(v.at) < 1e-9);
            let ends = piece.map_or(vec![v.at], |w| vec![w[0], w[1]]);
            for l in ends.into_iter().map(|e| frame.to(e)) {
                w0 = w0.min(l.x);
                w1 = w1.max(l.x);
            }
        }
        let w = (w0 - AHEAD, w1 + AHEAD);
        results.push(Scene::build(board, geo, p, frame, w).and_then(|scene| {
            best(&scene, geo)
                .map(|(t, m)| (t, m, frame))
                .map_err(|(missing, at)| Why::NoRoom { missing, at })
        }));
    }
    // the side that lays fewer tracks again, with the larger margin first
    let mut ok: Vec<(Try, f64, Frame)> = Vec::new();
    let mut why: Option<Why> = None;
    for r in results {
        match r {
            Ok(t) if t.0.tracks.is_empty() => why = why.or(Some(Why::Held)),
            Ok(t) => ok.push(t),
            Err(e) => why = Some(e),
        }
    }
    ok.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.tracks.len().cmp(&b.0.tracks.len())));
    let Some((t, margin, _)) = ok.into_iter().next() else {
        return Err(why.unwrap_or(Why::NoRun));
    };
    let rough = t.rough;
    let mut out = geo.clone();
    let mut changed = Vec::new();
    for (ti, track) in t.tracks {
        out.tracks[ti] = track;
        changed.push(ti);
    }
    Ok(Laid {
        geo: out,
        tracks: changed,
        margin,
        rough,
    })
}

/// Every spot of the check where the tracks can give way gets its tracks laid again, the
/// worst first, each group taken only if the check then finds nothing new anywhere and the
/// worst finding of the spot is gone. Vias and track ends stay where they are.
pub fn spread(board: &Board, geo: &mut Geometry) -> Report {
    let mut report = Report::default();
    let same = |a: &Violation, b: &Violation| {
        a.nets == b.nets && a.what == b.what && a.at.dist(b.at) < 1e-9 && (a.actual - b.actual).abs() < 1e-9
    };
    let mut tried: Vec<(String, Point)> = Vec::new();
    let mut all = check::check(board, geo);
    loop {
        let clear: Vec<Violation> = all.iter().filter(|v| !v.is_width()).cloned().collect();
        let passages = passage::passages(board, geo, &clear);
        let next = passages
            .iter()
            .filter(|p| !tried.iter().any(|(w, at)| *w == p.what && at.dist(p.at) < 1e-6))
            .filter(|p| p.gives_way(board, geo, LEAST))
            .max_by(|a, b| a.missing.total_cmp(&b.missing));
        let Some(p) = next else { break };
        tried.push((p.what.clone(), p.at));
        report.tried += 1;
        let back = |why: Why| Back {
            what: p.what.clone(),
            at: p.at,
            pieces: p.pieces,
            why,
        };
        match lay(board, geo, &clear, p) {
            Err(why) => report.back.push(back(why)),
            Ok(laid) => {
                let after = check::check(board, &laid.geo);
                let new = after.iter().filter(|v| !all.iter().any(|o| same(o, v))).count();
                let still = after.iter().any(|v| v.what == p.what && v.at.dist(p.at) < 1e-9);
                if new > 0 || still {
                    report.back.push(back(Why::Check { new, still }));
                } else {
                    *geo = laid.geo;
                    all = after;
                    // what kept its copper may have room now
                    tried.clear();
                    report.back.clear();
                    report.laid += 1;
                    report.tracks += laid.tracks.len();
                    report.rough += laid.rough;
                    report.margin = Some(report.margin.map_or(laid.margin, |m: f64| m.min(laid.margin)));
                }
            }
        }
    }
    report
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::realize::Via;

    fn track(pts: &[Point], net: NetId, width: f64) -> Track {
        Track {
            pts: pts.to_vec(),
            elems: pts.windows(2).map(|p| Elem::Line(p[0], p[1])).collect(),
            width,
            layer: 0,
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

    fn via(at: Point, net: NetId) -> Via {
        Via {
            at,
            diameter: 0.6,
            drill: 0.3,
            net,
            layer: None,
            origin: None,
            resting: false,
            half: Point::default(),
        }
    }

    /// The distance from p to the copper of track t, less half its width.
    fn off(t: &Track, p: Point) -> f64 {
        t.pts
            .windows(2)
            .map(|w| point_seg_dist(p, w[0], w[1]) - t.piece_width(w[0], w[1]) / 2.0)
            .fold(f64::INFINITY, f64::min)
    }

    /// The air between the copper of two tracks.
    fn air(a: &Track, b: &Track) -> f64 {
        a.pts
            .windows(2)
            .flat_map(|p| {
                b.pts.windows(2).map(move |q| {
                    crate::geom::seg_seg_dist(p[0], p[1], q[0], q[1])
                        - a.piece_width(p[0], p[1]) / 2.0
                        - b.piece_width(q[0], q[1]) / 2.0
                })
            })
            .fold(f64::INFINITY, f64::min)
    }

    fn shape(t: &Track) -> String {
        t.elems
            .iter()
            .map(|e| match e {
                Elem::Line(..) => 'l',
                Elem::Arc(..) => 'a',
            })
            .collect()
    }

    /// The worst spot of the board laid by [`lay`], and what the check says before and after.
    fn laid(b: &Board, geo: &Geometry) -> (Laid, Vec<Violation>) {
        let found = check::check(b, geo);
        assert!(!found.is_empty(), "nothing is too near");
        let all = passage::passages(b, geo, &found);
        let p = all
            .iter()
            .max_by(|a, b| a.missing.total_cmp(&b.missing))
            .expect("a spot");
        assert!(p.gives_way(b, geo, LEAST), "{}: no room", p.what);
        let l = lay(b, geo, &found, p).unwrap_or_else(|why| panic!("{}: {why}", p.what));
        let after = check::check(b, &l.geo);
        (l, after)
    }

    /// Two pads of X and Y, their faces `apart` from each other, and tracks of A, B, C between
    /// them at the places `at` from the face of X (centre lines).
    fn between_pads(apart: f64, at: &[f64]) -> (Board, Geometry) {
        let b = crate::route::tests::pad_board(&[
            ("U", "X", 5.0, 5.0),
            ("U", "Y", 5.4 + apart, 5.0),
            ("J", "A", 2.0, 2.0),
            ("J", "B", 3.0, 2.0),
            ("J", "C", 4.0, 2.0),
        ]);
        let w = b.class_of(2).track_width;
        let geo = Geometry {
            tracks: at
                .iter()
                .enumerate()
                .map(|(i, x)| {
                    // apart at the ends, side by side past the pads
                    let far = 5.2 + x + (i as f64 - 1.0) * 0.5;
                    track(
                        &[pt(far, 3.0), pt(5.2 + x, 4.5), pt(5.2 + x, 5.5), pt(far, 7.0)],
                        2 + i as NetId,
                        w,
                    )
                })
                .collect(),
            ..Default::default()
        };
        (b, geo)
    }

    #[test]
    fn three_tracks_between_two_pads_share_six_micrometres_evenly() {
        let b = between_pads(3.0, &[]).0;
        let (w, c) = (b.class_of(2).track_width, b.track_clearance(2, 3));
        let pad = 0.2f64.max(c);
        // packed from the face of X: no air to spare
        let packed = [pad + w / 2.0, pad + 1.5 * w + c, pad + 2.5 * w + 2.0 * c];
        let apart = packed[2] + w / 2.0 + pad + 0.006;
        // as they lie: all six at the pad of X, and B four too near to A
        let (b, geo) = between_pads(apart, &[packed[0] + 0.006, packed[1] + 0.002, packed[2] + 0.006]);
        let (l, after) = laid(&b, &geo);
        assert!(after.is_empty(), "{after:?}");
        // four gaps, 1.5 um each
        assert!((l.margin - 0.0015).abs() < 1e-4, "{}", l.margin);
        let t = &l.geo.tracks;
        let (x, y) = (pt(5.2, 5.0), pt(5.2 + apart, 5.0));
        let gaps = [
            off(&t[0], x) - pad,
            air(&t[0], &t[1]) - c,
            air(&t[1], &t[2]) - c,
            off(&t[2], y) - pad,
        ];
        for g in gaps {
            assert!((g - 0.0015).abs() < 1.5e-4, "{gaps:?}");
        }
        // the outer ones turn around the corners of their pads, the ends where they were
        assert_eq!(
            [shape(&t[0]), shape(&t[1]), shape(&t[2])],
            ["lalal", "lll", "lalal"].map(String::from)
        );
        for (new, old) in t.iter().zip(&geo.tracks) {
            assert_eq!((new.pts[0], *new.pts.last().unwrap()), (old.pts[0], old.pts[3]));
            assert_eq!(new.pts, flatten(&new.elems));
        }
    }

    #[test]
    fn a_track_bent_too_near_around_a_via_becomes_a_line_an_arc_and_a_line() {
        let b = crate::route::tests::pad_board(&[("J", "V", 2.0, 2.0), ("J", "A", 3.0, 2.0)]);
        let (w, c) = (b.class_of(1).track_width, b.class_of(1).clearance);
        // the via at (10, 10); the track passes over it with a corner, 10 um too near
        let r = 0.3 + c + w / 2.0;
        let geo = Geometry {
            vias: vec![via(pt(10.0, 10.0), 0)],
            tracks: vec![track(&[pt(8.0, 9.9), pt(10.0, 10.0 + r - 0.01), pt(12.0, 9.9)], 1, w)],
        };
        let (l, after) = laid(&b, &geo);
        assert!(after.is_empty(), "{after:?}");
        let t = &l.geo.tracks[0];
        assert_eq!(shape(t), "lal");
        assert_eq!((t.pts[0], *t.pts.last().unwrap()), (pt(8.0, 9.9), pt(12.0, 9.9)));
        // the arc around the via, at the clearance and the margin
        let Elem::Arc(a, m, e) = t.elems[1] else { unreachable!() };
        let (centre, rad, ..) = arc_from_three(a, m, e).unwrap();
        assert!(
            centre.dist(pt(10.0, 10.0)) < 1e-6 && (rad - r - MARGIN).abs() < 1e-6,
            "{centre:?} {rad}"
        );
        assert!((l.margin - MARGIN).abs() < 1e-12);
        // and the lines touch it
        let Elem::Line(p, q) = t.elems[0] else { unreachable!() };
        assert!(((q - p).norm().dot((q - centre).norm())).abs() < 1e-6);
    }

    #[test]
    fn a_stack_too_near_around_a_pad_corner_gets_arcs_around_that_corner() {
        let b = crate::route::tests::pad_board(&[
            ("U", "X", 10.0, 10.0),
            ("J", "A", 2.0, 2.0),
            ("J", "B", 3.0, 2.0),
            ("J", "C", 4.0, 2.0),
        ]);
        let (w, c) = (b.class_of(1).track_width, b.track_clearance(1, 2));
        let pad = 0.2f64.max(c);
        // The corner of the pad at (10.2, 10.2). Three tracks turn around it by one radian, each
        // with one corner on the diagonal. Their ends leave room, 30 um in every gap; at the
        // corner the first is too near to the pad, and the others lie too near to it to stay
        // when it gives way.
        let corner = pt(10.2, 10.2);
        let d = pt(1.0, 1.0).norm();
        let geo = Geometry {
            tracks: (0..3)
                .map(|i| {
                    let i = i as f64;
                    let far = pad + w / 2.0 + 0.03 + (w + c + 0.03) * i;
                    let near = pad + w / 2.0 - 0.01 + (w + c - 0.005) * i;
                    // a leg: three millimetres of the tangent to the circle of radius `far`
                    let leg = |turn: f64| {
                        let out = d.rotate(turn);
                        corner + out * far + out.perp() * (3.0 * turn.signum())
                    };
                    track(
                        &[leg(0.5), corner + d * (near / crate::math::cos(0.5)), leg(-0.5)],
                        1 + i as NetId,
                        w,
                    )
                })
                .collect(),
            ..Default::default()
        };
        let before = check::check(&b, &geo);
        assert!(before.len() >= 2, "{before:?}");
        let mut g = geo.clone();
        let report = spread(&b, &mut g);
        assert!(check::check(&b, &g).is_empty(), "{report:?}");
        for (i, t) in g.tracks.iter().enumerate() {
            assert_eq!(shape(t), "lal", "track {i}");
            let Elem::Arc(a, m, e) = t.elems[1] else { unreachable!() };
            let (centre, rad, ..) = arc_from_three(a, m, e).unwrap();
            let want = pad + MARGIN + w / 2.0 + (w + c + MARGIN) * i as f64;
            assert!(
                centre.dist(corner) < 1e-6 && (rad - want).abs() < 1e-6,
                "track {i}: {centre:?} {rad} {want}"
            );
            assert_eq!(
                (t.pts[0], *t.pts.last().unwrap()),
                (geo.tracks[i].pts[0], geo.tracks[i].pts[2])
            );
        }
        assert!((air(&g.tracks[0], &g.tracks[1]) - c - MARGIN).abs() < 1e-3);
    }

    #[test]
    fn a_row_at_the_least_pitch_turns_about_the_end_of_its_first_exit_piece() {
        // three pads 0.2 wide at 0.4 mm, track 0.2, clearance 0.2: the exit pieces lie side by
        // side with the clearance and nothing to spare
        let b = crate::exit::tests::row_board(3, 0.4, 0.2, 0.2, 0.2);
        let (w, c) = (0.2, 0.2);
        let x = |k: usize| 9.6 + 0.4 * k as f64;
        let pivot = pt(x(0), 8.6);
        // all turn left by 50 degrees; as they lie the second and third lean over at once
        let d = pt(-1.0, 0.0).rotate(-40f64.to_radians());
        let out = pt(d.y, -d.x);
        let mut tracks: Vec<Track> = (0..3)
            .map(|k| {
                let k = k as f64;
                let end = pivot + d * 3.0 + out * (0.5 * k);
                let from = pt(9.6 + 0.4 * k, 8.6);
                if k == 0.0 {
                    track(&[from, end], 0, w)
                } else {
                    track(&[from, from + pt(-0.1, 0.2 * k), end], k as NetId, w)
                }
            })
            .collect();
        tracks.extend((0..3).map(|k| track(&[pt(x(k), 8.0), pt(x(k), 8.6)], k as NetId, w)));
        let geo = Geometry {
            tracks,
            ..Default::default()
        };
        assert!(check::check(&b, &geo).len() >= 2);
        let mut g = geo.clone();
        let report = spread(&b, &mut g);
        assert!(check::check(&b, &g).is_empty(), "{report:?}");
        assert_eq!(report.margin, Some(MARGIN));
        // track k goes on straight until it is k margins off the circle about the end of the
        // first exit piece that touches its line, and meets the circle there with a corner
        let step = w + c + MARGIN;
        for k in 1..3 {
            let t = &g.tracks[k];
            assert_eq!(shape(t), "lal", "track {k}");
            let f = k as f64;
            let Elem::Line(a, e) = t.elems[0] else { unreachable!() };
            let up = (f * f * (step * step - 0.16)).sqrt();
            assert!(
                a == pt(x(k), 8.6) && e.dist(pt(x(k), 8.6 + up)) < 1e-6,
                "track {k}: {e:?} {up}"
            );
            let Elem::Arc(a, m, e) = t.elems[1] else { unreachable!() };
            let (centre, rad, ..) = arc_from_three(a, m, e).unwrap();
            assert!(
                centre.dist(pivot) < 1e-6 && (rad - f * step).abs() < 1e-6,
                "track {k}: {centre:?} {rad}"
            );
            assert_eq!(*t.pts.last().unwrap(), *geo.tracks[k].pts.last().unwrap());
        }
        // the exit pieces and the first track lie as they did
        for k in [0, 3, 4, 5] {
            assert_eq!(g.tracks[k].pts, geo.tracks[k].pts);
        }
    }

    #[test]
    fn a_track_between_a_via_and_a_pad_gets_the_same_air_on_both_sides() {
        let b = crate::route::tests::pad_board(&[("U", "X", 5.0, 5.0), ("J", "V", 2.0, 2.0), ("J", "A", 3.0, 2.0)]);
        let (w, c) = (b.class_of(2).track_width, b.class_of(2).clearance);
        let pad = 0.2f64.max(c);
        // from the face of the pad at x = 5.2: clearance, track, clearance, via; 3 um to spare,
        // and the track lies 4 um too near to the via
        let x = 5.2 + pad + w / 2.0 + 0.007;
        let at = pt(5.2 + pad + w + c + 0.3 + 0.003, 5.0);
        let geo = Geometry {
            vias: vec![via(at, 1)],
            tracks: vec![track(&[pt(x, 3.0), pt(x, 7.0)], 2, w)],
        };
        let (l, after) = laid(&b, &geo);
        assert!(after.is_empty(), "{after:?}");
        let t = &l.geo.tracks[0];
        let gaps = [off(t, pt(5.2, 5.0)) - pad, off(t, at) - 0.3 - c];
        assert!(
            (gaps[0] - 0.0015).abs() < 1.5e-4 && (gaps[1] - 0.0015).abs() < 1.5e-4,
            "{gaps:?}"
        );
        assert!((l.margin - 0.0015).abs() < 1e-4, "{}", l.margin);
        assert!(t.elems.len() <= 5, "{}", shape(t));
    }
}
