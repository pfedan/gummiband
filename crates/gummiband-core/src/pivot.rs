//! The last shape of a relaxed track, built from its pivots.
//!
//! The relaxation leaves a track as a chain of points; the shortcuts of `relax::taut` turn it
//! into a polyline that is as straight as the clearances allow. Each of its corners is there
//! because something lies in the way of the straight line past it: the corner of a pad, a via,
//! or the tracks between such a thing and this track. Where a track turns around one thing by
//! more than a few degrees the polyline has a corner every tenth of a millimetre, and rounding
//! each of them gives a row of arcs and lines of a few hundredths.
//!
//! This module asks of every corner what it bends around and builds the track again from the
//! answers: one arc around each pivot and tangent lines between the arcs. The radius of an arc
//! is the sum of what lies between the pivot and the track: the size of the pivot, its
//! clearance, and for every track further in its width and the clearance to the next. The
//! tracks of a bundle get arcs around one centre that way, each its gap outside the one before.
//!
//! Nothing is taken on trust: every line and every piece of an arc passes the same clearance
//! test as the shortcuts, against the copper as it is at that moment. Where one does not, a
//! wrap next to it is taken out again, and a corner that no wrap stands for any more comes
//! back and is rounded as before.
//!
//! The relaxations of the search do not use this: their copper is an intermediate state, and a
//! relaxation that starts from it ends elsewhere than one that starts from the corners of
//! before. `relax::pivot_pass` runs it once on the finished copper, track by track, and takes
//! a new form only where it is a shorter list of pieces and the own check finds nothing new.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};

use crate::board::{Board, ObstacleKind};
use crate::geom::{Point, Polygon, Rect, arc_from_three, arc_polyline, orient, point_seg_dist, pt, segs_intersect};
use crate::index::GridIndex;
use crate::math;
use crate::realize::{Elem, Geometry, Wrap, tangent};
use crate::topo::Substrate;

/// How far inside the radius of its stack a polyline may pass a pivot, per track in between.
const TOL: f64 = 0.03;
/// What an arc keeps beyond the margin of the test: its chords lie inside it.
const SAG: f64 = 0.003;
/// Chord error of an arc in the points of a track, as for the rounded corners.
pub(crate) const FLAT: f64 = 0.002;
/// An arc shorter than this becomes the corner of its two tangents.
const SHORT_ARC: f64 = 0.05;
/// Tracks between a pivot and the track, at most.
const DEPTH: usize = 24;

/// What the corners and wraps of the tracks came to, summed over all calls (for measurements).
pub static TALLY: [AtomicU64; NAMES.len()] = [const { AtomicU64::new(0) }; NAMES.len()];
pub const NAMES: [&str; 21] = [
    "tracks",
    "tracks without a corner",
    "tracks: every bend an arc around a pivot",
    "tracks: some bends",
    "tracks: no bend",
    "tracks given up",
    "corners the straight line does not need",
    "corners",
    "corners at the end of a rail",
    "corners with a pivot",
    "no pivot: a track is in the way that bends around nothing fixed",
    "no pivot: nothing of another net is in the way",
    "wraps",
    "wraps kept as arcs",
    "wraps that became the corner of their tangents",
    "corners put back between two wraps",
    "taken out: no tangent",
    "taken out: inside another circle",
    "taken out: the line passes it by",
    "taken out: an arc too close",
    "taken out: a line too close",
];

/// What became of a track in `relax::pivot_pass`, by index: the names, and the counts of all
/// passes of the process so far (for measurements).
pub const OUTCOMES: [&str; 11] = [
    "passes",
    "new form",
    "no corner",
    "bends around nothing fixed",
    "no chain of arcs and lines that keeps its distances",
    "the new form has no fewer pieces",
    "back to what it was: a new finding of the check",
    "one straight line",
    "widths fitted to zones",
    "soft copper or a tuning pattern",
    "a minimum width with neck zones",
];
/// How often the end with the arcs had more findings than the end without, so that the copper
/// without them was finished and got the pass last (`route_fixed`), for measurements.
pub static LAST_INSTEAD: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
pub static OUTCOME: [AtomicU64; OUTCOMES.len()] = [const { AtomicU64::new(0) }; OUTCOMES.len()];
pub(crate) const NEW_FORM: usize = 1;
pub(crate) const NO_CORNER: usize = 2;
pub(crate) const NO_PIVOT: usize = 3;
pub(crate) const GIVEN_UP: usize = 4;
pub(crate) const NO_FEWER: usize = 5;
pub(crate) const BACK: usize = 6;
pub(crate) const STRAIGHT: usize = 7;
pub(crate) const FITTED: usize = 8;
pub(crate) const SOFT: usize = 9;
pub(crate) const RULED: usize = 10;

fn tally(name: &str, n: usize) {
    if let Some(i) = NAMES.iter().position(|x| *x == name) {
        TALLY[i].fetch_add(n as u64, Ordering::Relaxed);
    }
}

/// The arcs of a finished track by the corner each of them rounds off (the bits of the
/// corner's coordinates): start, middle and end.
pub(crate) type Rounded = HashMap<[u64; 2], (Point, Point, Point)>;

fn bits(p: Point) -> [u64; 2] {
    [p.x.to_bits(), p.y.to_bits()]
}

/// The polyline whose corners the arcs of a finished track round off: the ends of its lines,
/// and for an arc the point where its two tangents meet, with the arc of each such corner. An
/// arc that turns by more than 150 degrees has no such corner nearby and stays the chain of
/// its chords.
pub(crate) fn corners_of(start: Point, elems: &[Elem]) -> (Vec<Point>, Rounded) {
    let mut rounded = Rounded::new();
    let mut out = vec![start];
    // the last point is the end of an arc: the line after it starts at the corner instead
    let mut arc_end = false;
    for e in elems {
        match *e {
            Elem::Line(_, b) => {
                if arc_end {
                    out.pop();
                }
                out.push(b);
                arc_end = false;
            }
            Elem::Arc(a, m, b) => match arc_from_three(a, m, b) {
                Some((c, r, sa, sweep)) if sweep.abs() < 150f64.to_radians() => {
                    let mid = sa + sweep / 2.0;
                    let corner = c + pt(math::cos(mid), math::sin(mid)) * (r / math::cos(sweep / 2.0));
                    if out.len() > 1 {
                        out.pop();
                    }
                    out.push(corner);
                    out.push(b);
                    rounded.insert(bits(corner), (a, m, b));
                    arc_end = true;
                }
                _ => {
                    out.extend(arc_polyline(a, m, b, FLAT).into_iter().skip(1));
                    arc_end = false;
                }
            },
        }
    }
    out.dedup_by(|a, b| a.dist(*b) < 1e-9);
    (out, rounded)
}

/// The copper a track is tested against in `relax::taut`.
pub(crate) struct Scene<'a> {
    pub board: &'a Board,
    pub sub: &'a Substrate,
    pub geo: &'a Geometry,
    /// (centre, radius, net, half line of a pair)
    pub vias: &'a [(Point, f64, u32, Point)],
    /// The pieces of the tracks as they are now: (a, b, track), `u32::MAX` for a removed one.
    pub pieces: &'a [(Point, Point, u32)],
    pub idx: &'a GridIndex,
    /// What the test of the shortcuts keeps beyond the clearance.
    pub m: f64,
}

fn nearest_on_seg(p: Point, a: Point, b: Point) -> Point {
    let ab = b - a;
    let l2 = ab.len2();
    let t = if l2 > 0.0 {
        ((p - a).dot(ab) / l2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    a + ab * t
}

/// The pivot a track has at vertex `vi` of a polygon: the vertex, or, where it is one of a row
/// on a circle (a round pad, the rounded corner of a pad, the end of a
/// kept track: all of them polygons around the true shape), the centre of that circle and its
/// radius. A row is at least three vertices joined by sides of about the same length, with all
/// vertices and sides within the chord error `e` of the circle through them.
pub(crate) fn rounding(poly: &Polygon, vi: usize, e: f64) -> (Point, f64) {
    const SIDE: f64 = 0.35;
    let n = poly.pts.len();
    let v = poly.pts[vi];
    if n < 3 {
        return (v, 0.0);
    }
    // side i: from vertex i to the next
    let side = |i: usize| poly.pts[i % n].dist(poly.pts[(i + 1) % n]);
    let limit = SIDE.min(2.6 * side(vi).min(side(vi + n - 1)));
    let mut lo = 0;
    while lo < n - 1 && side(vi + n - lo - 1) <= limit {
        lo += 1;
    }
    let mut hi = 0;
    while lo + hi < n - 1 && side(vi + hi) <= limit {
        hi += 1;
    }
    let len = lo + hi + 1;
    if len < 3 {
        return (v, 0.0);
    }
    let at = |k: usize| poly.pts[(vi + n - lo + k) % n];
    let whole = len == n && side(vi + hi) <= limit;
    // the first and the last vertex of a row lie on the true shape, the others on the polygon
    // around it: the circle through three of the others where there are that many
    let (a, m, b) = if whole {
        (at(0), at(n / 3), at(2 * n / 3))
    } else if len >= 5 {
        (at(1), at(len / 2), at(len - 2))
    } else {
        (at(0), at(len / 2), at(len - 1))
    };
    let Some((c, ..)) = arc_from_three(a, m, b) else {
        return (v, 0.0);
    };
    let (mut far, mut near) = (0.0f64, f64::INFINITY);
    for k in 0..len {
        let d = at(k).dist(c);
        far = far.max(d);
        near = near.min(d);
        if k + 1 < len || whole {
            near = near.min(((at(k) + at(k + 1)) * 0.5).dist(c));
        }
    }
    if far - near > 2.5 * e + 1e-6 || far > 10.0 {
        return (v, 0.0);
    }
    (c, far)
}

/// Where the segments a-b and c-d cross, as a fraction of a-b.
fn crossing(a: Point, b: Point, c: Point, d: Point) -> Option<f64> {
    if !segs_intersect(a, b, c, d) {
        return None;
    }
    let (r, s) = (b - a, d - c);
    let den = r.cross(s);
    (den.abs() > 1e-12).then(|| ((c - a).cross(s) / den).clamp(0.0, 1.0))
}

impl Scene<'_> {
    fn clr(&self, n: u32) -> f64 {
        self.board.class_of(n).clearance.max(self.board.rules.min_clearance)
    }

    /// The radius track `ti` has around a pivot at `c`, of size `r0` and net `pnet`, that
    /// asks `gap(net)` of a track: from the pivot outwards along the line to `at` (a point of
    /// the track) the tracks of other nets that cross it, each its width and its clearance
    /// outside the one before, and the track itself last. None: a track of the pivot's own net
    /// lies in between, or too many.
    fn stack(
        &self,
        ti: usize,
        c: Point,
        r0: f64,
        pnet: Option<u32>,
        gap: &dyn Fn(u32) -> f64,
        at: Point,
    ) -> Option<(f64, usize)> {
        let t = &self.geo.tracks[ti];
        let mut r = Rect::empty();
        r.add(c);
        r.add(at);
        // (distance from the pivot, track, width there)
        let mut between: Vec<(f64, u32, f64)> = Vec::new();
        for pi in self.idx.query(&r.grow(1e-6)) {
            let (p, q, tj) = self.pieces[pi as usize];
            if tj == u32::MAX || tj as usize == ti {
                continue;
            }
            let o = &self.geo.tracks[tj as usize];
            if o.net == t.net || o.layer != t.layer {
                continue;
            }
            if let Some(f) = crossing(c, at, p, q) {
                let d = f * c.dist(at);
                // a track that ends on the pivot is not one to pass
                if d <= r0 {
                    continue;
                }
                match between.iter_mut().find(|x| x.1 == tj) {
                    Some(x) if d < x.0 => *x = (d, tj, o.piece_width(p, q)),
                    Some(_) => {}
                    None => between.push((d, tj, o.piece_width(p, q))),
                }
            }
        }
        if between.len() > DEPTH {
            return None;
        }
        between.sort_by(|x, y| x.0.total_cmp(&y.0));
        let keep = self.m + SAG;
        let (mut sum, mut last) = (r0, None::<(u32, f64)>);
        for (net, w) in between
            .iter()
            .map(|&(_, tj, w)| (self.geo.tracks[tj as usize].net, w))
            .chain([(t.net, t.piece_width(at, at))])
        {
            sum += match last {
                None => {
                    if pnet == Some(net) {
                        return None;
                    }
                    gap(net)
                }
                Some((n0, w0)) => w0 / 2.0 + self.board.track_clearance(n0, net),
            } + w / 2.0
                + keep;
            last = Some((net, w));
        }
        Some((sum, between.len()))
    }

    /// The pivots of the corner a-b-c of track `ti`, in the order in which the track passes
    /// them: centre and radius. A pivot is something that does not move (a vertex or a rounding
    /// of an obstacle or of the board edge, a via) inside the angle of the corner, whose
    /// circle with the radius of the stack reaches across the straight line from a to c; the
    /// two legs of the corner pass it at that radius or more (less [`TOL`]).
    fn pivots(&self, ti: usize, a: Point, b: Point, c: Point) -> Vec<(Point, f64)> {
        let board = self.board;
        let t = &self.geo.tracks[ti];
        let (net, layer) = (t.net, t.layer);
        let turn = orient(a, b, c).signum();
        let chord = c - a;
        let len = chord.len();
        if len < 1e-9 || turn == 0.0 {
            return Vec::new();
        }
        // towards b from the line a-c
        let up = |x: Point| -turn * chord.cross(x - a) / len;
        let mut found: Vec<(f64, Point, f64)> = Vec::new();
        let mut take = |pc: Point, r0: f64, pnet: Option<u32>, gap: &dyn Fn(u32) -> f64| {
            // inside the angle at b
            if orient(a, b, pc) * turn <= 0.0 || orient(b, c, pc) * turn <= 0.0 {
                return;
            }
            if found.iter().any(|f| f.1.dist(pc) < 1e-6) {
                return;
            }
            let near = if point_seg_dist(pc, a, b) < point_seg_dist(pc, b, c) {
                nearest_on_seg(pc, a, b)
            } else {
                nearest_on_seg(pc, b, c)
            };
            let Some((r, n)) = self.stack(ti, pc, r0, pnet, gap, near) else {
                return;
            };
            let along = (pc - a).dot(chord) / len;
            if up(pc) <= -r || along < -r || along > len + r || pc.dist(near) < r - TOL * (n + 1) as f64 {
                return;
            }
            found.push((along, pc, r));
        };
        let mut r = Rect::empty();
        r.add(a);
        r.add(b);
        r.add(c);
        let reach = r.grow(t.width / 2.0 + 1.0);
        let inside = |p: Point| p.x >= reach.min.x && p.x <= reach.max.x && p.y >= reach.min.y && p.y <= reach.max.y;
        let e = board.rules.max_error;
        for oi in self.sub.shared.obs_index.query(&reach) {
            let o = &board.obstacles[oi as usize];
            if !o.layers.has(layer) || (o.net == Some(net) && o.kind != ObstacleKind::Hole) {
                continue;
            }
            let gap = |n: u32| match o.kind {
                ObstacleKind::Keepout { .. } => crate::board::KEEPOUT_GAP,
                ObstacleKind::Hole => board.hole_gap(n),
                _ => o.clearance.max(self.clr(n)).max(board.item_gap(n, o.net)),
            };
            if matches!(o.kind, ObstacleKind::Keepout { tracks: false, .. }) {
                continue;
            }
            let pnet = if o.kind == ObstacleKind::Hole { None } else { o.net };
            let poly = &self.sub.shared.obs_polys[oi as usize];
            for vi in 0..poly.pts.len() {
                if inside(poly.pts[vi])
                    && orient(a, b, poly.pts[vi]) * turn > 0.0
                    && orient(b, c, poly.pts[vi]) * turn > 0.0
                {
                    let (pc, r0) = rounding(poly, vi, e);
                    take(pc, r0, pnet, &gap);
                }
            }
        }
        for poly in &board.outline {
            for vi in 0..poly.pts.len() {
                if inside(poly.pts[vi])
                    && orient(a, b, poly.pts[vi]) * turn > 0.0
                    && orient(b, c, poly.pts[vi]) * turn > 0.0
                {
                    let (pc, r0) = rounding(poly, vi, e);
                    take(pc, r0, None, &|_| board.rules.edge_clearance);
                }
            }
        }
        for &(vc, vr, vn, half) in self.vias {
            if vn == net || !inside(vc) {
                continue;
            }
            let pair = half != Point::default();
            for cc in [vc - half, vc + half].into_iter().take(if pair { 2 } else { 1 }) {
                take(cc, vr, Some(vn), &|n| {
                    self.clr(n).max(self.clr(vn)).max(board.item_gap(n, Some(vn)))
                });
            }
        }
        found.sort_by(|x, y| x.0.total_cmp(&y.0));
        found.into_iter().map(|f| (f.1, f.2)).collect()
    }

    /// Does a track of another net lie in the way of the straight line from a to c for track
    /// `ti`? (For the count of the corners without a pivot.)
    fn track_in_the_way(&self, ti: usize, a: Point, c: Point) -> bool {
        let t = &self.geo.tracks[ti];
        let w = t.piece_width(a, c);
        let mut r = Rect::empty();
        r.add(a);
        r.add(c);
        self.idx.query(&r.grow(w / 2.0 + 1.0)).into_iter().any(|pi| {
            let (p, q, tj) = self.pieces[pi as usize];
            if tj == u32::MAX || tj as usize == ti {
                return false;
            }
            let o = &self.geo.tracks[tj as usize];
            o.net != t.net
                && o.layer == t.layer
                && crate::geom::seg_seg_dist(a, c, p, q)
                    < w / 2.0 + o.piece_width(p, q) / 2.0 + self.board.track_clearance(t.net, o.net) + self.m
        })
    }
}

/// A circle a track turns around, for the corners `from` to `to` of its polyline.
#[derive(Clone, Copy)]
struct Turn {
    c: Point,
    r: f64,
    side: f64,
    from: usize,
    to: usize,
    /// 1 a wrap, 2 the corner of the tangents of a short wrap, 0 taken out.
    kind: u8,
    /// Where along the polyline the track passes it (length from the start), and from where
    /// to where the polyline lies on the circle.
    s: f64,
    lo: f64,
    hi: f64,
}

#[derive(Clone, Copy)]
struct Node {
    c: Point,
    r: f64,
    side: f64,
    /// By how much the polyline turns over the corners this node stands for (radians).
    turn: f64,
    /// The wrap it is, or None for a corner of the polyline.
    wrap: Option<usize>,
    /// The stretch of the polyline it stands on (length from the start).
    lo: f64,
    hi: f64,
}

/// The part of a chain: an arc around circle `at`, or the line from circle `at` to the next
/// (circle 0 is the start, the nodes count from 1).
struct Part {
    elem: Elem,
    at: usize,
    arc: bool,
    /// Angle of the arc.
    sweep: f64,
}

/// Lines and arcs from `start` around the nodes to `end`, or the first pair of circles without
/// a tangent (kinds as in `realize::band_why`; 5: the arc of circle i turns further than the
/// polyline it stands for, by more than half a radian: the line passes that circle by).
fn chain(start: Point, end: Point, nodes: &[Node]) -> Result<Vec<Part>, (usize, u8)> {
    let point = |c: Point| Wrap {
        c,
        r: 0.0,
        side: 1.0,
        id: u32::MAX,
    };
    let mut circles = vec![point(start)];
    circles.extend(nodes.iter().map(|n| Wrap {
        c: n.c,
        r: n.r,
        side: n.side,
        id: 0,
    }));
    circles.push(point(end));
    let mut tangents = Vec::with_capacity(circles.len() - 1);
    for (i, w) in circles.windows(2).enumerate() {
        match tangent(&w[0], &w[1]) {
            Some(t) => tangents.push(t),
            None => {
                let kind = if w[0].c.dist(w[1].c) < 1e-9 {
                    4
                } else if w[0].r == 0.0 || w[1].r == 0.0 {
                    1
                } else if w[0].side == w[1].side {
                    2
                } else {
                    3
                };
                return Err((i, kind));
            }
        }
    }
    let mut out = Vec::new();
    for (i, &(a, b)) in tangents.iter().enumerate() {
        if i > 0 {
            let (prev_b, c) = (tangents[i - 1].1, circles[i]);
            if prev_b.dist(a) > 1e-6 {
                let a0 = math::atan2((prev_b - c.c).y, (prev_b - c.c).x);
                let a1 = math::atan2((a - c.c).y, (a - c.c).x);
                let mut sweep = a1 - a0;
                if c.side > 0.0 {
                    while sweep < 0.0 {
                        sweep += std::f64::consts::TAU;
                    }
                } else {
                    while sweep > 0.0 {
                        sweep -= std::f64::consts::TAU;
                    }
                }
                if sweep.abs() > nodes[i - 1].turn + 0.5 {
                    return Err((i, 5));
                }
                let mid = c.c + pt(c.r, 0.0).rotate(a0 + sweep / 2.0);
                let elem = if sweep.abs() * c.r > 0.002 {
                    Elem::Arc(prev_b, mid, a)
                } else {
                    Elem::Line(prev_b, a)
                };
                out.push(Part {
                    elem,
                    at: i,
                    arc: true,
                    sweep: sweep.abs(),
                });
            }
        }
        if a.dist(b) > 1e-6 {
            out.push(Part {
                elem: Elem::Line(a, b),
                at: i,
                arc: false,
                sweep: 0.0,
            });
        }
    }
    Ok(out)
}

/// The polyline `out` of track `ti` (the shortcuts of `relax::taut`) as lines and arcs around
/// its pivots, with its points. `clear` is the clearance test of the shortcuts for one straight
/// piece, `fillet` rounds a corner a-b-c that starts at `cur` as before (the arc as three
/// points and as chords), here with up to half of each leg, so that the arcs of two corners
/// meet and leave no short line between them. `fixed` are corners that stay (the ends of the
/// rails). A piece of `out` that is left in the result is not tested again: it is the track
/// as the rounds left it. `rounded` (the last pass): the arcs the copper has at corners of
/// `out`; such a corner gets its arc back where both of its legs stay, a new one where not,
/// and is never left as a corner. Err: the track is left to the caller as it is, and why
/// ([`NO_CORNER`], [`NO_PIVOT`], [`GIVEN_UP`]).
pub(crate) fn rebuild(
    sc: &Scene,
    ti: usize,
    out: &[Point],
    fixed: &[Point],
    rounded: &Rounded,
    clear: &dyn Fn(Point, Point) -> bool,
    fillet: &dyn Fn(Point, Point, Point, Point) -> Option<(Point, Point, Point, Vec<Point>)>,
) -> Result<(Vec<Point>, Vec<Elem>), usize> {
    tally("tracks", 1);
    if out.len() < 3 {
        tally("tracks without a corner", 1);
        return Err(NO_CORNER);
    }
    // a corner the straight line past it does not need: the search for the farthest point a
    // shortcut reaches takes it for granted that what lies in the way of one lies in the way of
    // every longer one
    let mut out = out.to_vec();
    let mut k = 1;
    while k + 1 < out.len() {
        if !fixed.iter().any(|f| f.dist(out[k]) < 1e-9) && clear(out[k - 1], out[k + 1]) {
            tally("corners the straight line does not need", 1);
            out.remove(k);
            k = k.max(2) - 1;
        } else {
            k += 1;
        }
    }
    let out = &out[..];
    let n = out.len();
    if n < 3 {
        let p = out.to_vec();
        return Ok((p.clone(), p.windows(2).map(|w| Elem::Line(w[0], w[1])).collect()));
    }
    // the pivots of every corner, as wraps in the order of the track
    let mut wraps: Vec<Turn> = Vec::new();
    let mut turns = vec![0.0; n];
    for k in 1..n - 1 {
        let (a, b, c) = (out[k - 1], out[k], out[k + 1]);
        let (u, v) = ((a - b).norm(), (c - b).norm());
        turns[k] = std::f64::consts::PI - math::acos(u.dot(v).clamp(-1.0, 1.0));
        tally("corners", 1);
        if fixed.iter().any(|f| f.dist(b) < 1e-9) {
            tally("corners at the end of a rail", 1);
            continue;
        }
        if turns[k] < 0.004 {
            continue;
        }
        let list = sc.pivots(ti, a, b, c);
        if list.is_empty() {
            tally(
                if sc.track_in_the_way(ti, a, c) {
                    "no pivot: a track is in the way that bends around nothing fixed"
                } else {
                    "no pivot: nothing of another net is in the way"
                },
                1,
            );
            continue;
        }
        tally("corners with a pivot", 1);
        let side = orient(a, b, c).signum();
        // a pivot of the corner before goes on, a new one comes after those
        let mut pos = 0;
        for (pc, r) in list {
            let same = |w: &Turn| w.to + 1 >= k && w.side == side && w.c.dist(pc) < 1e-6 && (w.r - r).abs() < 2e-3;
            match (pos..wraps.len()).find(|&p| same(&wraps[p])) {
                Some(p) => {
                    wraps[p].to = k;
                    wraps[p].r = wraps[p].r.max(r);
                    pos = p + 1;
                }
                None => {
                    wraps.push(Turn {
                        c: pc,
                        r,
                        side,
                        from: k,
                        to: k,
                        kind: 1,
                        s: 0.0,
                        lo: 0.0,
                        hi: 0.0,
                    });
                    pos = wraps.len();
                }
            }
        }
    }
    tally("wraps", wraps.len());
    if wraps.is_empty() {
        tally("tracks: no bend", 1);
        return Err(NO_PIVOT);
    }
    // where along the polyline a wrap lies
    let mut along = vec![0.0; n];
    for k in 1..n {
        along[k] = along[k - 1] + out[k - 1].dist(out[k]);
    }
    for w in &mut wraps {
        let mut best = (f64::INFINITY, 0.0);
        for k in w.from..=w.to + 1 {
            let q = nearest_on_seg(w.c, out[k - 1], out[k]);
            let d = q.dist(w.c);
            if d < best.0 {
                best = (d, along[k - 1] + out[k - 1].dist(q));
            }
        }
        w.s = best.1;
        (w.lo, w.hi) = (w.s, w.s);
        for k in w.from..=w.to {
            if (out[k].dist(w.c) - w.r).abs() < TOL {
                w.lo = w.lo.min(along[k]);
                w.hi = w.hi.max(along[k]);
            }
        }
    }
    // the chain: the wraps that are left, and every corner that no wrap stands for or that was
    // put back, in the order of the track
    let mut back = vec![false; n];
    let covered = |wraps: &[Turn], k: usize| wraps.iter().any(|w| w.kind != 0 && w.from <= k && k <= w.to);
    let nodes_of = |wraps: &[Turn], back: &[bool]| {
        let mut nodes: Vec<(f64, Node)> = Vec::new();
        for (i, w) in wraps.iter().enumerate().filter(|(_, w)| w.kind != 0) {
            nodes.push((
                w.s,
                Node {
                    c: w.c,
                    r: w.r,
                    side: w.side,
                    turn: turns[w.from..=w.to].iter().sum(),
                    wrap: Some(i),
                    lo: w.lo,
                    hi: w.hi,
                },
            ));
        }
        for k in 1..n - 1 {
            if back[k] || !covered(wraps, k) {
                nodes.push((
                    along[k],
                    Node {
                        c: out[k],
                        r: 0.0,
                        side: 1.0,
                        turn: turns[k],
                        wrap: None,
                        lo: along[k],
                        hi: along[k],
                    },
                ));
            }
        }
        nodes.sort_by(|x, y| x.0.total_cmp(&y.0));
        nodes.into_iter().map(|x| x.1).collect::<Vec<Node>>()
    };
    let (start, end) = (out[0], out[n - 1]);
    // the pieces of the polyline itself stand as they are (a track that is too close somewhere
    // came out of the rounds that way)
    let mut tested: HashMap<[u64; 4], bool> = out
        .windows(2)
        .map(|w| {
            (
                [w[0].x.to_bits(), w[0].y.to_bits(), w[1].x.to_bits(), w[1].y.to_bits()],
                true,
            )
        })
        .collect();
    let mut ok = |a: Point, b: Point| {
        *tested
            .entry([a.x.to_bits(), a.y.to_bits(), b.x.to_bits(), b.y.to_bits()])
            .or_insert_with(|| clear(a, b))
    };
    let mut done = None;
    for _ in 0..2 * wraps.len() + n + 8 {
        let nodes = nodes_of(&wraps, &back);
        // the wraps among circle i and the next (circle i is node i - 1)
        let near = |i: usize| {
            [i.checked_sub(1), (i < nodes.len()).then_some(i)]
                .into_iter()
                .flatten()
                .filter_map(|j| nodes[j].wrap.map(|w| (w, nodes[j].r)))
                .collect::<Vec<_>>()
        };
        let built = match chain(start, end, &nodes) {
            Ok(p) => p,
            Err((i, kind)) => {
                let pair = near(i);
                let victim = match kind {
                    5 => nodes[i - 1].wrap,
                    // of two circles on one side, one inside the other, the inner one is not
                    // what the track turns around
                    2 => pair.iter().min_by(|x, y| x.1.total_cmp(&y.1)).map(|x| x.0),
                    _ => pair.iter().max_by(|x, y| x.1.total_cmp(&y.1)).map(|x| x.0),
                };
                let Some(w) = victim else {
                    tally("tracks given up", 1);
                    return Err(GIVEN_UP);
                };
                tally(
                    match kind {
                        5 => "taken out: the line passes it by",
                        2 => "taken out: inside another circle",
                        _ => "taken out: no tangent",
                    },
                    1,
                );
                wraps[w].kind = 0;
                continue;
            }
        };
        // a short arc is the corner of its tangents
        if let Some((p, w)) = built.iter().find_map(|p| {
            let w = nodes[p.at.checked_sub(1)?].wrap?;
            (p.arc && wraps[w].kind == 1 && p.sweep * wraps[w].r < SHORT_ARC).then_some((p, w))
        }) {
            let x = &mut wraps[w];
            let (a, b) = match p.elem {
                Elem::Arc(a, _, b) | Elem::Line(a, b) => (a, b),
            };
            let mid = (a + b) * 0.5 - x.c;
            x.c = x.c + mid.norm() * (x.r / math::cos(p.sweep / 2.0));
            x.r = 0.0;
            x.kind = 2;
            continue;
        }
        let bad = built.iter().find(|p| match p.elem {
            Elem::Line(a, b) => !ok(a, b),
            Elem::Arc(a, m, b) => !arc_polyline(a, m, b, FLAT).windows(2).all(|w| ok(w[0], w[1])),
        });
        let Some(p) = bad else {
            done = Some((built, nodes));
            break;
        };
        if !p.arc {
            // a line between two nodes: first the corner between them comes back that lies
            // farthest from it
            let from = p.at.checked_sub(1).map_or(0.0, |j| nodes[j].hi);
            let to = nodes.get(p.at).map_or(along[n - 1], |x| x.lo);
            let (a, b) = match p.elem {
                Elem::Line(a, b) | Elem::Arc(a, _, b) => (a, b),
            };
            if let Some(k) = (1..n - 1)
                .filter(|&k| !back[k] && covered(&wraps, k) && along[k] > from + 1e-9 && along[k] < to - 1e-9)
                .max_by(|&x, &y| point_seg_dist(out[x], a, b).total_cmp(&point_seg_dist(out[y], a, b)))
            {
                tally("corners put back between two wraps", 1);
                back[k] = true;
                continue;
            }
        }
        let victim = if p.arc {
            nodes[p.at - 1].wrap
        } else {
            near(p.at).iter().max_by(|x, y| x.1.total_cmp(&y.1)).map(|x| x.0)
        };
        let Some(w) = victim else {
            tally("tracks given up", 1);
            return Err(GIVEN_UP);
        };
        tally(
            if p.arc {
                "taken out: an arc too close"
            } else {
                "taken out: a line too close"
            },
            1,
        );
        wraps[w].kind = 0;
    }
    let Some((parts, nodes)) = done else {
        tally("tracks given up", 1);
        return Err(GIVEN_UP);
    };
    let kept = wraps.iter().filter(|x| x.kind == 1).count();
    tally("wraps kept as arcs", kept);
    tally(
        "wraps that became the corner of their tangents",
        wraps.iter().filter(|x| x.kind == 2).count(),
    );
    tally(
        if wraps.iter().all(|x| x.kind == 0) {
            "tracks: no bend"
        } else if nodes.iter().all(|x| x.wrap.is_some() || x.turn < 0.03) {
            "tracks: every bend an arc around a pivot"
        } else {
            "tracks: some bends"
        },
        1,
    );
    // the corners that are left are rounded as before
    let mut elems: Vec<Elem> = Vec::new();
    let mut flat = vec![start];
    let mut cur = start;
    for (i, p) in parts.iter().enumerate() {
        match p.elem {
            Elem::Arc(a, m, b) => {
                elems.push(p.elem);
                flat.extend(arc_polyline(a, m, b, FLAT));
                cur = b;
            }
            Elem::Line(a, b) => {
                // a corner is rounded where the arc is not a short piece itself
                if !p.arc
                    && p.at < nodes.len()
                    && nodes[p.at].r == 0.0
                    && let Some(Part {
                        elem: Elem::Line(_, c), ..
                    }) = parts.get(i + 1)
                {
                    // the arc the copper has at this corner, where both legs are what they
                    // were; else a new one
                    let was = rounded
                        .get(&bits(b))
                        .filter(|(p1, _, p2)| point_seg_dist(*p1, cur, b) < 1e-7 && point_seg_dist(*p2, b, *c) < 1e-7);
                    let arc = match was {
                        Some(&(p1, mid, p2)) => Some((p1, mid, p2, arc_polyline(p1, mid, p2, FLAT))),
                        None => fillet(a, b, *c, cur)
                            .filter(|x| x.3.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>() >= SHORT_ARC),
                    };
                    if let Some((p1, mid, p2, arc)) = arc {
                        if cur.dist(p1) > 1e-6 {
                            elems.push(Elem::Line(cur, p1));
                        }
                        elems.push(Elem::Arc(p1, mid, p2));
                        flat.extend(arc);
                        cur = p2;
                        continue;
                    }
                    // The corner of an arc of the copper is no copper: nothing has tested it.
                    // Without an arc there the track stays as it was.
                    if rounded.contains_key(&bits(b)) {
                        tally("tracks given up", 1);
                        return Err(GIVEN_UP);
                    }
                }
                if cur.dist(b) > 1e-9 {
                    elems.push(Elem::Line(cur, b));
                    flat.push(b);
                    cur = b;
                }
            }
        }
    }
    flat.dedup_by(|a, b| a.dist(*b) < 1e-9);
    if elems.is_empty() {
        elems.push(Elem::Line(start, end));
        flat = vec![start, end];
    }
    Ok((flat, elems))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_round_pad_is_one_pivot_and_a_small_square_is_four() {
        let round = crate::geom::circle_polygon(pt(1.0, 2.0), 0.4, 0.005);
        let (c, r) = rounding(&round, 3, 0.005);
        assert!(c.dist(pt(1.0, 2.0)) < 1e-6, "{c:?}");
        assert!((0.4..0.406).contains(&r), "{r}");
        let square = Polygon::new(vec![pt(0.0, 0.0), pt(0.3, 0.0), pt(0.3, 0.3), pt(0.0, 0.3)]);
        assert_eq!(rounding(&square, 1, 0.005), (pt(0.3, 0.0), 0.0));
        // the rounded corner of a pad: its centre, not its vertices
        let pad = crate::geom::roundrect_polygon(pt(0.0, 0.0), 2.0, 1.0, 0.25, 0.0, 0.005);
        let near = |q: Point| {
            (0..pad.pts.len())
                .min_by(|&i, &j| pad.pts[i].dist(q).total_cmp(&pad.pts[j].dist(q)))
                .unwrap()
        };
        for q in [pt(0.75 + 0.25 * 0.7, 0.25 + 0.25 * 0.7), pt(1.0, 0.25), pt(0.75, 0.5)] {
            let (c, r) = rounding(&pad, near(q), 0.005);
            assert!(c.dist(pt(0.75, 0.25)) < 0.004, "{q:?}: {c:?}");
            assert!((0.25..0.257).contains(&r), "{q:?}: {r}");
        }
    }

    fn length(e: &Elem) -> f64 {
        crate::realize::flatten(std::slice::from_ref(e))
            .windows(2)
            .map(|w| w[0].dist(w[1]))
            .sum()
    }

    /// Two tracks from the left of a column of pads of other nets to just behind it, with no
    /// room between the pads: both turn around its upper end. The relaxed and shortened
    /// geometry, without and with the arcs around pivots.
    fn bent(pivots: bool) -> (Board, Geometry) {
        let mut pins = Vec::new();
        for (k, net) in ["A", "B"].into_iter().enumerate() {
            pins.push(("U1", net, 3.0, 9.0 + k as f64));
            pins.push(("U2", net, 11.0, 9.0 + k as f64));
        }
        let names: Vec<String> = (0..15).map(|k| format!("X{k}")).collect();
        for (k, net) in names.iter().enumerate() {
            pins.push(("X", net.as_str(), 10.0, 4.0 + 0.5 * k as f64));
        }
        let b = crate::route::tests::pad_board(&pins);
        let sub = Substrate::build(&b, &crate::topo::SubstrateConfig::default());
        let mut r = crate::route::Router::new(&b, &sub, crate::route::RouterConfig::default());
        r.plan();
        r.run();
        let mut geo = r.geometry();
        let cfg = crate::relax::RelaxConfig {
            pivots,
            ..Default::default()
        };
        crate::relax::relax(&b, &sub, &mut geo, &cfg);
        (b, geo)
    }

    #[test]
    fn a_bend_around_a_pad_corner_is_one_arc_around_that_corner() {
        let (b, geo) = bent(true);
        assert!(
            crate::check::check(&b, &geo).is_empty(),
            "{:?}",
            crate::check::check(&b, &geo)
        );
        let corners: Vec<Point> = b
            .obstacles
            .iter()
            .filter_map(|o| match &o.shape {
                crate::geom::Shape::Polygon(p) => Some(p.pts.clone()),
                _ => None,
            })
            .flatten()
            .collect();
        let (mut arcs, mut around) = (0, 0);
        // (centre, radius) of the arcs around pad corners
        let mut rings: Vec<(Point, f64)> = Vec::new();
        for e in geo.tracks.iter().flat_map(|t| t.elems.iter()) {
            assert!(length(e) >= 0.05, "a piece of {:.4} mm: {e:?}", length(e));
            if let Elem::Arc(a, m, c) = *e {
                arcs += 1;
                let (centre, r, ..) = arc_from_three(a, m, c).unwrap();
                if corners.iter().any(|v| v.dist(centre) < 1e-6) {
                    around += 1;
                    rings.push((centre, r));
                }
            }
        }
        assert!(
            around >= 4 && around * 2 > arcs,
            "{around} of {arcs} arcs around a pad corner"
        );
        // two tracks around one corner: the outer one a width and a clearance further out, and
        // what the test of the shortcuts and the chords of an arc ask for on top
        let cl = b.class_of(0);
        let step = cl.track_width + cl.clearance;
        let pair = rings
            .iter()
            .flat_map(|x| rings.iter().map(move |y| (x, y)))
            .find(|(x, y)| x.0.dist(y.0) < 1e-6 && y.1 > x.1 + 1e-6);
        let (x, y) = pair.expect("two tracks around one corner");
        let more = y.1 - x.1 - step;
        assert!((0.0..0.012).contains(&more), "{} and {}: {more}", x.1, y.1);
    }

    #[test]
    fn without_the_knob_the_same_bends_are_rows_of_short_pieces() {
        let (_, old) = bent(false);
        let (_, new) = bent(true);
        let count = |g: &Geometry| g.tracks.iter().map(|t| t.elems.len()).sum::<usize>();
        let short = |g: &Geometry| {
            g.tracks
                .iter()
                .flat_map(|t| t.elems.iter())
                .filter(|e| length(e) < 0.05)
                .count()
        };
        assert!(short(&old) > 0 && short(&new) == 0, "{} {}", short(&old), short(&new));
        assert!(
            count(&new) < count(&old),
            "{} elements against {}",
            count(&new),
            count(&old)
        );
    }

    #[test]
    fn a_chain_turns_around_its_circle_the_way_the_corners_did() {
        // a left turn of 90 degrees around (1, 1): from (0, 0) east, then north
        let node = Node {
            c: pt(1.0, 1.0),
            r: 0.5,
            side: 1.0,
            turn: std::f64::consts::FRAC_PI_2,
            wrap: Some(0),
            lo: 0.0,
            hi: 0.0,
        };
        let parts = chain(pt(0.0, 0.5), pt(1.5, 2.0), &[node]).unwrap();
        assert_eq!(parts.len(), 3);
        assert!(parts[1].arc && (parts[1].sweep - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
        // a circle the straight line passes at more than its radius would be a full turn: refused
        let idle = Node {
            c: pt(1.0, 1.2),
            turn: 0.1,
            ..node
        };
        assert_eq!(chain(pt(0.0, 0.5), pt(3.0, 0.5), &[idle]).err(), Some((1, 5)));
    }
    #[test]
    fn the_last_pass_gives_finished_copper_its_arcs_and_keeps_what_the_check_knows() {
        let (b, old) = bent(false);
        let sub = Substrate::build(&b, &crate::topo::SubstrateConfig::default());
        let short = |g: &Geometry| {
            g.tracks
                .iter()
                .flat_map(|t| t.elems.iter())
                .filter(|e| length(e) < 0.05)
                .count()
        };
        let count = |g: &Geometry| g.tracks.iter().map(|t| t.elems.len()).sum::<usize>();
        let mut new = old.clone();
        let n = crate::relax::pivot_pass(&b, &sub, &mut new);
        assert!(n > 0);
        assert!(crate::check::check(&b, &new).len() <= crate::check::check(&b, &old).len());
        assert!(short(&new) < short(&old), "{} {}", short(&new), short(&old));
        assert!(count(&new) < count(&old), "{} {}", count(&new), count(&old));
        // every track still starts and ends where it did, and one that got no new form is
        // the same copper as before
        let mut same = 0;
        for (t, o) in new.tracks.iter().zip(&old.tracks) {
            assert!(t.pts[0].dist(o.pts[0]) < 1e-9 && t.pts.last().unwrap().dist(*o.pts.last().unwrap()) < 1e-9);
            same += (t.pts.len() == o.pts.len() && t.elems.len() == o.elems.len()) as usize;
        }
        assert_eq!(same + n, new.tracks.len());
        // and no new form has a joint between two lines that is no corner
        assert_eq!(new.clone().join_lines(&b), old.clone().join_lines(&b));
        // a second pass has nothing to shorten that the first one left
        let mut twice = new.clone();
        crate::relax::pivot_pass(&b, &sub, &mut twice);
        assert!(short(&twice) <= short(&new) && count(&twice) <= count(&new));
    }
    #[test]
    fn the_corners_of_rounded_copper_are_where_its_tangents_meet() {
        // a right angle at (2, 0) rounded with radius 0.5, then a slight bend rounded as well
        let k = 0.5 - 0.5 * std::f64::consts::FRAC_1_SQRT_2;
        let elems = vec![
            Elem::Line(pt(0.0, 0.0), pt(1.5, 0.0)),
            Elem::Arc(pt(1.5, 0.0), pt(2.0 - k, k), pt(2.0, 0.5)),
            Elem::Line(pt(2.0, 0.5), pt(2.0, 3.0)),
        ];
        let (c, arcs) = corners_of(pt(0.0, 0.0), &elems);
        assert_eq!((c.len(), arcs.len()), (3, 1));
        assert!(
            c[1].dist(pt(2.0, 0.0)) < 1e-9 && c[2].dist(pt(2.0, 3.0)) < 1e-9,
            "{c:?}"
        );
        // an arc at the start and one at the end keep the ends of the track
        let (c, _) = corners_of(pt(1.5, 0.0), &elems[1..2]);
        assert_eq!(c.len(), 3);
        assert!(c[0].dist(pt(1.5, 0.0)) < 1e-9 && c[1].dist(pt(2.0, 0.0)) < 1e-9 && c[2].dist(pt(2.0, 0.5)) < 1e-9);
        // two arcs in a row: one corner each, and the point between them lies on the line
        // from one corner to the other
        let elems = vec![
            Elem::Line(pt(0.0, 0.0), pt(1.5, 0.0)),
            Elem::Arc(pt(1.5, 0.0), pt(2.0 - k, k), pt(2.0, 0.5)),
            Elem::Arc(pt(2.0, 0.5), pt(2.0 + k, 1.0 - k), pt(2.5, 1.0)),
            Elem::Line(pt(2.5, 1.0), pt(4.0, 1.0)),
        ];
        let (c, arcs) = corners_of(pt(0.0, 0.0), &elems);
        assert_eq!((c.len(), arcs.len()), (4, 2), "{c:?}");
        assert!(
            c[1].dist(pt(2.0, 0.0)) < 1e-9 && c[2].dist(pt(2.0, 1.0)) < 1e-9,
            "{c:?}"
        );
    }
}
