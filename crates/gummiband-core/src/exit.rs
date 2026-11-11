//! Pad exits: how a track leaves a pad in a tight row.
//!
//! Between the pads of a fine pitch part a track has room on the middle line of its pad and
//! hardly anywhere else: beside the pad every sideways offset costs clearance to the next pad one
//! to one. So the first piece of such a track is constructed and not left to the relaxation: one
//! straight line on the middle line, from the pin to an exit point beyond the end of the pad,
//! where the track may turn towards either neighbour without coming nearer to its corner than
//! the clearance.
//!
//! The piece is part of the pad for every step of the route: [`Board::set_exits`] gives the pad
//! obstacle the shape of pad and piece together, the mesh lets tracks of the net cross that
//! shape only at a narrow gate in the far end of the piece (`topo::Mesh::exit_edges`), and the
//! tracks of the net end at the exit point (`Router::seg_end`). Nothing after the search can
//! move, split or shorten it, because it is no track. It becomes a line of its own when the
//! route is done ([`stub_tracks`], [`Board::without_exits`]).

use crate::board::{Board, Exit, NetId, ObstacleKind, TightPad};
use crate::geom::{Point, Polygon, Rect, Shape};
use crate::knobs;
use crate::math;
use crate::realize::{Elem, Geometry, Track};

/// How far beyond the end of the pad an exit point is looked for, and in which steps.
const REACH: f64 = 0.6;
const STEP: f64 = 0.005;
/// A pad has a middle line if it is this much longer than wide.
const LONG: f64 = 1.25;
/// How far the turn at the exit point is followed.
const RAY: f64 = 1.0;
/// How far the piece reaches beyond the exit point as part of the pad, so that the exit point
/// lies inside the pad's shape. The track that begins there covers this with its own round end.
const MOUTH: f64 = 0.01;

/// Half the width of the gate at the far end of a piece: the part of the pad's outline that a
/// track of the net crosses. Narrow, so that the track passes the exit point and not the corner
/// of the piece, where it would stand beside the middle line again.
const GATE: f64 = 0.01;

/// The exit piece as part of the pad: a rectangle of half width `half` from `from` to `to`,
/// with the two ends of the gate as corners on its far side.
fn piece(from: Point, to: Point, half: f64) -> Polygon {
    let side = (to - from).norm().perp();
    let (n, g) = (side * half, side * GATE.min(half / 2.0));
    Polygon::new(vec![from - n, to - n, to - g, to + g, to + n, from + n])
}

/// A pad as its middle line sees it: `u` along the longer side, the pad reaches `fwd` in front
/// of the pin and `back` behind it, and `half` to either side.
#[derive(Debug, Clone, Copy)]
pub struct Frame {
    pub u: Point,
    pub fwd: f64,
    pub back: f64,
    pub half: f64,
}

/// The frame of a convex pad that is clearly longer than wide and has its pin on the middle
/// line. None for round and square pads and for shapes with a dent: they have no middle line.
pub fn frame(poly: &Polygon, pos: Point) -> Option<Frame> {
    let n = poly.pts.len();
    if n < 3 || !poly.contains(pos) {
        return None;
    }
    let turn = poly.signed_area().signum();
    let convex = (0..n).all(|i| {
        let (a, b, c) = (poly.pts[i], poly.pts[(i + 1) % n], poly.pts[(i + 2) % n]);
        (b - a).cross(c - b) * turn >= -1e-9
    });
    if !convex {
        return None;
    }
    let (a, b) = poly.edges().max_by(|x, y| x.0.dist(x.1).total_cmp(&y.0.dist(y.1)))?;
    if a.dist(b) < 1e-9 {
        return None;
    }
    let extent = |u: Point| {
        let v = u.perp();
        let (mut lo, mut hi, mut left, mut right) =
            (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for &p in &poly.pts {
            let d = p - pos;
            lo = lo.min(d.dot(u));
            hi = hi.max(d.dot(u));
            left = left.min(d.dot(v));
            right = right.max(d.dot(v));
        }
        (lo, hi, left, right)
    };
    let mut u = (b - a).norm();
    let (mut lo, mut hi, mut left, mut right) = extent(u);
    if right - left > hi - lo {
        u = u.perp();
        (lo, hi, left, right) = extent(u);
    }
    let (along, across) = (hi - lo, right - left);
    if along < LONG * across || (left + right).abs() > 0.02 * across + 1e-6 {
        return None;
    }
    Some(Frame {
        u,
        fwd: hi,
        back: -lo,
        half: across / 2.0,
    })
}

/// The width a track has where it arrives at pin p: the width of its class, or the neck the
/// router gives it there (`Router::necks`).
pub fn width_at(board: &Board, p: usize) -> f64 {
    let pin = &board.pins[p];
    let cls = board.class_of(pin.net);
    if !neck_allowed(board, p) {
        return cls.track_width;
    }
    let clr = cls.clearance.max(board.rules.min_clearance);
    board
        .rules
        .min_track_width
        .max(board.pin_room(p, clr).min(0.4))
        .min(cls.track_width)
}

/// Whether the rules let a track be narrower than its class at pin p.
fn neck_allowed(board: &Board, p: usize) -> bool {
    let pin = &board.pins[p];
    let cls = board.class_of(pin.net);
    cls.min_width.is_none()
        || (cls.neck_refs.contains(&pin.footprint)
            && board
                .courtyards
                .get(&pin.footprint)
                .is_some_and(|r| r.contains(pin.pos)))
}

/// The clearance copper of `net` keeps from obstacle `o` (None: `o` is no obstacle to it).
pub fn gap_to(board: &Board, net: NetId, o: &crate::board::Obstacle) -> Option<f64> {
    if o.net == Some(net) {
        return None;
    }
    let own = board.class_of(net).clearance.max(board.rules.min_clearance);
    match o.kind {
        ObstacleKind::Keepout { tracks: false, .. } => None,
        ObstacleKind::Keepout { .. } => Some(crate::board::KEEPOUT_GAP),
        ObstacleKind::Hole => Some(board.hole_gap(net).max(o.clearance)),
        _ => Some(o.clearance.max(own).max(board.item_gap(net, o.net))),
    }
}

/// The piece of an exit at `e` in direction `dir` of the pad with the frame `f`: from a little
/// behind the pin, so that the pin lies inside the piece, to the gate.
fn piece_at(f: &Frame, pin: Point, e: Point, dir: Point, width: f64) -> Polygon {
    let behind = if dir.dot(f.u) > 0.0 { f.back } else { f.fwd }.min(width / 2.0);
    piece(pin - dir * behind, e + dir * MOUTH, width / 2.0)
}

/// What [`plan`] lays out for one pad.
#[derive(Debug, Clone)]
pub struct Plan {
    pub pin: usize,
    pub obs: usize,
    pub frame: Frame,
    /// Width of the track at the pin, and of the exit piece (narrower where the row asks for it
    /// and the rules allow a neck).
    pub track_width: f64,
    pub width: f64,
    /// What the row leaves beside the piece after the clearance (negative: missing).
    pub spare: f64,
    /// The pad obstacles of other nets beside this one.
    pub beside: Vec<usize>,
    pub exits: Vec<Exit>,
    /// Pad and exit pieces as one shape.
    pub shape: Polygon,
}

impl Board {
    /// The shape of obstacle `o` as the file has it, without exit pieces.
    pub fn bare_shape(&self, o: usize) -> &Shape {
        self.bare_pads
            .iter()
            .find(|(i, _)| *i == o)
            .map_or(&self.obstacles[o].shape, |(_, s)| s)
    }

    /// The exits of pin p.
    pub fn exits_of(&self, p: usize) -> &[Exit] {
        let lo = self.exits.partition_point(|e| e.pin < p);
        let hi = self.exits.partition_point(|e| e.pin <= p);
        &self.exits[lo..hi]
    }

    /// The board with its pads as the file has them and no exits: the board that
    /// [`with_stubs`] turns a route into copper for. The pads that are too tight stay listed.
    pub fn without_exits(&self) -> Board {
        let mut b = self.clone();
        for (o, shape) in std::mem::take(&mut b.bare_pads) {
            b.obstacles[o].shape = shape;
        }
        b.exits.clear();
        b.exits_on = false;
        b
    }

    /// Lay out the exits of the board anew ([`plan`]) and give the pads their shapes with the
    /// exit pieces. `GB_EXITS=0`: no exits. A board with unlocked copper that is part of the
    /// topology gets none: that copper lies where it lies.
    pub fn set_exits(&mut self) {
        for (o, shape) in std::mem::take(&mut self.bare_pads) {
            self.obstacles[o].shape = shape;
        }
        self.exits.clear();
        self.tight_pads.clear();
        self.exits_on = knobs::EXITS.on() && self.soft.chains.is_empty();
        if !self.exits_on {
            return;
        }
        for p in plan(self) {
            if p.spare < -1e-9 {
                self.tight_pads.push(TightPad {
                    pin: p.pin,
                    width: p.width,
                    missing: -p.spare,
                });
            }
            if p.exits.is_empty() {
                continue;
            }
            self.bare_pads.push((p.obs, self.obstacles[p.obs].shape.clone()));
            self.obstacles[p.obs].shape = Shape::Polygon(p.shape);
            self.exits.extend(p.exits);
        }
    }
}

/// The pads of the board that get an exit, with their exits: every pad on one routing layer
/// whose net is routed (not one of a differential pair, whose ends the pair router lays), that
/// has a middle line ([`frame`]) and beside it a pad of another net of the same part so near
/// that a track on the middle line has less than one clearance of room to the side. Beside
/// means next to the long sides: a pad that faces an end of this one takes no room from the
/// middle line.
///
/// The exit point lies on the middle line beyond the end of the pad, as near to it as a track
/// may turn there by 45 degrees towards either of those neighbours and keep its clearance to
/// them; at the end of the pad if there is no such point within [`REACH`]. An end of the pad
/// gets no exit where the piece would come too near to other copper, a hole, a keep-out or the
/// board edge. So a pad has an exit at each end that is free: outwards, and inwards under the
/// part where there is room.
pub fn plan(board: &Board) -> Vec<Plan> {
    let routing = board.routing_layers();
    let boxes: Vec<Rect> = (0..board.obstacles.len()).map(|o| board.bare_shape(o).bbox()).collect();
    let mut parts: std::collections::BTreeMap<&str, Vec<usize>> = Default::default();
    for (i, p) in board.pins.iter().enumerate() {
        if !p.footprint.is_empty() {
            parts.entry(p.footprint.as_str()).or_default().push(i);
        }
    }
    let centres: std::collections::BTreeMap<&str, Point> = parts
        .iter()
        .map(|(&name, pins)| {
            let sum = pins
                .iter()
                .fold(crate::geom::pt(0.0, 0.0), |a, &i| a + board.pins[i].pos);
            (name, sum * (1.0 / pins.len() as f64))
        })
        .collect();
    // the pieces laid so far: (from, to, half width, net, layer)
    let mut laid: Vec<(Point, Point, f64, NetId, u8)> = Vec::new();
    let mut out = Vec::new();
    for (p, pin) in board.pins.iter().enumerate() {
        let net = &board.nets[pin.net as usize];
        let layers: Vec<u8> = pin.layers.iter().filter(|&l| routing.has(l)).collect();
        // a net with one pin and no plane gets no track
        // (a pad drawn in several pieces has a pin for each, all of one name)
        let alone = net.pins.iter().all(|&q| board.pins[q].name == pin.name);
        let routed = net.pins.contains(&p) && (!alone || net.planes.0 != 0);
        if !routed || pin.obstacles.len() != 1 || layers.len() != 1 || board.in_diff_pair(pin.net) {
            continue;
        }
        let layer = layers[0];
        let obs = pin.obstacles[0];
        let o = &board.obstacles[obs];
        if o.kind != ObstacleKind::Pad || o.hole.is_some() {
            continue;
        }
        let Shape::Polygon(pad) = board.bare_shape(obs) else {
            continue;
        };
        let Some(f) = frame(pad, pin.pos) else { continue };
        let own = board.class_of(pin.net).clearance.max(board.rules.min_clearance);
        let track_width = width_at(board, p);
        // the pads of other nets of the part beside this one, each with its clearance
        let body = (pin.pos - f.u * f.back, pin.pos + f.u * f.fwd);
        let beside: Vec<(usize, f64, f64)> = parts
            .get(pin.footprint.as_str())
            .into_iter()
            .flatten()
            .filter(|&&q| q != p && board.pins[q].layers.has(layer))
            .flat_map(|&q| board.pins[q].obstacles.iter().copied())
            .filter_map(|q| {
                let c = gap_to(board, pin.net, &board.obstacles[q])?;
                let shape = board.bare_shape(q);
                let d = shape.dist_to_seg(body.0, body.1);
                // beside the pad, not beyond one of its ends (the pad under the part, say)
                let along = shape.to_polygon(board.rules.max_error).pts.iter().fold(
                    (f64::INFINITY, f64::NEG_INFINITY),
                    |(lo, hi), &x| {
                        let a = (x - pin.pos).dot(f.u);
                        (lo.min(a), hi.max(a))
                    },
                );
                let overlaps = along.0 < f.fwd - 1e-6 && along.1 > -f.back + 1e-6;
                (overlaps && d - c - track_width / 2.0 < own).then_some((q, c, d))
            })
            .collect();
        // room for half a track on the middle line
        let room = beside.iter().map(|&(_, c, d)| d - c).fold(f64::INFINITY, f64::min);
        if room - track_width / 2.0 >= own {
            continue;
        }
        let width = if room >= track_width / 2.0 || !neck_allowed(board, p) {
            track_width
        } else {
            (2.0 * room).max(board.rules.min_track_width).min(track_width)
        };
        let spare = room - width / 2.0;
        let half = track_width / 2.0;
        let mut exits = Vec::new();
        let mut shape = pad.clone();
        let centre = centres.get(pin.footprint.as_str()).copied().unwrap_or(pin.pos);
        for (dir, len) in [(f.u, f.fwd), (f.u * -1.0, f.back)] {
            if !knobs::EXIT_INWARD.on() && dir.dot(pin.pos - centre) < 0.0 {
                continue;
            }
            let at = |y: f64| pin.pos + dir * (len + y);
            // the turn of 45 degrees at the exit point, to either side
            let turns = [
                dir.rotate(std::f64::consts::FRAC_PI_4),
                dir.rotate(-std::f64::consts::FRAC_PI_4),
            ];
            let may_turn = |e: Point| {
                turns.iter().all(|&t| {
                    beside
                        .iter()
                        .all(|&(q, c, _)| board.bare_shape(q).dist_to_seg(e, e + t * RAY) >= half + c - 1e-9)
                })
            };
            // the piece keeps its distance from everything that is not its net; beside the pad
            // it has the room of the row and no less
            let cls = board.class_of(pin.net);
            let free = |e: Point| {
                if board.edge_dist(e) < board.rules.edge_clearance + half {
                    return false;
                }
                // under a minimum width rule a piece narrower than the class ends inside the
                // courtyard that allows the neck
                if cls.min_width.is_some()
                    && width < cls.track_width - 1e-9
                    && !board.courtyards.get(&pin.footprint).is_some_and(|r| r.contains(e))
                {
                    return false;
                }
                let reach = Rect {
                    min: crate::geom::pt(pin.pos.x.min(e.x), pin.pos.y.min(e.y)),
                    max: crate::geom::pt(pin.pos.x.max(e.x), pin.pos.y.max(e.y)),
                }
                .grow(half + 1.0);
                let clear = (0..board.obstacles.len()).all(|q| {
                    let other = &board.obstacles[q];
                    if !other.layers.has(layer) || !boxes[q].overlaps(&reach) {
                        return true;
                    }
                    let Some(c) = gap_to(board, pin.net, other) else {
                        return true;
                    };
                    let shape = board.bare_shape(q);
                    let floor = if beside.iter().any(|b| b.0 == q) {
                        spare.min(0.0)
                    } else {
                        0.0
                    };
                    shape.dist_to_seg(pin.pos, e) - c - width / 2.0 >= floor - 1e-6
                        && shape.dist_to_point(e) - c - half >= floor - 1e-6
                });
                clear
                    && laid.iter().all(|&(a, b, r, n, l)| {
                        n == pin.net
                            || l != layer
                            || crate::geom::seg_seg_dist(a, b, pin.pos, e)
                                >= r + half + board.clearance(Some(n), Some(pin.net)) - 1e-6
                    })
            };
            let steps = (REACH / STEP).round() as usize;
            let y = (0..=steps).map(|k| k as f64 * STEP).find(|&y| may_turn(at(y)));
            let Some(e) = y
                .map(at)
                .filter(|&e| free(e))
                .or_else(|| Some(at(0.0)).filter(|&e| free(e)))
            else {
                continue;
            };
            let nose = piece_at(&f, pin.pos, e, dir, width);
            let Some(joined) = star_union(&shape, &nose, pin.pos) else {
                continue;
            };
            shape = joined;
            laid.push((pin.pos, e, width / 2.0, pin.net, layer));
            exits.push(Exit {
                pin: p,
                obs,
                at: e,
                dir,
                width,
                spare,
            });
        }
        out.push(Plan {
            pin: p,
            obs,
            frame: f,
            track_width,
            width,
            spare,
            beside: beside.iter().map(|b| b.0).collect(),
            exits,
            shape,
        });
    }
    out
}

/// The union of two convex polygons that both hold `c` inside: its outline is what a ray from
/// `c` meets last, so the corners of both that do not lie inside the other, and the points where
/// their outlines cross, in the order of their direction from `c`. None if that is no polygon.
pub fn star_union(a: &Polygon, b: &Polygon, c: Point) -> Option<Polygon> {
    const EPS: f64 = 1e-7;
    let inside = |q: &Polygon, p: Point| q.contains(p) && q.boundary_dist(p) > EPS;
    let mut pts: Vec<Point> = Vec::new();
    pts.extend(a.pts.iter().copied().filter(|&p| !inside(b, p)));
    pts.extend(b.pts.iter().copied().filter(|&p| !inside(a, p)));
    for (p0, p1) in a.edges() {
        for (q0, q1) in b.edges() {
            let (r, s) = (p1 - p0, q1 - q0);
            let den = r.cross(s);
            if den.abs() < 1e-12 {
                continue;
            }
            let t = (q0 - p0).cross(s) / den;
            let k = (q0 - p0).cross(r) / den;
            if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&k) {
                pts.push(p0 + r * t);
            }
        }
    }
    if pts.iter().any(|p| p.dist(c) < EPS) {
        return None;
    }
    let angle = |p: Point| math::atan2(p.y - c.y, p.x - c.x);
    pts.sort_by(|x, y| angle(*x).total_cmp(&angle(*y)));
    let mut ring: Vec<Point> = Vec::with_capacity(pts.len());
    for p in pts {
        if ring.last().is_none_or(|q| q.dist(p) > EPS) {
            ring.push(p);
        }
    }
    while ring.len() > 1 && ring[0].dist(ring[ring.len() - 1]) <= EPS {
        ring.pop();
    }
    let joined = Polygon::new(ring);
    let (area, areas) = (joined.signed_area(), (a.signed_area().abs(), b.signed_area().abs()));
    if joined.pts.len() < 3 || area < areas.0.max(areas.1) - 1e-9 || area > areas.0 + areas.1 + 1e-9 {
        return None;
    }
    Some(joined)
}

/// Whether `p`, a point on the outline of the pad with its pieces, lies on the gate of exit `e`:
/// the part of the outline that tracks of the net may cross.
pub fn on_mouth(e: &Exit, p: Point) -> bool {
    let d = p - (e.at + e.dir * MOUTH);
    d.dot(e.dir).abs() <= 2e-5 && d.cross(e.dir).abs() <= GATE.min(e.width / 4.0) + 2e-5
}

/// The exit pieces that carry one of `tracks`, each as a track of one line from the pin to the
/// exit point: what the writer adds to the tracks of the route, and what the length of a net
/// counts besides them.
pub fn stubs_of<'a>(board: &Board, tracks: impl Iterator<Item = &'a Track>) -> Vec<Track> {
    if board.exits.is_empty() {
        return Vec::new();
    }
    let mut by_net: crate::index::FastMap<NetId, Vec<usize>> = Default::default();
    for (i, e) in board.exits.iter().enumerate() {
        by_net.entry(board.pins[e.pin].net).or_default().push(i);
    }
    let mut used = vec![false; board.exits.len()];
    let mut out = Vec::new();
    for t in tracks {
        let (Some(&a), Some(&b), Some(list)) = (t.pts.first(), t.pts.last(), by_net.get(&t.net)) else {
            continue;
        };
        for end in [a, b] {
            let hit = list.iter().copied().find(|&i| {
                let e = &board.exits[i];
                e.at.dist(end) < 1e-6 && board.pins[e.pin].layers.has(t.layer)
            });
            let Some(i) = hit else { continue };
            if std::mem::replace(&mut used[i], true) {
                continue;
            }
            let e = &board.exits[i];
            let from = board.pins[e.pin].pos;
            out.push(Track {
                pts: vec![from, e.at],
                elems: vec![Elem::Line(from, e.at)],
                width: e.width,
                layer: t.layer,
                net: t.net,
                necks: Vec::new(),
                neck_w: e.width,
                origin: None,
                resting: false,
                fitted: Vec::new(),
                pattern: None,
                seg: u32::MAX,
                tap: None,
            });
        }
    }
    out
}

/// The board with the exit pieces that no track of `geo` took given back: those pads have their
/// outline of the file again, or that outline with the one piece in use. None if every piece is
/// in use. The exits stay listed. For the geometry steps after the search: a piece without a
/// track is no copper, and what it held away may come as near as the pad allows.
pub fn release_unused(board: &Board, geo: &Geometry) -> Option<Board> {
    let stubs = stub_tracks(board, geo);
    let used = |e: &Exit| {
        stubs
            .iter()
            .any(|s| s.pts[1].dist(e.at) < 1e-9 && s.pts[0].dist(board.pins[e.pin].pos) < 1e-9)
    };
    if board.exits.iter().all(used) {
        return None;
    }
    let mut b = board.clone();
    for (o, bare) in &board.bare_pads {
        let Shape::Polygon(pad) = bare else { continue };
        let mut shape = pad.clone();
        for e in board.exits.iter().filter(|e| e.obs == *o && used(e)) {
            let pin = board.pins[e.pin].pos;
            let nose = piece_at(&frame(pad, pin)?, pin, e.at, e.dir, e.width);
            shape = star_union(&shape, &nose, pin)?;
        }
        b.obstacles[*o].shape = Shape::Polygon(shape);
    }
    Some(b)
}

/// [`stubs_of`] all tracks of `geo`.
pub fn stub_tracks(board: &Board, geo: &Geometry) -> Vec<Track> {
    stubs_of(board, geo.tracks.iter())
}

/// `geo` with its exit pieces as tracks ([`stub_tracks`]), for a reader that knows the pads as
/// the file has them ([`Board::without_exits`]).
pub fn with_stubs(board: &Board, geo: &Geometry) -> Geometry {
    let mut g = geo.clone();
    g.tracks.extend(stub_tracks(board, geo));
    g
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{LayerSet, Net, Obstacle, Pin};
    use crate::geom::pt;
    use crate::relax::{RelaxConfig, relax};
    use crate::route::{Router, RouterConfig};
    use crate::topo::{Substrate, SubstrateConfig};
    use std::f64::consts::FRAC_PI_4;

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon {
        Polygon::new(vec![pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1)])
    }

    /// A row of `n` pads of part U at `pitch`, each `w` wide and 0.9 mm long with a net of its
    /// own, and for each net a square pad of part J in a row 1.6 mm apart, 5 mm further up.
    /// Tracks are `track` wide and keep `clearance`.
    fn row_board(n: usize, pitch: f64, w: f64, clearance: f64, track: f64) -> Board {
        let mut b = crate::topo::tests::two_pad_board();
        b.outline = vec![rect(0.0, 0.0, 20.0, 20.0)];
        b.classes[0].track_width = track;
        b.classes[0].clearance = clearance;
        b.nets.clear();
        b.pins.clear();
        b.obstacles.clear();
        let pad = |b: &mut Board, fp: &str, net: usize, at: Point, size: (f64, f64)| {
            let i = b.pins.len();
            b.nets[net].pins.push(i);
            b.footprint_obstacles.entry(fp.into()).or_default().push(i);
            b.obstacles.push(Obstacle {
                shape: Shape::Polygon(rect(
                    at.x - size.0 / 2.0,
                    at.y - size.1 / 2.0,
                    at.x + size.0 / 2.0,
                    at.y + size.1 / 2.0,
                )),
                layers: LayerSet::single(0),
                net: Some(net as NetId),
                kind: ObstacleKind::Pad,
                hole: None,
                clearance,
                pin: Some(i),
                locked: true,
            });
            b.pins.push(Pin {
                name: format!("{fp}.{}", net + 1),
                footprint: fp.into(),
                net: net as NetId,
                pos: at,
                layers: LayerSet::single(0),
                obstacles: vec![i],
                size,
            });
        };
        for i in 0..n {
            b.nets.push(Net {
                name: format!("N{i}"),
                class: 0,
                pins: Vec::new(),
                planes: LayerSet::default(),
                priority: 0,
            });
        }
        let mid = (n as f64 - 1.0) / 2.0;
        for i in 0..n {
            pad(&mut b, "U", i, pt(10.0 + (i as f64 - mid) * pitch, 8.0), (w, 0.9));
        }
        for i in 0..n {
            pad(&mut b, "J", i, pt(10.0 + (i as f64 - mid) * 1.6, 13.5), (0.5, 0.5));
        }
        b
    }

    #[test]
    fn a_long_pad_has_a_middle_line_and_a_square_one_has_none() {
        let f = frame(&rect(-0.1, -0.4, 0.1, 0.5), pt(0.0, 0.0)).unwrap();
        assert!(f.u.x.abs() < 1e-12 && (f.u.y.abs() - 1.0).abs() < 1e-12);
        assert!((f.half - 0.1).abs() < 1e-12);
        assert!((f.fwd + f.back - 0.9).abs() < 1e-12);
        assert!(frame(&rect(-0.2, -0.2, 0.2, 0.2), pt(0.0, 0.0)).is_none());
        // the pin off the middle line: no exit to construct
        assert!(frame(&rect(-0.1, -0.4, 0.1, 0.4), pt(0.05, 0.0)).is_none());
        // a pad at 30 degrees
        let turned = Polygon::new(rect(-0.4, -0.1, 0.4, 0.1).pts.iter().map(|p| p.rotate(0.5)).collect());
        let f = frame(&turned, pt(0.0, 0.0)).unwrap();
        assert!(f.u.cross(pt(1.0, 0.0).rotate(0.5)).abs() < 1e-9);
    }

    #[test]
    fn pad_and_piece_join_into_one_outline() {
        let pad = rect(-0.1, -0.4, 0.1, 0.4);
        // a piece narrower than the pad, as wide, and wider
        for w in [0.19, 0.2, 0.25] {
            let nose = piece(pt(0.0, -0.1), pt(0.0, 0.5), w / 2.0);
            let u = star_union(&pad, &nose, pt(0.0, 0.0)).unwrap();
            assert!(u.signed_area() > pad.signed_area());
            assert!(u.contains(pt(0.0, 0.49)), "the end of the piece");
            assert!(u.contains(pt(0.09, -0.39)), "the far corner of the pad");
            assert!(!u.contains(pt(0.0, 0.51)));
            assert!(!u.contains(pt(w / 2.0 + 0.02, 0.45)));
            assert!(u.contains(pt(w / 2.0 - 0.01, 0.45)));
        }
    }

    /// 0.4 mm pitch, pads 0.2 wide, track and clearance 0.19: 15 µm to spare on the middle line
    /// and nowhere else. Every pad of the row gets an exit at both ends, beyond the end of the
    /// pad by as much as a turn of 45 degrees needs to pass the corner of the next pad.
    #[test]
    fn a_pad_with_a_neighbour_at_0_4_mm_gets_its_exit() {
        let mut b = row_board(6, 0.4, 0.2, 0.19, 0.19);
        b.set_exits();
        assert!(b.tight_pads.is_empty());
        for p in 0..6 {
            let exits = b.exits_of(p);
            assert_eq!(exits.len(), 2, "pad {p}");
            for e in exits {
                assert!((e.spare - 0.015).abs() < 1e-9 && (e.width - 0.19).abs() < 1e-9, "{e:?}");
                let pin = b.pins[p].pos;
                assert!((e.at.x - pin.x).abs() < 1e-12, "on the middle line");
                // the corner of the next pad is 0.3 away from the middle line, and the turn
                // needs 0.19 + 0.095 from it: (0.3 + y) / sqrt 2 >= 0.285
                let y = (e.at.y - pin.y).abs() - 0.45;
                assert!((0.103..0.103 + 2.0 * STEP).contains(&y), "exit {y} beyond the pad");
                for q in [p.wrapping_sub(1), p + 1] {
                    for turn in [FRAC_PI_4, -FRAC_PI_4] {
                        if q < 6 {
                            let d = b.bare_shape(q).dist_to_seg(e.at, e.at + e.dir.rotate(turn) * RAY);
                            assert!(d >= 0.285 - 1e-9, "the turn passes the next pad at {d}");
                        }
                    }
                }
            }
            // the pad holds its pieces, and the file's shape is kept beside it
            assert!(b.obstacles[p].shape.dist_to_point(exits[0].at) <= 0.0);
            assert!(b.bare_shape(p).dist_to_point(exits[0].at) > 0.1);
        }
        // the pads of part J stand alone: no exit
        assert!((6..12).all(|p| b.exits_of(p).is_empty()));
        // and at 1.27 mm pitch a track has room beside the middle line
        let mut wide = row_board(6, 1.27, 0.6, 0.19, 0.19);
        wide.set_exits();
        assert!(wide.exits.is_empty());
        // a part that moves takes its exits along
        let moved = b.with_offsets(&[("U".to_string(), pt(0.25, -1.0))].into_iter().collect());
        assert_eq!(moved.exits.len(), b.exits.len());
        for (e, was) in moved.exits.iter().zip(&b.exits) {
            assert!(e.at.dist(was.at + pt(0.25, -1.0)) < 1e-9, "{e:?} {was:?}");
        }
        assert!(moved.obstacles[0].shape.dist_to_point(moved.exits[0].at) <= 0.0);
        // the board without exits is the board of the file again
        let plain = b.without_exits();
        assert!(plain.exits.is_empty() && plain.bare_pads.is_empty());
        assert!((plain.obstacles[0].shape.bbox().height() - 0.9).abs() < 1e-9);
    }

    /// The tracks of the row fan out to pads four times as far apart: tension pulls each of
    /// them sideways. With exits the track begins at the exit point and the piece from the pin
    /// to there is one line on the middle line, after the relaxation with its taut step and
    /// after the passes that know the rails of fine pitch pads.
    #[test]
    fn an_exit_stays_one_line_on_the_middle_line_through_relaxation_and_taut() {
        let mut b = row_board(6, 0.4, 0.2, 0.19, 0.19);
        b.set_exits();
        let sub = Substrate::build(&b, &SubstrateConfig::default());
        let mut r = Router::new(&b, &sub, RouterConfig::default());
        r.plan();
        for ci in 0..r.conns.len() {
            assert!(r.route_one(ci), "connection {ci}");
        }
        let mut geo = r.geometry();
        let on_exits = |geo: &Geometry, when: &str| {
            for p in 0..6 {
                let t = geo.tracks.iter().find(|t| t.net == p as NetId).expect("a track");
                let ends = [t.pts[0], t.pts[t.pts.len() - 1]];
                let e = b
                    .exits_of(p)
                    .iter()
                    .find(|e| ends.iter().any(|x| x.dist(e.at) < 1e-9))
                    .unwrap_or_else(|| panic!("{when}: the track of pad {p} ends at {ends:?}"));
                assert!(e.dir.y > 0.0, "{when}: pad {p} is left towards its goal");
                // nothing of the track lies beside the pad: it begins beyond its end
                assert!(
                    t.pts.iter().all(|q| q.y >= e.at.y - 1e-9),
                    "{when}: the track of pad {p} comes back beside the pad"
                );
            }
        };
        on_exits(&geo, "constructed");
        relax(&b, &sub, &mut geo, &RelaxConfig::default());
        on_exits(&geo, "relaxed");
        let rails = RelaxConfig {
            via_edge: true,
            ..Default::default()
        };
        relax(&b, &sub, &mut geo, &rails);
        on_exits(&geo, "relaxed with rails");
        // as copper: one line per pad from the pin to the exit point, and the board of the file
        // has no complaint about any of it
        let (plain, copper) = (b.without_exits(), with_stubs(&b, &geo));
        let stubs: Vec<&Track> = copper.tracks[geo.tracks.len()..].iter().collect();
        assert_eq!(stubs.len(), 6);
        for s in &stubs {
            let pin = b.pins[s.net as usize].pos;
            assert_eq!(s.elems.len(), 1);
            assert!(s.pts[0] == pin && s.pts[1].x == pin.x && s.pts[1].y > pin.y + 0.45);
        }
        let found = crate::check::check(&plain, &copper);
        assert!(found.is_empty(), "{found:?}");
        // the length of a net counts its piece
        let len = crate::matching::net_length(&b, &geo, 0).total();
        let same = crate::matching::net_length(&plain, &copper, 0).total();
        assert!((len - same).abs() < 1e-9 && len > 5.4, "{len} {same}");
    }

    /// The goals of the row lie in a column to one side, so every track turns hard right behind
    /// its exit point and the next one has to go around it. The tracks leave the row with 10 µm
    /// to spare between them (0.2 wide) or with none before they have spread (0.25 wide, 0.2 in
    /// the row). They stay on the middle line for the first half of the rail beyond the exit point
    /// (the arc around the turn may begin on the second half), and nothing comes nearer to its
    /// neighbour than the clearance.
    #[test]
    fn a_row_that_turns_hard_behind_its_exits_keeps_its_clearance() {
        for (clr, track) in [(0.19, 0.19), (0.19, 0.2), (0.19, 0.25)] {
            let mut b = row_board(6, 0.4, 0.2, clr, track);
            b.rules.min_track_width = 0.2f64.min(track);
            // the rightmost pad of the row goes to the lowest goal
            for i in 0..6 {
                let at = pt(13.5, 9.2 + (5 - i) as f64 * 1.0);
                b.pins[6 + i].pos = at;
                b.obstacles[6 + i].shape = Shape::Polygon(rect(at.x - 0.25, at.y - 0.25, at.x + 0.25, at.y + 0.25));
            }
            b.set_exits();
            assert_eq!(b.exits.len(), 12);
            let sub = Substrate::build(&b, &SubstrateConfig::default());
            let mut r = Router::new(&b, &sub, RouterConfig::default());
            r.plan();
            for ci in 0..r.conns.len() {
                assert!(r.route_one(ci), "connection {ci}");
            }
            let mut geo = r.geometry();
            relax(&b, &sub, &mut geo, &RelaxConfig::default());
            let rails = RelaxConfig {
                via_edge: true,
                ..Default::default()
            };
            relax(&b, &sub, &mut geo, &rails);
            let rail = knobs::EXIT_RAIL.num();
            for t in &geo.tracks {
                let e = b
                    .exits_of(t.net as usize)
                    .iter()
                    .find(|e| e.at.dist(t.pts[0]) < 1e-9 || e.at.dist(t.pts[t.pts.len() - 1]) < 1e-9)
                    .expect("the track ends at an exit");
                for q in t
                    .pts
                    .iter()
                    .filter(|q| (**q - e.at).dot(e.dir) < rail / 2.0 - 1e-6 && q.dist(e.at) < rail)
                {
                    assert!(
                        (*q - e.at).cross(e.dir).abs() < 0.008,
                        "track {track}: net {} leaves its middle line at {q:?}",
                        t.net
                    );
                }
            }
            let (plain, copper) = (b.without_exits(), with_stubs(&b, &geo));
            let found = crate::check::check(&plain, &copper);
            assert!(found.is_empty(), "track {track}: {found:?}");
        }
    }

    /// A piece that no track took is given back for the geometry after the search: the pad has
    /// its outline of the file again at that end, the piece in use stays, and nothing changes
    /// for the tracks or their pieces as copper.
    #[test]
    fn a_piece_without_a_track_is_given_back() {
        let mut b = row_board(6, 0.4, 0.2, 0.19, 0.19);
        b.set_exits();
        assert_eq!(b.exits.len(), 12);
        let sub = Substrate::build(&b, &SubstrateConfig::default());
        let mut r = Router::new(&b, &sub, RouterConfig::default());
        r.plan();
        for ci in 0..r.conns.len() {
            assert!(r.route_one(ci), "connection {ci}");
        }
        let geo = r.geometry();
        let free = release_unused(&b, &geo).expect("the six inward pieces have no track");
        assert_eq!(free.exits.len(), 12);
        for p in 0..6 {
            let (Shape::Polygon(all), Shape::Polygon(one), Shape::Polygon(pad)) =
                (&b.obstacles[p].shape, &free.obstacles[p].shape, b.bare_shape(p))
            else {
                panic!("polygons");
            };
            let (all, one, pad) = (all.bbox(), one.bbox(), pad.bbox());
            // outwards (up) as with both pieces, inwards (down) as the pad of the file
            assert!(all.min.y < pad.min.y - 0.05 && all.max.y > pad.max.y + 0.05);
            assert!((one.max.y - all.max.y).abs() < 1e-9 && (one.min.y - pad.min.y).abs() < 1e-9);
        }
        assert_eq!(stub_tracks(&free, &geo).len(), 6);
        // with every piece in use there is nothing to give back
        let mut both = geo.clone();
        for t in geo.tracks.iter() {
            let mut back = t.clone();
            let e = b.exits_of(t.net as usize).iter().find(|e| e.dir.y < 0.0).unwrap();
            back.pts = vec![e.at, e.at + e.dir];
            both.tracks.push(back);
        }
        assert!(release_unused(&b, &both).is_none());
    }

    /// The outline of a pad with exits is shut for its net except at the gates, also where the
    /// free room in front of the pad ends at the board edge.
    #[test]
    fn the_outline_of_a_pad_with_exits_opens_at_its_gates_only() {
        let mut b = row_board(6, 0.4, 0.2, 0.19, 0.19);
        b.outline = vec![rect(0.0, 7.0, 20.0, 20.0)];
        b.set_exits();
        assert_eq!(b.exits.len(), 12);
        let sub = Substrate::build(&b, &SubstrateConfig::default());
        let m = &sub.m[0];
        let mut gates = vec![0; b.exits.len()];
        for (&e, &x) in m.exit_edges.iter() {
            if x != crate::topo::NONE {
                gates[x as usize] += 1;
                // the gate is 0.02 wide, 0.01 beyond the exit point
                let [u, v] = m.edges[e as usize];
                let (pu, pv) = (m.verts[u as usize], m.verts[v as usize]);
                let at = b.exits[x as usize].at;
                assert!((pu.dist(pv) - 0.02).abs() < 1e-4 && ((pu + pv) * 0.5).dist(at) < 0.01 + 1e-4);
            }
        }
        assert!(gates.iter().all(|&n| n == 1), "{gates:?}");
        let shut = m.exit_edges.values().filter(|&&x| x == crate::topo::NONE).count();
        assert!(shut >= 12 * 6, "{shut} edges shut");
    }

    /// A row that leaves no legal way out is reported with what is missing, at the narrowest
    /// width the rules allow at the pad.
    #[test]
    fn a_row_too_tight_for_its_track_is_reported() {
        // 0.4 mm pitch, pads 0.26 wide, clearance 0.2: 0.07 from the middle line to the
        // clearance of the next pad. The track necks down to the board minimum of 0.15, and
        // half of that is 5 µm too much.
        let mut b = row_board(4, 0.4, 0.26, 0.2, 0.2);
        b.set_exits();
        assert_eq!(b.tight_pads.len(), 4);
        for t in &b.tight_pads {
            assert!(
                (t.width - 0.15).abs() < 1e-9 && (t.missing - 0.005).abs() < 1e-9,
                "{t:?}"
            );
        }
        assert!(b.exits.iter().all(|e| (e.width - 0.15).abs() < 1e-9));
        // a class that must keep its width (a minimum width rule, no courtyard exempt): the
        // piece stays as wide as the track, and more is missing
        b.classes[0].min_width = Some(0.2);
        b.set_exits();
        assert_eq!(b.tight_pads.len(), 4);
        assert!(
            b.tight_pads
                .iter()
                .all(|t| t.width == 0.2 && (t.missing - 0.03).abs() < 1e-9)
        );
        // pads 0.2 wide and a clearance of 0.19 leave room: nothing to report
        let mut fits = row_board(4, 0.4, 0.2, 0.19, 0.19);
        fits.set_exits();
        assert!(fits.tight_pads.is_empty() && !fits.exits.is_empty());
    }
}
