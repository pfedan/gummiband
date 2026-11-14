//! The passage of a finding: is there room for what lies there, or is there none?
//!
//! The check says that two things are too near. That alone does not say whose fault it is. If
//! the two tracks lie between two pads that leave room for both with their clearances, the
//! geometry laid them badly. If the pads leave less than both need, no geometry can help and
//! the search put one track too many through.
//!
//! [`passages`] answers that per spot. From the two items of the worst finding of a spot it
//! takes the straight cut through their nearest points and walks outwards on both sides,
//! through everything that can move (tracks and vias of the route), to the first thing that
//! cannot: a pad, a hole, a rule area, the board edge, copper that is kept, a zone. That is a
//! wall. A side with more than [`OPEN`] of free room beyond its last thing is open, with a
//! wall at that distance.
//!
//! **Needed** is the sum of the widths of what lies between the walls and of the clearances
//! between neighbours, each by the rule the check uses for that pair. A track counts with the
//! least width it may have there: what it has grown beyond its class is the geometry's to take
//! back, and is given beside the room. **Available** is the
//! distance from wall to wall along the cut. A track on a pad of its own net needs no room
//! beside that pad as far as the pad covers it; it counts with what it sticks out when it lies
//! on the middle, and with half of what the pad is wider as slack.
//!
//! What this is not: a track counts with its width across itself, also where the cut meets it
//! at a slant; a via counts only where the cut goes through its copper; a via can move here,
//! whatever holds it on other layers; and the cut is one line, not the narrowest place of the
//! passage, which can lie beside it.
//!
//! A via is one body through all layers, so the cut on one layer does not say whether it can
//! give way. [`via_room`] answers that for a via: between the via and every fixed thing around
//! it, on every layer, do the tracks that pass between the two fit? If not where the via is:
//! is there a place within [`NEAR`] where they do, that the via reaches without crossing a
//! track?

use crate::board::{Board, KEEPOUT_GAP, LayerId, NetId, ObstacleKind};
use crate::check::Violation;
use crate::geom::{Point, Polygon, Shape, point_seg_dist, point_seg_offset, seg_seg_nearest, segs_intersect};
use crate::realize::Geometry;

/// More free room than this beyond the last thing of a side, in mm, and the side is open.
pub const OPEN: f64 = 1.0;
/// How far along the cut things are looked at, in mm.
const REACH: f64 = 8.0;
/// How far from where it is a place for a via is looked for, in mm, and in what steps.
pub const NEAR: f64 = 0.5;
const STEP: f64 = 0.02;
/// What the check lets pass below a clearance, in mm.
const PASS: f64 = crate::check::TOL;

/// What a thing on the cut is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize)]
pub enum Kind {
    /// A track of the route: it can move.
    Track,
    /// A via of the route: it can move here (what holds it on other layers is not looked at).
    Via,
    Pad,
    /// A hole without copper.
    Hole,
    /// A rule area that keeps tracks out, or a solder mask opening of another net.
    Keepout,
    /// The board edge.
    Edge,
    /// Copper that stays: locked or kept tracks and vias, soft copper.
    Kept,
    /// A zone, a pour or a copper graphic.
    Zone,
    /// No thing within [`OPEN`]: free room.
    Open,
}

impl Kind {
    pub fn word(self) -> &'static str {
        match self {
            Kind::Track => "track",
            Kind::Via => "via",
            Kind::Pad => "pad",
            Kind::Hole => "hole",
            Kind::Keepout => "keepout",
            Kind::Edge => "edge",
            Kind::Kept => "kept copper",
            Kind::Zone => "zone",
            Kind::Open => "open",
        }
    }

    /// It cannot move.
    pub fn wall(self) -> bool {
        !matches!(self, Kind::Track | Kind::Via)
    }

    /// A wall of copper: a track of its net may lie on it.
    fn copper(self) -> bool {
        matches!(self, Kind::Pad | Kind::Kept | Kind::Zone)
    }
}

/// A thing on the cut.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Thing {
    pub kind: Kind,
    pub net: Option<NetId>,
    /// Where the cut enters and leaves it, in mm from the first item of the finding towards the
    /// second. A track: its centre line minus and plus half its width.
    pub lo: f64,
    pub hi: f64,
    /// A track: what it is wider here than its class asks (a width fitted to the room). That
    /// much the geometry can take back, so it is not counted as needed.
    pub grown: f64,
    #[serde(skip)]
    obstacle: Option<usize>,
    /// The track or via of the geometry.
    #[serde(skip)]
    id: usize,
}

impl Thing {
    fn mid(&self) -> f64 {
        (self.lo + self.hi) / 2.0
    }
    pub fn width(&self) -> f64 {
        self.hi - self.lo
    }
    /// The least width it may have: a track without what it has grown.
    pub fn least(&self) -> f64 {
        self.width() - self.grown
    }
}

/// The passage of one spot.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Passage {
    /// The worst finding of the spot: what it is, where, and how much it lacks.
    pub what: String,
    pub nets: (NetId, Option<NetId>),
    pub at: Point,
    pub missing: f64,
    /// The findings of the spot (the check has one per piece of track).
    pub pieces: usize,
    /// The cut: from the nearest point of the first item, this direction, on this layer.
    pub from: Point,
    pub dir: Point,
    pub layer: LayerId,
    /// From wall to wall in order: the wall before the first item, what lies between (the two
    /// items if they can move, and the tracks and vias beside them; copper of one net that
    /// overlaps is one thing), and the wall behind the second item.
    pub chain: Vec<Thing>,
    /// The clearance the check asks between each thing of the chain and the next (0: one net).
    pub clearances: Vec<f64>,
    /// The two items of the finding, as places in the chain.
    pub items: [usize; 2],
    /// The first thing after the wall lies on it, its own copper, and so the last before the
    /// other wall.
    pub own: [bool; 2],
    /// The two items cross or touch: there is no cut, and nothing above is measured.
    pub crossing: bool,
}

impl Passage {
    /// What thing `i` of the chain takes of the room between things `a` and `b`: the least
    /// width it may have, or on its own copper what it sticks out when it lies on the middle.
    fn takes(&self, i: usize) -> f64 {
        let k = self.chain.len() - 1;
        let x = &self.chain[i];
        if i == 1 && self.own[0] {
            ((x.least() - self.chain[0].width()) / 2.0).max(0.0)
        } else if i + 1 == k && self.own[1] {
            ((x.least() - self.chain[k].width()) / 2.0).max(0.0)
        } else {
            x.least()
        }
    }

    /// Half of what a pad is wider than the track of its net on it: room the track has on its
    /// own copper, for the thing beside wall 0 or the last wall.
    fn slack_on(&self, wall: usize) -> f64 {
        let k = self.chain.len() - 1;
        let (w, x, own) = if wall == 0 {
            (&self.chain[0], &self.chain[1.min(k)], self.own[0])
        } else {
            (&self.chain[k], &self.chain[k.saturating_sub(1)], self.own[1])
        };
        if own {
            ((w.width() - x.least()) / 2.0).max(0.0)
        } else {
            0.0
        }
    }

    /// The room between things `a` and `b` of the chain, taken as walls, when every clearance
    /// gets `margin` on top: negative, and so much is missing.
    pub fn room_between(&self, a: usize, b: usize, margin: f64) -> f64 {
        let k = self.chain.len() - 1;
        let mut room = self.chain[b].lo - self.chain[a].hi;
        if a == 0 {
            room += self.slack_on(0);
        }
        if b == k {
            room += self.slack_on(k);
        }
        for i in a + 1..b {
            room -= self.takes(i);
        }
        for c in &self.clearances[a..b] {
            if *c > 0.0 {
                room -= c + margin;
            }
        }
        room
    }

    /// Wall to wall along the cut, in mm.
    pub fn available(&self) -> f64 {
        self.chain[self.chain.len() - 1].lo - self.chain[0].hi
    }

    pub fn own_slack(&self) -> f64 {
        self.slack_on(0) + self.slack_on(self.chain.len() - 1)
    }

    /// Widths and clearances of everything between the walls, in mm.
    pub fn needed(&self) -> f64 {
        self.available() + self.own_slack() - self.room(0.0)
    }

    /// What the tracks between things `a` and `b` of the chain are wider than their classes
    /// ask: the room counts them without it, and whoever lays the passage has to take that
    /// much back where the room is less than this.
    pub fn grown_between(&self, a: usize, b: usize) -> f64 {
        self.chain[a + 1..b.max(a + 1)].iter().map(|t| t.grown).sum()
    }

    /// The same from wall to wall.
    pub fn grown(&self) -> f64 {
        self.grown_between(0, self.chain.len() - 1)
    }

    /// The same with the vias where they are.
    pub fn grown_vias_stay(&self) -> f64 {
        let (a, b) = self.stay();
        self.grown_between(a, b)
    }

    /// The clearances that count: a margin on top is needed once for each.
    pub fn gaps(&self) -> usize {
        self.clearances.iter().filter(|c| **c > 0.0).count()
    }

    /// The room from wall to wall with `margin` on top of every clearance.
    pub fn room(&self, margin: f64) -> f64 {
        self.room_between(0, self.chain.len() - 1, margin)
    }

    /// The same with the vias where they are: from the last via or wall before the finding to
    /// the first behind it.
    pub fn room_vias_stay(&self, margin: f64) -> f64 {
        let (a, b) = self.stay();
        self.room_between(a, b, margin)
    }

    /// The last via or wall before the finding and the first behind it, as places in the chain.
    fn stay(&self) -> (usize, usize) {
        let k = self.chain.len() - 1;
        let fixed = |i: usize| self.chain[i].kind == Kind::Via || i == 0 || i == k;
        let a = (0..=self.items[0]).rev().find(|&i| fixed(i)).unwrap_or(0);
        let b = (self.items[1]..=k).find(|&i| fixed(i)).unwrap_or(k);
        (a, b)
    }

    /// What each gap of the chain has over its clearance and `margin` as the copper lies.
    /// A thing on its own copper: how far it lies off the middle towards the other wall.
    fn spare(&self, g: usize, margin: f64) -> f64 {
        let k = self.chain.len() - 1;
        let (x, y) = (&self.chain[g], &self.chain[g + 1]);
        if g == 0 && self.own[0] {
            return y.mid() - x.mid();
        }
        if g + 1 == k && self.own[1] {
            return y.mid() - x.mid();
        }
        let c = self.clearances[g];
        y.lo - x.hi - c - if c > 0.0 { margin } else { 0.0 }
    }

    /// The fewest things that have to move so that the two items get their clearance and
    /// `margin`, each moved thing using up what the gap behind it has to spare. None: the gaps
    /// between the walls do not have it. With `vias` false no via moves.
    pub fn movers(&self, margin: f64, vias: bool) -> Option<usize> {
        let k = self.chain.len() - 1;
        let [p, q] = self.items;
        if q != p + 1 {
            return None;
        }
        let lacking = -self.spare(p, margin);
        if lacking <= 0.0 {
            return Some(0);
        }
        let can = |i: usize| i > 0 && i < k && (vias || self.chain[i].kind != Kind::Via);
        // what moving the first l things before the gap, or the first r behind it, frees
        let mut left = vec![0.0];
        for l in 1..=p {
            if !can(p + 1 - l) {
                break;
            }
            left.push(left[l - 1] + self.spare(p - l, margin));
        }
        let mut right = vec![0.0];
        for r in 1..=k - q {
            if !can(q + r - 1) {
                break;
            }
            right.push(right[r - 1] + self.spare(q + r - 1, margin));
        }
        let mut best: Option<usize> = None;
        for (l, a) in left.iter().enumerate() {
            for (r, b) in right.iter().enumerate() {
                if a + b >= lacking - 1e-9 && best.is_none_or(|x| l + r < x) {
                    best = Some(l + r);
                }
            }
        }
        best
    }

    /// The vias of the route among the two items of the finding (a track that lies on its via
    /// counts as that via), as indices of the geometry.
    pub fn via_items(&self) -> Vec<usize> {
        let mut out: Vec<usize> = self
            .items
            .iter()
            .filter(|&&i| self.chain[i].kind == Kind::Via)
            .map(|&i| self.chain[i].id)
            .collect();
        out.dedup();
        out
    }

    /// Can the tracks give way here, with the vias where they are and `margin` on top of every
    /// clearance? With a via among the two items: it has room where it is, as one body through
    /// all layers. Without: the passage has room between the nearest vias or walls on both
    /// sides, at the widths the tracks have.
    pub fn gives_way(&self, board: &Board, geo: &Geometry, margin: f64) -> bool {
        if self.crossing {
            return false;
        }
        let vias = self.via_items();
        if !vias.is_empty() {
            return vias.iter().any(|&vi| {
                let v = &geo.vias[vi];
                let local = Local::around(board, geo, v.at, NEAR + OPEN + v.diameter + 1.0);
                local.via_at(vi, v.at, margin, &[self.dir], Some(-PASS)).0 >= -PASS
            });
        }
        let room = self.room_vias_stay(margin);
        room >= 0.0 && room >= self.grown_vias_stay()
    }

    /// The kind of the finding: the first word of what the check calls it.
    pub fn kind(&self) -> &str {
        self.what.split(' ').next().unwrap_or("")
    }

    pub fn walls(&self) -> [&Thing; 2] {
        [&self.chain[0], &self.chain[self.chain.len() - 1]]
    }

    /// What lies between the walls.
    pub fn between(&self) -> &[Thing] {
        &self.chain[1..self.chain.len() - 1]
    }

    pub fn tracks(&self) -> usize {
        self.between().iter().filter(|t| t.kind == Kind::Track).count()
    }

    pub fn vias(&self) -> usize {
        self.between().iter().filter(|t| t.kind == Kind::Via).count()
    }
}

/// The centre line of an item of the geometry.
#[derive(Clone, Copy)]
enum Line {
    Seg(Point, Point),
    At(Point),
}

fn on_seg(p: Point, a: Point, b: Point) -> Point {
    p - point_seg_offset(p, a, b)
}

/// The nearest points of a centre line and a segment. Where two segments run side by side at
/// one distance, the pair in the middle of the stretch they share: a cut at an end of it would
/// pass by what lies beside the stretch.
fn nearest(l: Line, c: Point, d: Point) -> (Point, Point) {
    match l {
        Line::Seg(a, b) => {
            let pq = seg_seg_nearest(a, b, c, d);
            let ab = b - a;
            let l2 = ab.len2();
            if l2 < 1e-18 {
                return pq;
            }
            let s = |p: Point| ((p - a).dot(ab) / l2).clamp(0.0, 1.0);
            let p = a + ab * ((s(c) + s(d)) / 2.0);
            let q = on_seg(p, c, d);
            if p.dist(q) <= pq.0.dist(pq.1) + 1e-7 {
                (p, q)
            } else {
                pq
            }
        }
        Line::At(p) => (p, on_seg(p, c, d)),
    }
}

/// The nearest points of a centre line and the boundary of a shape.
fn nearest_shape(l: Line, s: &Shape) -> (Point, Point) {
    let poly = s.to_polygon(0.002);
    poly.edges()
        .map(|(c, d)| nearest(l, c, d))
        .min_by(|x, y| x.0.dist(x.1).total_cmp(&y.0.dist(y.1)))
        .unwrap_or(match l {
            Line::Seg(a, _) | Line::At(a) => (a, a),
        })
}

/// Where the line from o along n crosses the segment a-b, as the distance from o.
fn cross_seg(o: Point, n: Point, a: Point, b: Point) -> Option<f64> {
    let e = b - a;
    let den = n.cross(e);
    if den.abs() < 1e-12 {
        return None;
    }
    let s = (a - o).cross(n) / den;
    let t = (a - o).cross(e) / den;
    ((-1e-9..=1.0 + 1e-9).contains(&s) && t.abs() <= REACH).then_some(t)
}

/// The findings of the check, one group per spot: the worst finding and how many there are.
/// A spot is what [`crate::check::spots`] calls one: a pair of nets within 2 mm.
fn worst(violations: &[Violation]) -> Vec<(&Violation, usize)> {
    let mut out: Vec<(&Violation, usize)> = Vec::new();
    let key = |v: &Violation| {
        let k = v.nets.1.unwrap_or(u32::MAX);
        (v.nets.0.min(k), v.nets.0.max(k))
    };
    for v in violations.iter().filter(|v| !v.is_width()) {
        match out.iter_mut().find(|(w, _)| key(w) == key(v) && w.at.dist(v.at) < 2.0) {
            Some((w, n)) => {
                *n += 1;
                if v.required - v.actual > w.required - w.actual {
                    *w = v;
                }
            }
            None => out.push((v, 1)),
        }
    }
    out
}

/// The clearance the check asks between two neighbours on the cut.
fn clearance(board: &Board, a: &Thing, b: &Thing) -> f64 {
    let clr = |n: NetId| board.class_of(n).clearance.max(board.rules.min_clearance);
    // the one that can move first
    let (m, w) = if a.kind.wall() { (b, a) } else { (a, b) };
    let Some(net) = m.net.filter(|_| !m.kind.wall()) else {
        return 0.0;
    };
    match w.kind {
        Kind::Open => 0.0,
        Kind::Edge => board.rules.edge_clearance,
        Kind::Track | Kind::Via => match w.net {
            Some(o) if o == net => 0.0,
            Some(o) if m.kind == Kind::Track && w.kind == Kind::Track => board.track_clearance(net, o),
            Some(o) => clr(net).max(clr(o)).max(board.item_gap(net, Some(o))),
            None => clr(net),
        },
        Kind::Keepout => KEEPOUT_GAP,
        Kind::Hole => board.hole_gap(net),
        Kind::Pad | Kind::Kept | Kind::Zone => {
            if w.net == Some(net) {
                return 0.0;
            }
            let own = w.obstacle.map_or(0.0, |o| board.obstacles[o].clearance);
            own.max(clr(net)).max(board.item_gap(net, w.net))
        }
    }
}

/// The clearance the check asks between a track of `net` and a thing of this kind (of net
/// `other`; a board item: its obstacle).
pub(crate) fn asked(board: &Board, net: NetId, kind: Kind, other: Option<NetId>, obstacle: Option<usize>) -> f64 {
    let thing = |kind, net, obstacle| Thing {
        kind,
        net,
        lo: 0.0,
        hi: 0.0,
        grown: 0.0,
        obstacle,
        id: 0,
    };
    clearance(
        board,
        &thing(Kind::Track, Some(net), None),
        &thing(kind, other, obstacle),
    )
}

/// What an obstacle of the board is on a cut (None: nothing that tracks care about).
pub(crate) fn cut_kind(kind: &ObstacleKind) -> Option<Kind> {
    Some(match kind {
        ObstacleKind::Pad => Kind::Pad,
        ObstacleKind::Track | ObstacleKind::Via => Kind::Kept,
        ObstacleKind::Keepout { tracks, mask, .. } if *tracks || *mask => Kind::Keepout,
        ObstacleKind::Keepout { .. } => return None,
        ObstacleKind::Zone | ObstacleKind::Pour | ObstacleKind::Graphic => Kind::Zone,
        ObstacleKind::Hole => Kind::Hole,
    })
}

/// What lies around a place, taken once: the cuts through it look only at this.
struct Local<'a> {
    board: &'a Board,
    geo: &'a Geometry,
    tracks: Vec<usize>,
    vias: Vec<usize>,
    /// The obstacle and its outline (None: a circle).
    obstacles: Vec<(usize, Option<Polygon>)>,
    edges: Vec<(Point, Point)>,
}

impl<'a> Local<'a> {
    fn around(board: &'a Board, geo: &'a Geometry, c: Point, reach: f64) -> Local<'a> {
        let tracks = (0..geo.tracks.len())
            .filter(|&ti| {
                geo.tracks[ti]
                    .pts
                    .windows(2)
                    .any(|w| point_seg_dist(c, w[0], w[1]) <= reach)
            })
            .collect();
        let vias = (0..geo.vias.len())
            .filter(|&vi| geo.vias[vi].at.dist(c) <= reach + geo.vias[vi].diameter)
            .collect();
        let obstacles = board
            .obstacles
            .iter()
            .enumerate()
            .filter(|(_, ob)| {
                let bb = ob.shape.bbox();
                (bb.center() - c).len() <= reach + bb.width() + bb.height()
            })
            .map(|(oi, ob)| {
                (
                    oi,
                    match &ob.shape {
                        Shape::Circle { .. } => None,
                        s => Some(s.to_polygon(0.002)),
                    },
                )
            })
            .collect();
        let edges = board
            .outline
            .iter()
            .flat_map(|poly| poly.edges())
            .filter(|(a, b)| point_seg_dist(c, *a, *b) <= reach)
            .collect();
        Local {
            board,
            geo,
            tracks,
            vias,
            obstacles,
            edges,
        }
    }

    /// Everything the line from o along n meets on a layer, unordered. Without via `but` and
    /// without the copper of net `not` (a mask opening of that net as well).
    fn things(&self, o: Point, n: Point, layer: LayerId, but: Option<usize>, not: Option<NetId>) -> Vec<Thing> {
        let (board, geo) = (self.board, self.geo);
        let mut out = Vec::new();
        for &ti in &self.tracks {
            let t = &geo.tracks[ti];
            if t.layer != layer || Some(t.net) == not {
                continue;
            }
            let mut hits: Vec<(f64, f64)> = t
                .pts
                .windows(2)
                .filter_map(|w| cross_seg(o, n, w[0], w[1]).map(|x| (x, t.piece_width(w[0], w[1]))))
                .collect();
            hits.sort_by(|a, b| a.0.total_cmp(&b.0));
            hits.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-6);
            let kind = if t.origin.is_some() { Kind::Kept } else { Kind::Track };
            out.extend(hits.into_iter().map(|(x, w)| Thing {
                kind,
                net: Some(t.net),
                lo: x - w / 2.0,
                hi: x + w / 2.0,
                obstacle: None,
                grown: (w - t.width).max(0.0),
                id: ti,
            }));
        }
        let circle = |c: Point, r: f64| {
            let (s, h) = ((c - o).dot(n), (c - o).cross(n).abs());
            (h < r && s.abs() <= REACH).then(|| {
                let half = (r * r - h * h).sqrt();
                (s - half, s + half)
            })
        };
        for &vi in &self.vias {
            let v = &geo.vias[vi];
            if v.drill <= 0.0 || Some(vi) == but || Some(v.net) == not {
                continue;
            }
            if let Some((lo, hi)) = circle(v.at, v.diameter / 2.0) {
                out.push(Thing {
                    kind: if v.origin.is_some() { Kind::Kept } else { Kind::Via },
                    net: Some(v.net),
                    lo,
                    hi,
                    obstacle: None,
                    grown: 0.0,
                    id: vi,
                });
            }
        }
        for (oi, poly) in &self.obstacles {
            let ob = &board.obstacles[*oi];
            let Some(kind) = cut_kind(&ob.kind) else { continue };
            if !(ob.layers.has(layer) || kind == Kind::Hole) || (ob.net.is_some() && ob.net == not) {
                continue;
            }
            let span = match (&ob.shape, poly) {
                (Shape::Circle { c, r }, _) => circle(*c, *r),
                (_, Some(poly)) => {
                    let ts: Vec<f64> = poly.edges().filter_map(|(a, b)| cross_seg(o, n, a, b)).collect();
                    (ts.len() >= 2).then(|| {
                        (
                            ts.iter().copied().fold(f64::INFINITY, f64::min),
                            ts.iter().copied().fold(f64::NEG_INFINITY, f64::max),
                        )
                    })
                }
                _ => None,
            };
            if let Some((lo, hi)) = span {
                out.push(Thing {
                    kind,
                    net: ob.net,
                    lo,
                    hi,
                    obstacle: Some(*oi),
                    grown: 0.0,
                    id: *oi,
                });
            }
        }
        for &(a, b) in &self.edges {
            if let Some(x) = cross_seg(o, n, a, b) {
                out.push(Thing {
                    kind: Kind::Edge,
                    net: None,
                    lo: x,
                    hi: x,
                    obstacle: None,
                    grown: 0.0,
                    id: 0,
                });
            }
        }
        out
    }
}

/// The fixed thing that leaves a via the least room.
#[derive(Debug, Clone, Copy, serde::Serialize)]
pub struct Tight {
    pub layer: LayerId,
    pub kind: Kind,
    pub net: Option<NetId>,
    /// Tracks that pass between the via and the thing.
    pub between: usize,
    /// From the copper of the via to the thing, in mm, and in which direction.
    pub available: f64,
    pub dir: Point,
}

/// The room of a via of the route as one body through all layers, see [`via_room`].
#[derive(Debug, Clone, serde::Serialize)]
pub struct ViaRoom {
    pub via: usize,
    pub at: Point,
    /// The least room where the via is, in mm: over all layers and all fixed things around it
    /// (pads, holes, rule areas, the edge, kept copper, other vias), the distance between the
    /// two less the clearances and the widths of the tracks that pass between them. Negative:
    /// so much is missing there, whatever the tracks do.
    pub stays: f64,
    /// Where that is.
    pub tight: Option<Tight>,
    /// The nearest place within [`NEAR`] where nothing is missing and that the via reaches
    /// without crossing a track, and how far it is (0: it can stay). None: there is none.
    pub home: Option<(Point, f64)>,
}

/// The least room so far, and where.
struct Least(f64, Option<Tight>);

impl Least {
    /// Room `room` towards the point `to`, where `need` is asked of what is `kind`.
    #[allow(clippy::too_many_arguments)]
    fn take(
        &mut self,
        room: f64,
        need: f64,
        layer: LayerId,
        kind: Kind,
        net: Option<NetId>,
        between: usize,
        dir: Point,
    ) {
        if room < self.0 {
            *self = Least(
                room,
                Some(Tight {
                    layer,
                    kind,
                    net,
                    between,
                    available: room + need,
                    dir,
                }),
            );
        }
    }
}

impl Local<'_> {
    /// The least room of via `vi` if its centre were at `p`, with `margin` on top of every
    /// clearance, and where it is. `also`: more directions to cut in. Stops early below `stop`.
    fn via_at(&self, vi: usize, p: Point, margin: f64, also: &[Point], stop: Option<f64>) -> (f64, Option<Tight>) {
        let (board, geo) = (self.board, self.geo);
        let v = &geo.vias[vi];
        let (r, net) = (v.diameter / 2.0, v.net);
        let clr = |n: NetId| board.class_of(n).clearance.max(board.rules.min_clearance);
        let routing = board.routing_layers();
        let mut worst = Least(f64::INFINITY, None);
        let towards = |q: Point| {
            if q.dist(p) > 1e-9 {
                (q - p).norm()
            } else {
                Point::default()
            }
        };
        // the fixed things themselves, as the check measures them
        let any = routing.iter().next().unwrap_or(0);
        let me = Shape::Circle { c: p, r };
        for (oi, _) in &self.obstacles {
            let o = &board.obstacles[*oi];
            let oc = o.shape.bbox().center();
            if let Some(h) = o.hole
                && !(o.net == Some(net) && v.at.dist(oc) < 1e-3)
            {
                let need = h + v.drill / 2.0 + board.rules.hole_to_hole - r;
                worst.take(p.dist(oc) - r - need, need, any, Kind::Hole, o.net, 0, towards(oc));
            }
            if o.net == Some(net) && o.kind != ObstacleKind::Hole {
                continue;
            }
            let on = o.layers.iter().find(|&l| routing.has(l));
            if on.is_none() && o.hole.is_none() && !matches!(o.kind, ObstacleKind::Keepout { vias: true, .. }) {
                continue;
            }
            let (need, kind) = match o.kind {
                ObstacleKind::Keepout { vias, .. } => {
                    if !vias {
                        continue;
                    }
                    (KEEPOUT_GAP, Kind::Keepout)
                }
                ObstacleKind::Hole => (board.hole_gap(net), Kind::Hole),
                _ => (
                    o.clearance.max(clr(net)).max(board.item_gap(net, o.net)),
                    cut_kind(&o.kind).unwrap_or(Kind::Kept),
                ),
            };
            let need = need + margin;
            worst.take(
                o.shape.dist(&me) - need,
                need,
                on.unwrap_or(any),
                kind,
                o.net,
                0,
                towards(oc),
            );
        }
        for &(a, b) in &self.edges {
            let need = board.rules.edge_clearance + margin;
            let q = on_seg(p, a, b);
            worst.take(q.dist(p) - r - need, need, any, Kind::Edge, None, 0, towards(q));
        }
        for &vj in &self.vias {
            let x = &geo.vias[vj];
            if vj == vi || x.drill <= 0.0 {
                continue;
            }
            let d = p.dist(x.at) - r - x.diameter / 2.0;
            let holes = v.drill / 2.0 + x.drill / 2.0 + board.rules.hole_to_hole - r - x.diameter / 2.0;
            worst.take(d - holes, holes, any, Kind::Via, Some(x.net), 0, towards(x.at));
            if x.net != net {
                let need = clr(net).max(clr(x.net)).max(board.item_gap(net, Some(x.net))) + margin;
                worst.take(d - need, need, any, Kind::Via, Some(x.net), 0, towards(x.at));
            }
        }
        if stop.is_some_and(|s| worst.0 < s) {
            return (worst.0, worst.1);
        }
        // and the tracks between the via and each of them, layer by layer
        let me = Thing {
            kind: Kind::Via,
            net: Some(net),
            lo: -r,
            hi: r,
            obstacle: None,
            grown: 0.0,
            id: vi,
        };
        let far = r + OPEN + 0.5;
        let fixed = |x: &Thing| x.kind.wall() || x.kind == Kind::Via;
        for layer in routing.iter() {
            // towards the nearest point of every fixed thing, and all around
            let mut dirs: Vec<Point> = (0..16)
                .map(|k| Point { x: 1.0, y: 0.0 }.rotate(k as f64 * std::f64::consts::PI / 8.0))
                .collect();
            dirs.extend(also.iter().flat_map(|d| [*d, *d * -1.0]));
            for (oi, poly) in &self.obstacles {
                let o = &board.obstacles[*oi];
                let Some(kind) = cut_kind(&o.kind) else { continue };
                if !(o.layers.has(layer) || kind == Kind::Hole) || o.net == Some(net) {
                    continue;
                }
                let q = match (&o.shape, poly) {
                    (Shape::Circle { c, .. }, _) => *c,
                    (_, Some(poly)) => poly
                        .edges()
                        .map(|(a, b)| on_seg(p, a, b))
                        .min_by(|x, y| x.dist(p).total_cmp(&y.dist(p)))
                        .unwrap_or(p),
                    _ => continue,
                };
                if q.dist(p) > 1e-6 && q.dist(p) < far {
                    dirs.push((q - p).norm());
                }
            }
            for &vj in &self.vias {
                let x = &geo.vias[vj];
                if vj != vi && x.drill > 0.0 && x.at.dist(p) > 1e-6 && x.at.dist(p) < far + x.diameter / 2.0 {
                    dirs.push((x.at - p).norm());
                }
            }
            for &(a, b) in &self.edges {
                let q = on_seg(p, a, b);
                if q.dist(p) > 1e-6 && q.dist(p) < far {
                    dirs.push((q - p).norm());
                }
            }
            for n in dirs {
                let mut xs: Vec<Thing> = self
                    .things(p, n, layer, Some(vi), Some(net))
                    .into_iter()
                    .filter(|x| x.mid() > 0.0 && x.hi > 0.0)
                    .collect();
                // A track that lies on fixed copper of its net is part of it. One that ends on it
                // does not pass between: it can come to its pad or via from where there is room.
                let mut part = vec![false; xs.len()];
                for i in 0..xs.len() {
                    if !(fixed(&xs[i]) && xs[i].net.is_some() && (xs[i].kind.copper() || xs[i].kind == Kind::Via)) {
                        continue;
                    }
                    let ends_on = |t: &crate::realize::Track| {
                        [t.pts[0], t.pts[t.pts.len() - 1]].iter().any(|e| match xs[i].obstacle {
                            Some(o) => board.obstacles[o].shape.dist_to_point(*e) < 1e-6,
                            None => geo.vias[xs[i].id].at.dist(*e) < geo.vias[xs[i].id].diameter / 2.0,
                        })
                    };
                    let mut lo = xs[i].lo;
                    for j in 0..xs.len() {
                        if fixed(&xs[j]) || xs[j].net != xs[i].net {
                            continue;
                        }
                        if xs[j].lo < xs[i].hi + 1e-6 && xs[i].lo < xs[j].hi + 1e-6 {
                            part[j] = true;
                            lo = lo.min(xs[j].lo);
                        } else if ends_on(&geo.tracks[xs[j].id]) {
                            part[j] = true;
                        }
                    }
                    xs[i].lo = lo;
                }
                let mut xs: Vec<&Thing> = xs.iter().zip(&part).filter(|x| !*x.1).map(|x| x.0).collect();
                xs.sort_by(|a, b| a.lo.total_cmp(&b.lo));
                let (mut prev, mut need, mut between) = (me.clone(), 0.0, 0);
                for x in xs {
                    if x.lo - prev.hi > OPEN {
                        break;
                    }
                    // copper of one net that overlaps is one thing: two tracks at a junction
                    if between > 0 && !fixed(x) && x.net == prev.net && x.lo < prev.hi + 1e-6 {
                        need += (x.hi - prev.hi).max(0.0);
                        prev.hi = prev.hi.max(x.hi);
                        continue;
                    }
                    let c = clearance(board, &prev, x);
                    let c = board.courtyard_clearance_at(p, c).map_or(c, |min| c.min(min));
                    if c > 0.0 {
                        need += c + margin;
                    }
                    if fixed(x) {
                        // with no track between, the thing itself was measured above
                        if between > 0 {
                            worst.take(x.lo - r - need, need, layer, x.kind, x.net, between, n);
                        }
                        break;
                    }
                    need += x.least();
                    between += 1;
                    prev = x.clone();
                }
                if stop.is_some_and(|s| worst.0 < s) {
                    return (worst.0, worst.1);
                }
            }
        }
        (worst.0, worst.1)
    }

    /// Does via `vi` get to `p` in a straight line without crossing a track of another net, a
    /// fixed thing or another via?
    fn reaches(&self, vi: usize, p: Point) -> bool {
        let (board, geo) = (self.board, self.geo);
        let v = &geo.vias[vi];
        let from = v.at;
        if from.dist(p) < 1e-9 {
            return true;
        }
        self.tracks.iter().all(|&ti| {
            let t = &geo.tracks[ti];
            t.net == v.net || !t.pts.windows(2).any(|w| segs_intersect(from, p, w[0], w[1]))
        }) && self.vias.iter().all(|&vj| {
            let x = &geo.vias[vj];
            vj == vi || x.drill <= 0.0 || point_seg_dist(x.at, from, p) >= x.diameter / 2.0
        }) && self.obstacles.iter().all(|(oi, poly)| {
            let o = &board.obstacles[*oi];
            if (o.net == Some(v.net) && o.kind != ObstacleKind::Hole) || cut_kind(&o.kind).is_none() {
                return true;
            }
            match (&o.shape, poly) {
                (Shape::Circle { c, r }, _) => point_seg_dist(*c, from, p) >= *r,
                (_, Some(poly)) => !poly.edges().any(|(a, b)| segs_intersect(from, p, a, b)),
                _ => true,
            }
        })
    }
}

/// The room of via `vi` of the route as one body through all layers, with `margin` on top of
/// every clearance: where it is, and if something is missing there, the nearest place within
/// [`NEAR`] where nothing is. The tracks count as free to move aside as long as they fit between
/// the via and the fixed things; the other vias stay; the tracks of the via's own net follow it
/// and are not counted, and neither is a track that ends on the fixed thing itself. `also`:
/// directions to cut in on top of those towards the fixed things.
pub fn via_room(board: &Board, geo: &Geometry, vi: usize, margin: f64, also: &[Point]) -> ViaRoom {
    let v = &geo.vias[vi];
    let local = Local::around(board, geo, v.at, NEAR + OPEN + v.diameter + 1.0);
    let (stays, tight) = local.via_at(vi, v.at, margin, also, None);
    let home = if stays >= -PASS {
        Some((v.at, 0.0))
    } else {
        let n = (NEAR / STEP).floor() as i32;
        let mut places: Vec<Point> = (-n..=n)
            .flat_map(|i| {
                (-n..=n).map(move |j| Point {
                    x: i as f64 * STEP,
                    y: j as f64 * STEP,
                })
            })
            .filter(|d| d.len() <= NEAR && d.len() > 0.0)
            .collect();
        places.sort_by(|a, b| a.len().total_cmp(&b.len()));
        places
            .into_iter()
            .map(|d| v.at + d)
            .find(|&p| local.reaches(vi, p) && local.via_at(vi, p, margin, &[], Some(-PASS)).0 >= -PASS)
            .map(|p| (p, p.dist(v.at)))
    };
    ViaRoom {
        via: vi,
        at: v.at,
        stays,
        tight,
        home,
    }
}

fn open(at: f64) -> Thing {
    Thing {
        kind: Kind::Open,
        net: None,
        lo: at,
        hi: at,
        obstacle: None,
        grown: 0.0,
        id: 0,
    }
}

/// From thing `from` outwards (`sign` -1 or 1): the things passed that can move, and the wall.
/// The flag says that the last thing before the wall lies on it, its own copper.
fn walk(all: &[Thing], from: usize, sign: f64, skip: &[usize]) -> (Vec<usize>, Thing, bool) {
    let mut chain: Vec<usize> = Vec::new();
    let mut cur = from;
    loop {
        let c = all[cur].mid();
        // its own copper under it
        if let Some(w) = all
            .iter()
            .find(|w| w.kind.copper() && w.net.is_some() && w.net == all[cur].net && w.lo <= c && c <= w.hi)
        {
            return (chain, w.clone(), true);
        }
        let outer = if sign > 0.0 { all[cur].hi } else { all[cur].lo };
        let gap = |x: &Thing| if sign > 0.0 { x.lo - outer } else { outer - x.hi };
        let next = all
            .iter()
            .enumerate()
            .filter(|(j, x)| {
                *j != cur
                    && *j != from
                    && !chain.contains(j)
                    && !skip.contains(j)
                    && sign * (x.mid() - c) > 1e-9
                    // a mask opening is no wall for the copper that lies in it
                    && !(x.kind == Kind::Keepout && x.net.is_some() && x.net == all[cur].net)
            })
            .min_by(|a, b| gap(a.1).total_cmp(&gap(b.1)));
        match next {
            Some((j, x)) if gap(x) <= OPEN => {
                if x.kind.wall() {
                    return (chain, x.clone(), false);
                }
                chain.push(j);
                cur = j;
            }
            _ => return (chain, open(outer + sign * OPEN), false),
        }
    }
}

/// The cut on one layer.
#[allow(clippy::too_many_arguments)]
fn cut(
    board: &Board,
    local: &Local,
    v: &Violation,
    pieces: usize,
    from: Point,
    to: Point,
    layer: LayerId,
    first: (Kind, usize),
    second: (Kind, Option<usize>),
) -> Passage {
    let d = from.dist(to);
    let n = (to - from).norm();
    let mut all = local.things(from, n, layer, None, None);
    // the two items of the finding among them
    let find = |all: &[Thing], (kind, id): (Kind, Option<usize>), at: f64| {
        all.iter()
            .enumerate()
            .filter(|(_, x)| {
                (x.kind == kind || (x.kind == Kind::Kept && !kind.wall()))
                    && id.is_none_or(|id| x.id == id)
                    && x.lo - 1e-4 <= at
                    && at <= x.hi + 1e-4
            })
            .min_by(|a, b| (a.1.mid() - at).abs().total_cmp(&(b.1.mid() - at).abs()))
            .map(|(i, _)| i)
    };
    let p = find(&all, (first.0, Some(first.1)), 0.0).unwrap_or_else(|| {
        all.push(Thing {
            kind: first.0,
            net: Some(v.nets.0),
            lo: 0.0,
            hi: 0.0,
            obstacle: None,
            grown: 0.0,
            id: first.1,
        });
        all.len() - 1
    });
    let q = find(&all, second, d).unwrap_or_else(|| {
        all.push(Thing {
            kind: second.0,
            net: v.nets.1,
            lo: d,
            hi: d,
            obstacle: second.1.filter(|_| second.0.wall() && second.0 != Kind::Edge),
            grown: 0.0,
            id: second.1.unwrap_or(0),
        });
        all.len() - 1
    });
    let (left, lwall, lown) = if all[p].kind.wall() {
        (Vec::new(), all[p].clone(), false)
    } else {
        walk(&all, p, -1.0, &[q])
    };
    let (right, rwall, rown) = if all[q].kind.wall() {
        (Vec::new(), all[q].clone(), false)
    } else {
        walk(&all, q, 1.0, &[p])
    };
    // in order from wall to wall, the two items marked
    let mut seq: Vec<(Thing, u8)> = vec![(lwall, if all[p].kind.wall() { 1 } else { 0 })];
    seq.extend(left.iter().rev().map(|&i| (all[i].clone(), 0)));
    if !all[p].kind.wall() {
        seq.push((all[p].clone(), 1));
    }
    if !all[q].kind.wall() {
        seq.push((all[q].clone(), 2));
    }
    seq.extend(right.iter().map(|&i| (all[i].clone(), 0)));
    seq.push((rwall, if all[q].kind.wall() { 2 } else { 0 }));
    // copper of one net that overlaps is one thing: a track on its via
    let mut chain: Vec<(Thing, u8)> = Vec::new();
    let last = seq.len() - 1;
    for (i, (x, mark)) in seq.into_iter().enumerate() {
        match chain.last_mut() {
            Some((y, m))
                if i > 1
                    && i < last
                    && !x.kind.wall()
                    && !y.kind.wall()
                    && x.net == y.net
                    && x.lo < y.hi + 1e-6
                    && !(*m != 0 && mark != 0) =>
            {
                y.lo = y.lo.min(x.lo);
                y.hi = y.hi.max(x.hi);
                y.grown = if x.kind == Kind::Via || y.kind == Kind::Via {
                    0.0
                } else {
                    y.grown.max(x.grown)
                };
                if x.kind == Kind::Via {
                    y.kind = Kind::Via;
                    y.id = x.id;
                }
                *m |= mark;
            }
            _ => chain.push((x, mark)),
        }
    }
    let place = |mark: u8| chain.iter().position(|c| c.1 & mark != 0).unwrap_or(0);
    let items = [place(1), place(2)];
    let chain: Vec<Thing> = chain.into_iter().map(|c| c.0).collect();
    let k = chain.len() - 1;
    // a clearance of a custom courtyard rule holds for all of them there
    let clearances: Vec<f64> = chain
        .windows(2)
        .enumerate()
        .map(|(g, w)| {
            if (g == 0 && lown) || (g + 1 == k && rown) {
                return 0.0;
            }
            let c = clearance(board, &w[0], &w[1]);
            board.courtyard_clearance_at(v.at, c).map_or(c, |min| c.min(min))
        })
        .collect();
    Passage {
        what: v.what.clone(),
        nets: v.nets,
        at: v.at,
        missing: v.required - v.actual,
        pieces,
        from,
        dir: n,
        layer,
        chain,
        clearances,
        items,
        own: [lown, rown],
        crossing: false,
    }
}

/// The passages of the clearance findings, one per spot (see the module).
pub fn passages(board: &Board, geo: &Geometry, violations: &[Violation]) -> Vec<Passage> {
    let routing = board.routing_layers();
    let mut out = Vec::new();
    for (v, pieces) in worst(violations) {
        let kind = v.what.split(' ').next().unwrap_or("");
        // the first item: a piece of track with its middle there, or a via
        let seg = geo
            .tracks
            .iter()
            .enumerate()
            .filter(|(_, t)| t.net == v.nets.0)
            .find_map(|(ti, t)| {
                t.pts
                    .windows(2)
                    .find(|w| w[0].lerp(w[1], 0.5).dist(v.at) < 1e-9)
                    .map(|w| (ti, w[0], w[1]))
            });
        let via = geo
            .vias
            .iter()
            .position(|x| x.drill > 0.0 && x.net == v.nets.0 && x.at.dist(v.at) < 1e-9);
        let (line, first, r1, own_layer) = match (seg, via) {
            (Some((ti, a, b)), _) => {
                let t = &geo.tracks[ti];
                (
                    Line::Seg(a, b),
                    (Kind::Track, ti),
                    t.piece_width(a, b) / 2.0,
                    Some(t.layer),
                )
            }
            (None, Some(vi)) => (Line::At(v.at), (Kind::Via, vi), geo.vias[vi].diameter / 2.0, None),
            (None, None) => continue,
        };
        // the second: what the finding names, of the other net, at the distance the check found
        let fits = |pq: (Point, Point), r2: f64| (pq.0.dist(pq.1) - r1 - r2 - v.actual).abs();
        // (how well it fits, nearest points, kind, index, layer)
        let mut best: Option<(f64, (Point, Point), Kind, Option<usize>, Option<LayerId>)> = None;
        let mut offer = |c: (f64, (Point, Point), Kind, Option<usize>, Option<LayerId>)| {
            if best.as_ref().is_none_or(|b| c.0 < b.0) {
                best = Some(c);
            }
        };
        match kind {
            "edge" => {
                for poly in &board.outline {
                    for (c, d) in poly.edges() {
                        let pq = nearest(line, c, d);
                        offer((fits(pq, 0.0), pq, Kind::Edge, None, None));
                    }
                }
            }
            "track" | "via" | "hole_to_hole" if v.nets.1.is_some() => {
                if kind == "track" {
                    for (tj, t) in geo.tracks.iter().enumerate() {
                        if Some(t.net) != v.nets.1 || own_layer.is_some_and(|l| l != t.layer) {
                            continue;
                        }
                        for w in t.pts.windows(2) {
                            let pq = nearest(line, w[0], w[1]);
                            let r2 = t.piece_width(w[0], w[1]) / 2.0;
                            offer((fits(pq, r2), pq, Kind::Track, Some(tj), Some(t.layer)));
                        }
                    }
                } else {
                    for (vj, x) in geo.vias.iter().enumerate() {
                        if Some(x.net) != v.nets.1 || x.drill <= 0.0 || (first == (Kind::Via, vj)) {
                            continue;
                        }
                        let pq = match line {
                            Line::Seg(a, b) => (on_seg(x.at, a, b), x.at),
                            Line::At(p) => (p, x.at),
                        };
                        let fit = if kind == "via" {
                            fits(pq, x.diameter / 2.0)
                        } else {
                            pq.0.dist(pq.1)
                        };
                        offer((fit, pq, Kind::Via, Some(vj), None));
                    }
                }
                // a hole of the board against the drill of a via
                if kind == "hole_to_hole" {
                    for (oi, ob) in board.obstacles.iter().enumerate() {
                        if ob.hole.is_none() || ob.net != v.nets.1 {
                            continue;
                        }
                        let c = ob.shape.bbox().center();
                        offer((v.at.dist(c), (v.at, c), Kind::Hole, Some(oi), None));
                    }
                }
            }
            _ => {
                let me = match line {
                    Line::Seg(a, b) => Shape::Capsule { a, b, r: r1 },
                    Line::At(c) => Shape::Circle { c, r: r1 },
                };
                for (oi, ob) in board.obstacles.iter().enumerate() {
                    if ob.net != v.nets.1 || (ob.shape.bbox().center() - v.at).len() > REACH {
                        continue;
                    }
                    let fit = (ob.shape.dist(&me) - v.actual).abs();
                    if fit > 1e-6 {
                        continue;
                    }
                    let k = match ob.kind {
                        ObstacleKind::Pad => Kind::Pad,
                        ObstacleKind::Track | ObstacleKind::Via => Kind::Kept,
                        ObstacleKind::Keepout { .. } => Kind::Keepout,
                        ObstacleKind::Zone | ObstacleKind::Pour | ObstacleKind::Graphic => Kind::Zone,
                        ObstacleKind::Hole => Kind::Hole,
                    };
                    let on = ob
                        .layers
                        .iter()
                        .find(|&l| routing.has(l) && own_layer.is_none_or(|o| o == l));
                    offer((fit, nearest_shape(line, &ob.shape), k, Some(oi), on));
                }
            }
        }
        let Some((_, (from, to), k2, id2, layer2)) = best else {
            continue;
        };
        let layers: Vec<LayerId> = match own_layer.or(layer2) {
            Some(l) => vec![l],
            None => routing.iter().collect(),
        };
        if from.dist(to) < 1e-9 || layers.is_empty() {
            out.push(Passage {
                what: v.what.clone(),
                nets: v.nets,
                at: v.at,
                missing: v.required - v.actual,
                pieces,
                from,
                dir: Point::default(),
                layer: layers.first().copied().unwrap_or(0),
                chain: vec![open(0.0), open(0.0)],
                clearances: vec![0.0],
                items: [0, 1],
                own: [false, false],
                crossing: true,
            });
            continue;
        }
        // a via against a via or a board item: the layer with the least room
        let local = Local::around(board, geo, from, REACH);
        let p = layers
            .into_iter()
            .map(|l| cut(board, &local, v, pieces, from, to, l, first, (k2, id2)))
            .min_by(|a, b| a.room(0.0).total_cmp(&b.room(0.0)))
            .unwrap();
        out.push(p);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::pt;
    use crate::realize::{Elem, Track};

    fn track(a: Point, b: Point, net: NetId, width: f64) -> Track {
        Track {
            pts: vec![a, b],
            elems: vec![Elem::Line(a, b)],
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

    /// Two pads of nets X and Y, 0.4 mm wide, `apart` between their faces, and two tracks of
    /// nets A and B between them, `gap` apart.
    fn between_pads(apart: f64, gap: f64) -> (Board, Geometry, f64, f64) {
        let b = crate::route::tests::pad_board(&[
            ("U", "X", 5.0, 5.0),
            ("U", "Y", 5.4 + apart, 5.0),
            ("J", "A", 2.0, 2.0),
            ("J", "B", 3.0, 2.0),
        ]);
        let (a, bn) = (2, 3);
        let w = b.class_of(a).track_width;
        let c = b.track_clearance(a, bn);
        let mid = 5.2 + apart / 2.0;
        let x1 = mid - (gap + w) / 2.0;
        let x2 = mid + (gap + w) / 2.0;
        let geo = Geometry {
            tracks: vec![
                track(pt(x1, 4.0), pt(x1, 6.0), a, w),
                track(pt(x2, 4.0), pt(x2, 6.0), bn, w),
            ],
            ..Default::default()
        };
        (b, geo, w, c)
    }

    #[test]
    fn two_tracks_too_near_between_pads_that_leave_room_for_both() {
        let probe = between_pads(3.0, 0.1);
        let (w, c) = (probe.2, probe.3);
        // the pads ask 0.2 of a track, the tracks c of each other
        let need = 2.0 * w + c + 2.0 * 0.2f64.max(c);
        let (b, geo, ..) = between_pads(need + 0.05, c - 0.02);
        let found = crate::check::check(&b, &geo);
        assert!(found.iter().any(|v| v.what.starts_with("track ")), "{found:?}");
        let all = passages(&b, &geo, &found);
        let p = all
            .iter()
            .find(|p| p.kind() == "track")
            .expect("the spot of the two tracks");
        assert_eq!([p.walls()[0].kind, p.walls()[1].kind], [Kind::Pad, Kind::Pad]);
        assert_eq!((p.tracks(), p.vias(), p.gaps()), (2, 0, 3));
        assert!((p.missing - 0.02).abs() < 1e-6, "{}", p.missing);
        assert!((p.needed() - need).abs() < 1e-6, "{} {need}", p.needed());
        assert!((p.available() - need - 0.05).abs() < 1e-6, "{}", p.available());
        // 50 µm of room: enough for a margin of 10 µm on each of the three clearances, not for 20
        assert!((p.room(0.0) - 0.05).abs() < 1e-6);
        assert!(p.room(0.01) > 0.0 && p.room(0.02) < 0.0);
        // each track has 35 µm to its pad to spare: one of them moves, with 10 µm of margin both
        assert_eq!(p.movers(0.0, true), Some(1));
        assert_eq!(p.movers(0.01, true), Some(2));
        assert!((p.room_vias_stay(0.0) - p.room(0.0)).abs() < 1e-9);
    }

    #[test]
    fn the_same_tracks_between_pads_that_leave_too_little() {
        let probe = between_pads(3.0, 0.1);
        let (w, c) = (probe.2, probe.3);
        let need = 2.0 * w + c + 2.0 * 0.2f64.max(c);
        let (b, geo, ..) = between_pads(need - 0.03, c - 0.02);
        let found = crate::check::check(&b, &geo);
        let all = passages(&b, &geo, &found);
        // every spot there, whichever pair it names, lies in the same passage
        assert!(!all.is_empty());
        for p in &all {
            assert_eq!(
                [p.walls()[0].kind, p.walls()[1].kind],
                [Kind::Pad, Kind::Pad],
                "{}",
                p.what
            );
            assert!((p.room(0.0) + 0.03).abs() < 1e-6, "{}: {}", p.what, p.room(0.0));
            assert_eq!(p.movers(0.0, true), None, "{}", p.what);
        }
    }

    #[test]
    fn a_track_that_grew_wider_than_the_pads_leave_room_for_counts_with_its_class_width() {
        let probe = between_pads(3.0, 0.1);
        let (w, c) = (probe.2, probe.3);
        let need = 2.0 * w + c + 2.0 * 0.2f64.max(c);
        // room for both at the class width, with 50 µm to spare; one of them is 0.1 wider
        let (b, mut geo, ..) = between_pads(need + 0.05, c);
        let mut zone = crate::geom::Rect::empty();
        zone.add(geo.tracks[1].pts[0].lerp(geo.tracks[1].pts[1], 0.5));
        geo.tracks[1].fitted.push((zone.grow(1e-6), w + 0.1));
        let found = crate::check::check(&b, &geo);
        assert!(!found.is_empty());
        for p in passages(&b, &geo, &found) {
            assert!((p.room(0.0) - 0.05).abs() < 1e-6, "{}: {}", p.what, p.room(0.0));
            assert!((p.grown() - 0.1).abs() < 1e-9 && (p.grown_vias_stay() - 0.1).abs() < 1e-9);
            // no shifting mends it as long as the track is that wide
            assert_eq!(p.movers(0.0, true), None, "{}", p.what);
        }
    }

    #[test]
    fn two_tracks_too_near_in_the_open_have_room() {
        let (b, geo, _, c) = between_pads(5.0, 0.0);
        // the pads are 1.5 mm and more from the tracks: no wall
        let (b, mut geo) = (b, geo);
        let w = geo.tracks[0].width;
        geo.tracks[1].pts = vec![
            pt(geo.tracks[0].pts[0].x + w + c - 0.01, 4.0),
            pt(geo.tracks[0].pts[0].x + w + c - 0.01, 6.0),
        ];
        let found = crate::check::check(&b, &geo);
        let all = passages(&b, &geo, &found);
        let p = all.iter().find(|p| p.kind() == "track").expect("the spot");
        assert_eq!([p.walls()[0].kind, p.walls()[1].kind], [Kind::Open, Kind::Open]);
        assert!((p.missing - 0.01).abs() < 1e-6);
        assert!(p.room(0.0) > 1.9, "{}", p.room(0.0));
        assert_eq!(p.movers(0.002, true), Some(1));
    }

    /// Pads of X and Y, `apart` between their faces, and a via of V (0.6 across) between them,
    /// `off` from the middle towards Y.
    fn via_between_pads(apart: f64, off: f64) -> (Board, Geometry) {
        let b = crate::route::tests::pad_board(&[
            ("U", "X", 5.0, 5.0),
            ("U", "Y", 5.4 + apart, 5.0),
            ("J", "V", 2.0, 2.0),
            ("J", "A", 3.0, 2.0),
        ]);
        let geo = Geometry {
            vias: vec![crate::realize::Via {
                at: pt(5.2 + apart / 2.0 + off, 5.0),
                diameter: 0.6,
                drill: 0.3,
                net: 2,
                layer: None,
                origin: None,
                resting: false,
                half: Point::default(),
            }],
            ..Default::default()
        };
        (b, geo)
    }

    #[test]
    fn a_via_between_two_pads_has_the_room_they_leave_and_goes_back_to_the_middle() {
        // the pads ask 0.2 of the via: 0.05 are left on each side
        let (b, geo) = via_between_pads(0.6 + 0.4 + 0.1, 0.0);
        let need = 0.2f64.max(b.class_of(2).clearance);
        let r = via_room(&b, &geo, 0, 0.0, &[]);
        assert!((r.stays - (0.25 - need)).abs() < 1e-6, "{r:?}");
        assert_eq!(r.home.map(|h| h.1), Some(0.0));
        assert_eq!(r.tight.map(|t| (t.kind, t.between)), Some((Kind::Pad, 0)));
        // with a margin of 60 µm on each clearance there is no place between these pads: the
        // nearest one is beside the row, where the corners of the pads are far enough
        let r = via_room(&b, &geo, 0, 0.06, &[]);
        let (at, far) = r.home.expect("a place beside the row");
        assert!((r.stays + 0.01).abs() < 1e-6 && (far - 0.32).abs() < 1e-9, "{r:?}");
        assert!((at.x - 5.75).abs() < 1e-9 && (at.y - 5.0).abs() > 0.3, "{at:?}");
        // 80 µm off the middle: 30 µm are missing at the pad of Y, and two steps back is room
        let (b, geo) = via_between_pads(0.6 + 0.4 + 0.1, 0.08);
        let r = via_room(&b, &geo, 0, 0.0, &[]);
        assert!((r.stays - (0.25 - need - 0.08)).abs() < 1e-6, "{r:?}");
        assert_eq!(r.tight.map(|t| (t.kind, t.net)), Some((Kind::Pad, Some(1))));
        let (at, far) = r.home.expect("a place nearer to the middle");
        assert!((far - 0.04).abs() < 1e-9 && at.x < geo.vias[0].at.x, "{at:?} {far}");
    }

    #[test]
    fn a_track_between_a_via_and_a_pad_needs_its_width_and_two_clearances_there() {
        // room for the via alone, 0.05 on each side; a track of A passes between it and Y
        let (b, mut geo) = via_between_pads(0.6 + 0.4 + 0.1, 0.0);
        let x = geo.vias[0].at.x + 0.3 + 0.125;
        geo.tracks.push(track(pt(x, 2.0), pt(x, 8.0), 3, 0.2));
        let (cv, cp) = (
            b.class_of(2).clearance.max(b.class_of(3).clearance),
            0.2f64.max(b.class_of(3).clearance),
        );
        let r = via_room(&b, &geo, 0, 0.0, &[]);
        assert!((r.stays - (0.25 - cv - 0.2 - cp)).abs() < 1e-6, "{r:?}");
        assert_eq!(
            r.tight.map(|t| (t.kind, t.net, t.between)),
            Some((Kind::Pad, Some(1), 1))
        );
        // nowhere near is room for both: the pad of X is 0.25 away on the other side
        assert!(r.home.is_none(), "{r:?}");
        // the passage of the finding says the same wall to wall
        let found = crate::check::check(&b, &geo);
        let all = passages(&b, &geo, &found);
        assert!(all.iter().any(|p| p.via_items() == [0]), "{all:?}");
        assert!(all.iter().all(|p| p.room(0.0) < 0.0), "{all:?}");
    }

    #[test]
    fn a_track_off_the_middle_of_its_pad_beside_the_next_pad_has_the_room_of_the_row() {
        // pads 0.4 wide, 0.25 apart; the track of X is 0.2 wide and asks 0.2 of the pad of Y
        let b = crate::route::tests::pad_board(&[("U", "X", 5.0, 5.0), ("U", "Y", 5.65, 5.0)]);
        let mut t = track(pt(5.18, 5.0), pt(5.18, 6.0), 0, 0.2);
        t.pts = vec![pt(5.18, 5.0), pt(5.18, 5.15), pt(5.18, 6.0)];
        let geo = Geometry {
            tracks: vec![t],
            ..Default::default()
        };
        let found = crate::check::check(&b, &geo);
        let all = passages(&b, &geo, &found);
        let p = all
            .iter()
            .find(|p| p.kind() == "pad")
            .expect("track against the next pad");
        // from its own pad to the next: 0.25, and 0.1 of slack on the pad; the clearance is 0.2
        assert_eq!([p.walls()[0].kind, p.walls()[1].kind], [Kind::Pad, Kind::Pad]);
        assert_eq!(p.walls()[0].net, Some(0));
        assert!((p.available() - 0.25).abs() < 1e-6, "{}", p.available());
        assert!((p.own_slack() - 0.1).abs() < 1e-6, "{}", p.own_slack());
        assert!((p.needed() - 0.2).abs() < 1e-6, "{}", p.needed());
        assert!((p.room(0.0) - 0.15).abs() < 1e-6, "{}", p.room(0.0));
        // the track goes back towards the middle of its pad
        assert_eq!(p.movers(0.002, true), Some(1));
    }
}
