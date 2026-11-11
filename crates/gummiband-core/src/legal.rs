//! The last word on a routed result, on the finished copper and without the router's own books:
//! no short between two nets is written, and a net counts as routed only if its copper really
//! hangs together.
//!
//! The search keeps the topology legal and the geometry step keeps the clearances, but both are
//! large, and a flaw in either used to end as a short in the written board or as a net that was
//! reported routed with a piece of it hanging in the air. So the pipeline ends here: a track that
//! still crosses copper of another net is taken out (the connection is open, which the user sees
//! and can route again, where a short is a broken board), and the connections of every net are
//! counted on the copper itself. Then the stubs go: a track end of the route that lies on
//! nothing of its net is cut back to where the track is last held, and the copper is counted
//! once more.

use crate::RunStats;
use crate::board::{Board, NetId, ObstacleKind};
use crate::geom::{Point, point_seg_dist, seg_seg_dist};
use crate::realize::{Elem, Geometry, Track, Via};

/// What [`enforce`] found and did.
#[derive(Debug, Clone, Default)]
pub struct Legal {
    /// Lines split where a track end met them in their middle.
    pub split: usize,
    /// Tracks taken out because they shorted another net: (net, the other net).
    pub dropped: Vec<(String, String)>,
    /// How many of them ended on a track of their own net (a tap). A tap can come to lie across
    /// a track that was laid between it and the track it ends on.
    pub dropped_taps: usize,
    /// Connections that are open in the copper although the search counted them as routed, as
    /// `"NET from PIN"` (the pin is one of the piece that hangs apart).
    pub open: Vec<String>,
    /// Shorts that could not be removed (no track of the search is part of them).
    pub shorts_left: usize,
    /// Stubs taken back: tracks of the route cut back from an end that lay on nothing of their
    /// net to where they are last held, or taken out as a whole.
    pub trimmed: usize,
    /// Track ends that still lie on nothing of their net, as `"NET at X Y"`: copper of the file,
    /// or a stub that could not go.
    pub dangling: Vec<String>,
}

/// Slack for "this point lies on that copper", in mm.
const ON: f64 = 1e-3;
/// Two pads this near are one piece of copper, in mm: the router decides on polygons drawn
/// around round shapes, which reach a few micrometres further than the shapes themselves.
const TOUCH: f64 = 0.01;

/// One net's copper as a graph: which of its pins hang together.
struct NetCopper<'a> {
    board: &'a Board,
    net: NetId,
    pins: Vec<usize>,
    tracks: Vec<&'a Track>,
    vias: Vec<&'a Via>,
    /// Kept tracks and vias of the board.
    kept: Vec<usize>,
    parent: Vec<usize>,
}

impl<'a> NetCopper<'a> {
    fn new(board: &'a Board, geo: &'a Geometry, net: NetId) -> NetCopper<'a> {
        let pins = board.nets[net as usize].pins.clone();
        let tracks: Vec<&Track> = geo.tracks.iter().filter(|t| t.net == net && t.pts.len() >= 2).collect();
        let vias: Vec<&Via> = geo.vias.iter().filter(|v| v.net == net).collect();
        let kept: Vec<usize> = (0..board.obstacles.len())
            .filter(|&o| {
                let ob = &board.obstacles[o];
                ob.net == Some(net) && matches!(ob.kind, ObstacleKind::Track | ObstacleKind::Via | ObstacleKind::Pour)
            })
            .collect();
        let n = pins.len() + tracks.len() + vias.len() + kept.len() + 1;
        let mut c = NetCopper {
            board,
            net,
            pins,
            tracks,
            vias,
            kept,
            parent: (0..n).collect(),
        };
        c.connect();
        c
    }

    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]];
            x = self.parent[x];
        }
        x
    }

    fn join(&mut self, a: usize, b: usize) {
        let (a, b) = (self.find(a), self.find(b));
        self.parent[a] = b;
    }

    fn track_node(&self, i: usize) -> usize {
        self.pins.len() + i
    }
    fn via_node(&self, i: usize) -> usize {
        self.pins.len() + self.tracks.len() + i
    }
    fn kept_node(&self, i: usize) -> usize {
        self.pins.len() + self.tracks.len() + self.vias.len() + i
    }
    fn plane_node(&self) -> usize {
        self.parent.len() - 1
    }

    /// Does p on `layer` (None: any layer of the pad) lie on the pad of pin `p`?
    fn on_pad(&self, pin: usize, p: Point, layer: Option<u8>) -> bool {
        let pn = &self.board.pins[pin];
        if layer.is_some_and(|l| !pn.layers.has(l)) {
            return false;
        }
        pn.obstacles.iter().any(|&o| {
            let ob = &self.board.obstacles[o];
            layer.is_none_or(|l| ob.layers.has(l)) && ob.shape.dist_to_point(p) <= ON
        })
    }

    /// Do the pads of two pins touch on a layer they share? Pins of one part often do: the
    /// small pads of a power package lie in its large one, and the router takes such pins for
    /// one (`Router::pin_groups`).
    fn pads_touch(&self, p: usize, q: usize) -> bool {
        let b = self.board;
        b.pins[p].obstacles.iter().any(|&o| {
            let a = &b.obstacles[o];
            let near = a.shape.bbox().grow(TOUCH);
            b.pins[q].obstacles.iter().any(|&o2| {
                let c = &b.obstacles[o2];
                a.layers.intersects(c.layers) && near.overlaps(&c.shape.bbox()) && a.shape.dist(&c.shape) <= TOUCH
            })
        })
    }

    fn in_plane(&self, p: Point) -> bool {
        self.board.planes.iter().any(|pl| pl.net == self.net && pl.covers(p))
    }

    fn connect(&mut self) {
        let b = self.board;
        // pins: overlapping pads, and through hole pins in a plane of the net
        for i in 0..self.pins.len() {
            let pi = &b.pins[self.pins[i]];
            for j in 0..i {
                let pj = &b.pins[self.pins[j]];
                if pi.layers.0 & pj.layers.0 != 0 && self.pads_touch(self.pins[i], self.pins[j]) {
                    self.join(i, j);
                }
            }
            let net = self.net;
            if b.planes
                .iter()
                .any(|pl| pl.net == net && pi.layers.has(pl.layer) && pl.covers(pi.pos))
            {
                let pl = self.plane_node();
                self.join(i, pl);
            }
        }
        // track ends: on a pad, a via, kept copper, or another track of the layer
        for i in 0..self.tracks.len() {
            let t = self.tracks[i];
            let node = self.track_node(i);
            for end in [t.pts[0], *t.pts.last().unwrap()] {
                for j in 0..self.pins.len() {
                    if self.on_pad(self.pins[j], end, Some(t.layer)) {
                        self.join(node, j);
                    }
                }
                for j in 0..self.vias.len() {
                    let v = self.vias[j];
                    if v.layer.is_none_or(|l| l == t.layer) && on_via(v, end) {
                        let vn = self.via_node(j);
                        self.join(node, vn);
                    }
                }
                for j in 0..self.kept.len() {
                    let ob = &b.obstacles[self.kept[j]];
                    if ob.layers.has(t.layer) && ob.shape.dist_to_point(end) <= ON {
                        let kn = self.kept_node(j);
                        self.join(node, kn);
                    }
                }
                for j in 0..self.tracks.len() {
                    let u = self.tracks[j];
                    if j != i && u.layer == t.layer && on_track(u, end) {
                        let un = self.track_node(j);
                        self.join(node, un);
                    }
                }
            }
        }
        // vias: on a pad (the via's copper overlaps the pad, as KiCad joins them), in a plane, on
        // kept copper, on the middle of a track, on each other
        for i in 0..self.vias.len() {
            let v = self.vias[i];
            let node = self.via_node(i);
            let r = if v.is_pair() { 0.0 } else { v.diameter / 2.0 };
            for j in 0..self.pins.len() {
                let pn = &b.pins[self.pins[j]];
                let overlaps = pn.obstacles.iter().any(|&o| {
                    let ob = &b.obstacles[o];
                    v.layer.is_none_or(|l| ob.layers.has(l)) && ob.shape.dist_to_point(v.at) <= r + ON
                });
                if overlaps || self.on_pad(self.pins[j], v.at, v.layer) {
                    self.join(node, j);
                }
            }
            for j in 0..self.tracks.len() {
                let u = self.tracks[j];
                if v.layer.is_none_or(|l| l == u.layer) && u.pts.len() >= 2 && on_track(u, v.at) {
                    let un = self.track_node(j);
                    self.join(node, un);
                }
            }
            if v.drill > 0.0 && self.in_plane(v.at) {
                let pl = self.plane_node();
                self.join(node, pl);
            }
            for j in 0..self.kept.len() {
                let ob = &b.obstacles[self.kept[j]];
                if v.layer.is_none_or(|l| ob.layers.has(l)) && ob.shape.dist_to_point(v.at) <= ON {
                    let kn = self.kept_node(j);
                    self.join(node, kn);
                }
            }
            for j in 0..i {
                let w = self.vias[j];
                if on_via(w, v.at) || on_via(v, w.at) {
                    let wn = self.via_node(j);
                    self.join(node, wn);
                }
            }
        }
        // kept copper: on a pad, in a plane, on each other
        for i in 0..self.kept.len() {
            let ob = &b.obstacles[self.kept[i]];
            let node = self.kept_node(i);
            let ends: Vec<Point> = match &ob.shape {
                crate::geom::Shape::Circle { c, .. } => vec![*c],
                crate::geom::Shape::Capsule { a, b, .. } => vec![*a, *b],
                crate::geom::Shape::Polygon(_) => Vec::new(),
            };
            // a pour laid out before the routing holds every pad and kept piece it touches
            if ends.is_empty() {
                let near = ob.shape.bbox().grow(TOUCH);
                for j in 0..self.pins.len() {
                    let touches = b.pins[self.pins[j]].obstacles.iter().any(|&o| {
                        let pad = &b.obstacles[o];
                        pad.layers.intersects(ob.layers)
                            && near.overlaps(&pad.shape.bbox())
                            && ob.shape.dist(&pad.shape) <= TOUCH
                    });
                    if touches {
                        self.join(node, j);
                    }
                }
                for j in 0..self.kept.len() {
                    let other = &b.obstacles[self.kept[j]];
                    if j != i
                        && other.layers.intersects(ob.layers)
                        && near.overlaps(&other.shape.bbox())
                        && ob.shape.dist(&other.shape) <= TOUCH
                    {
                        let kn = self.kept_node(j);
                        self.join(node, kn);
                    }
                }
            }
            for &end in &ends {
                for j in 0..self.pins.len() {
                    let pn = &b.pins[self.pins[j]];
                    if pn.layers.0 & ob.layers.0 != 0 && self.on_pad(self.pins[j], end, None) {
                        self.join(node, j);
                    }
                }
                if ob.kind == ObstacleKind::Via && self.in_plane(end) {
                    let pl = self.plane_node();
                    self.join(node, pl);
                }
                for j in 0..self.kept.len() {
                    let other = &b.obstacles[self.kept[j]];
                    if j != i && other.layers.0 & ob.layers.0 != 0 && other.shape.dist_to_point(end) <= ON {
                        let kn = self.kept_node(j);
                        self.join(node, kn);
                    }
                }
            }
        }
    }

    /// The pieces of the net that hold a pin, each by its first pin; the piece of the plane (or
    /// of the net's first pin) comes first.
    fn pieces(&mut self) -> Vec<usize> {
        let has_plane = self.board.planes.iter().any(|pl| pl.net == self.net);
        let mut roots: Vec<(usize, usize)> = Vec::new();
        if has_plane {
            let r = self.find(self.plane_node());
            roots.push((r, usize::MAX));
        }
        for i in 0..self.pins.len() {
            let r = self.find(i);
            if !roots.iter().any(|x| x.0 == r) {
                roots.push((r, self.pins[i]));
            }
        }
        roots.into_iter().map(|x| x.1).collect()
    }
}

fn on_via(v: &Via, p: Point) -> bool {
    if v.is_pair() {
        point_seg_dist(p, v.at - v.half, v.at + v.half) <= v.diameter / 2.0 + ON
    } else {
        // a branch point of soft tracks has no copper of its own: the tracks meet in it
        p.dist(v.at) <= (v.diameter / 2.0).max(0.01) + ON
    }
}

fn on_track(t: &Track, p: Point) -> bool {
    t.pts
        .windows(2)
        .any(|w| point_seg_dist(p, w[0], w[1]) <= t.piece_width(w[0], w[1]) / 2.0 + ON)
}

/// The nets whose copper falls into more than one piece: per net the pins that name the pieces
/// apart from the first one (a pin of each). A net without a plane and with fewer than two pins
/// has nothing to connect.
pub fn open_in_copper(board: &Board, geo: &Geometry, nets: &[NetId]) -> Vec<(NetId, Vec<usize>)> {
    let mut out = Vec::new();
    for &net in nets {
        let has_plane = board.planes.iter().any(|pl| pl.net == net);
        if board.nets[net as usize].pins.len() < 2 && !has_plane {
            continue;
        }
        let pieces = NetCopper::new(board, geo, net).pieces();
        if pieces.len() > 1 {
            out.push((net, pieces[1..].to_vec()));
        }
    }
    out
}

/// Does the end `p` of track `skip` of `net` lie on something of its net on `layer`: a pad, a
/// via, kept copper, another track, a plane?
fn held(board: &Board, geo: &Geometry, net: NetId, skip: usize, p: Point, layer: u8) -> bool {
    held_fixed(board, net, p, layer)
        || geo
            .vias
            .iter()
            .any(|v| v.net == net && v.layer.is_none_or(|l| l == layer) && on_via(v, p))
        || geo
            .tracks
            .iter()
            .enumerate()
            .any(|(j, u)| j != skip && u.net == net && u.layer == layer && u.pts.len() >= 2 && on_track(u, p))
}

/// The part of [`held`] that the board answers alone: a pad, kept copper or a plane of `net`
/// at `p` on `layer`.
fn held_fixed(board: &Board, net: NetId, p: Point, layer: u8) -> bool {
    board.nets[net as usize].pins.iter().any(|&pin| {
        let pn = &board.pins[pin];
        pn.layers.has(layer)
            && pn
                .obstacles
                .iter()
                .any(|&o| board.obstacles[o].layers.has(layer) && board.obstacles[o].shape.dist_to_point(p) <= ON)
    }) || board.obstacles.iter().any(|ob| {
        ob.net == Some(net)
            && matches!(ob.kind, ObstacleKind::Track | ObstacleKind::Via | ObstacleKind::Pour)
            && ob.layers.has(layer)
            && ob.shape.dist_to_point(p) <= ON
    }) || board
        .planes
        .iter()
        .any(|pl| pl.net == net && pl.layer == layer && pl.covers(p))
}

/// The track ends of the result that lie on nothing of their net, as `"NET at X Y"`. Such an
/// end connects nothing. [`trim_stubs`] takes the tracks of the route back from them; what this
/// names afterwards is copper of the file, or a stub that could not go.
fn dangling(board: &Board, geo: &Geometry) -> Vec<String> {
    let mut out = Vec::new();
    for (i, t) in geo.tracks.iter().enumerate() {
        if t.pts.len() < 2 {
            continue;
        }
        for p in [t.pts[0], *t.pts.last().unwrap()] {
            if !held(board, geo, t.net, i, p, t.layer) {
                out.push(format!("{} at {:.3} {:.3}", board.nets[t.net as usize].name, p.x, p.y));
            }
        }
    }
    out
}

/// Take out the tracks of `net` that end in the air, and the vias that then hold nothing, until
/// what is left hangs on something at both ends. Soft copper that rests is never touched.
fn prune(board: &Board, geo: &mut Geometry, net: NetId) {
    loop {
        let before = (geo.tracks.len(), geo.vias.len());
        let held = |geo: &Geometry, skip: usize, p: Point, layer: u8| held(board, geo, net, skip, p, layer);
        let loose: Vec<usize> = (0..geo.tracks.len())
            .filter(|&i| {
                let t = &geo.tracks[i];
                t.net == net
                    && t.origin.is_none()
                    && t.pts.len() >= 2
                    && (!held(geo, i, t.pts[0], t.layer) || !held(geo, i, *t.pts.last().unwrap(), t.layer))
            })
            .collect();
        for &i in loose.iter().rev() {
            geo.tracks.remove(i);
        }
        drop_idle_vias(board, geo, net);
        if (geo.tracks.len(), geo.vias.len()) == before {
            return;
        }
    }
}

/// Take out the vias of `net` that the route made and that join nothing any more.
fn drop_idle_vias(board: &Board, geo: &mut Geometry, net: NetId) {
    let tracks = &geo.tracks;
    geo.vias.retain(|v| {
        if v.net != net || v.origin.is_some() || v.drill <= 0.0 {
            return true;
        }
        // the layers on which the via joins something: a track end, a pad its copper overlaps
        // (as KiCad joins them), a plane. A via must join on two layers, or KiCad calls it
        // dangling ("connected on only one layer"), and two track ends on one layer meet anyway.
        let mut layers: u64 = 0;
        for t in tracks.iter().filter(|t| t.net == net && t.pts.len() >= 2) {
            if on_via(v, t.pts[0]) || on_via(v, *t.pts.last().unwrap()) {
                layers |= 1 << t.layer;
            }
        }
        let r = if v.is_pair() { 0.0 } else { v.diameter / 2.0 };
        for &pin in &board.nets[net as usize].pins {
            for &o in &board.pins[pin].obstacles {
                let ob = &board.obstacles[o];
                if ob.shape.dist_to_point(v.at) <= r + ON {
                    for l in 0..64u8 {
                        if ob.layers.has(l) {
                            layers |= 1 << l;
                        }
                    }
                }
            }
        }
        for pl in board.planes.iter().filter(|pl| pl.net == net && pl.covers(v.at)) {
            layers |= 1 << pl.layer;
        }
        // kept copper of the net under the via, and a track the via sits on
        for ob in board.obstacles.iter().filter(|ob| {
            ob.net == Some(net) && matches!(ob.kind, ObstacleKind::Track | ObstacleKind::Via | ObstacleKind::Pour)
        }) {
            if ob.shape.dist_to_point(v.at) <= r + ON {
                for l in (0..64u8).filter(|&l| ob.layers.has(l)) {
                    layers |= 1 << l;
                }
            }
        }
        for t in tracks
            .iter()
            .filter(|t| t.net == net && t.pts.len() >= 2 && on_track(t, v.at))
        {
            layers |= 1 << t.layer;
        }
        layers.count_ones() >= 2
    });
}

/// Turn a track round: its last point becomes its first.
fn reverse(t: &mut Track) {
    t.pts.reverse();
    t.elems.reverse();
    for e in &mut t.elems {
        *e = match *e {
            Elem::Line(a, b) => Elem::Line(b, a),
            Elem::Arc(a, mid, b) => Elem::Arc(b, mid, a),
        };
    }
}

/// Where track `i`, whose last point lies on nothing, is last held on its way there: the index
/// of the piece and the points that end the track in place of the rest of it (the foot on the
/// piece, and the point of what holds it if that lies beside the centre line). `None`: nothing
/// holds it after its first point, the whole track is the stub.
fn last_junction(board: &Board, geo: &Geometry, i: usize) -> Option<(usize, Vec<Point>)> {
    let t = &geo.tracks[i];
    let foot = |p: Point, a: Point, b: Point| {
        let d = b - a;
        let l2 = d.dot(d);
        if l2 <= 0.0 {
            0.0
        } else {
            ((p - a).dot(d) / l2).clamp(0.0, 1.0)
        }
    };
    for k in (0..t.pts.len() - 1).rev() {
        let (a, b) = (t.pts[k], t.pts[k + 1]);
        let reach = t.piece_width(a, b) / 2.0 + ON;
        // what ends on this piece: the ends of other tracks of the net, and its vias
        let mut on_piece: Vec<Point> = geo
            .tracks
            .iter()
            .enumerate()
            .filter(|(j, u)| *j != i && u.net == t.net && u.layer == t.layer && u.pts.len() >= 2)
            .flat_map(|(_, u)| [u.pts[0], *u.pts.last().unwrap()])
            .filter(|&e| point_seg_dist(e, a, b) <= reach)
            .collect();
        on_piece.extend(
            geo.vias
                .iter()
                .filter(|v| v.net == t.net && !v.is_pair() && v.layer.is_none_or(|l| l == t.layer))
                .filter(|v| point_seg_dist(v.at, a, b) <= (v.diameter / 2.0).max(0.01) + ON)
                .map(|v| v.at),
        );
        // the one nearest to the loose end
        let best = on_piece
            .into_iter()
            .map(|e| (foot(e, a, b), e))
            .max_by(|x, y| x.0.total_cmp(&y.0));
        if let Some((s, e)) = best {
            let at = a.lerp(b, s);
            let mut tail = vec![at];
            if at.dist(e) > ON {
                tail.push(e);
            }
            return (k > 0 || at.dist(t.pts[0]) > ON).then_some((k, tail));
        }
        // or the corner before it lies on a pad, on kept copper, in a plane, or on a via of a
        // pair
        let pair = geo
            .vias
            .iter()
            .any(|v| v.net == t.net && v.is_pair() && v.layer.is_none_or(|l| l == t.layer) && on_via(v, a));
        if k > 0 && (pair || held_fixed(board, t.net, a, t.layer)) {
            return Some((k - 1, vec![a]));
        }
    }
    None
}

/// Cut track `t` after its piece `k` and end it with `tail` (the first point of `tail` lies on
/// piece `k`, or is the end of piece `k`). Lines and arcs that end before the cut stay, what is
/// left of the one that is cut becomes lines.
fn cut(t: &mut Track, k: usize, tail: &[Point]) {
    let mut pts: Vec<Point> = t.pts[..=k].to_vec();
    for &p in tail {
        if pts.last().is_none_or(|q| q.dist(p) > 1e-9) {
            pts.push(p);
        }
    }
    let mut elems = Vec::new();
    let mut at = 0;
    for e in &t.elems {
        let (from, to) = match *e {
            Elem::Line(a, b) | Elem::Arc(a, _, b) => (a, b),
        };
        let next = (at + 1..=k).find(|&j| t.pts[j].dist(to) < 1e-9);
        match next {
            Some(j) if t.pts[at].dist(from) < 1e-9 => {
                elems.push(*e);
                at = j;
            }
            _ => break,
        }
    }
    elems.extend(pts[at..].windows(2).map(|w| Elem::Line(w[0], w[1])));
    t.pts = pts;
    t.elems = elems;
}

/// Take the stubs back: a track of the route whose end lies on nothing of its net is cut back
/// to where it is last held, or goes as a whole if nothing holds it on the way; a via that then
/// joins nothing goes too, and the track that ended on it is the next stub. Such an end connects
/// nothing, so no connection can go with it; a net that would fall into more pieces all the same
/// keeps its copper as it was. Copper of the file is never cut. Returns the number of stubs
/// taken back.
fn trim_stubs(board: &Board, geo: &mut Geometry) -> usize {
    // a track of the route with a loose end, and whether that is its last point
    let loose = |geo: &Geometry, net: NetId| {
        geo.tracks.iter().enumerate().find_map(|(i, t)| {
            if t.net != net || t.origin.is_some() || t.pts.len() < 2 {
                return None;
            }
            let first = !held(board, geo, t.net, i, t.pts[0], t.layer);
            let last = !held(board, geo, t.net, i, *t.pts.last().unwrap(), t.layer);
            (first || last).then_some((i, last))
        })
    };
    let mut nets: Vec<NetId> = geo
        .tracks
        .iter()
        .enumerate()
        .filter(|(i, t)| {
            t.origin.is_none()
                && t.pts.len() >= 2
                && (!held(board, geo, t.net, *i, t.pts[0], t.layer)
                    || !held(board, geo, t.net, *i, *t.pts.last().unwrap(), t.layer))
        })
        .map(|(_, t)| t.net)
        .collect();
    nets.sort_unstable();
    nets.dedup();
    let mut trimmed = 0;
    for net in nets {
        let before = geo.clone();
        let pieces = NetCopper::new(board, geo, net).pieces().len();
        let mut count = 0;
        // every round ends a track at something that holds it, or takes one out
        for _ in 0..2 * geo.tracks.len() + 8 {
            let Some((i, at_end)) = loose(geo, net) else {
                break;
            };
            if !at_end {
                reverse(&mut geo.tracks[i]);
            }
            match last_junction(board, geo, i) {
                Some((k, tail)) => {
                    let t = &mut geo.tracks[i];
                    cut(t, k, &tail);
                    if at_end {
                        t.tap = None;
                    } else {
                        reverse(t);
                    }
                }
                None => {
                    geo.tracks.remove(i);
                }
            }
            drop_idle_vias(board, geo, net);
            count += 1;
        }
        if NetCopper::new(board, geo, net).pieces().len() > pieces || loose(geo, net).is_some() {
            *geo = before;
        } else {
            trimmed += count;
        }
    }
    trimmed
}

/// Does the copper of track `t` touch copper of net `other` (a track or via of the result, a
/// pad or kept copper of the board) on its layer?
fn touches(board: &Board, geo: &Geometry, t: &Track, other: NetId) -> bool {
    t.pts.windows(2).any(|w| {
        let r = t.piece_width(w[0], w[1]) / 2.0;
        geo.tracks.iter().any(|u| {
            u.net == other
                && u.layer == t.layer
                && u.pts
                    .windows(2)
                    .any(|x| seg_seg_dist(w[0], w[1], x[0], x[1]) <= r + u.piece_width(x[0], x[1]) / 2.0)
        }) || geo.vias.iter().any(|v| {
            v.net == other
                && v.layer.is_none_or(|l| l == t.layer)
                && point_seg_dist(v.at, w[0], w[1]) <= r + v.diameter / 2.0
        }) || board.obstacles.iter().any(|ob| {
            ob.net == Some(other)
                && ob.layers.has(t.layer)
                && matches!(
                    ob.kind,
                    ObstacleKind::Pad | ObstacleKind::Track | ObstacleKind::Via | ObstacleKind::Pour
                )
                && ob.shape.dist_to_seg(w[0], w[1]) <= r
        })
    })
}

/// The same for a via.
fn via_touches(board: &Board, geo: &Geometry, v: &Via, other: NetId) -> bool {
    let r = v.diameter / 2.0;
    geo.tracks.iter().any(|u| {
        u.net == other
            && v.layer.is_none_or(|l| l == u.layer)
            && u.pts
                .windows(2)
                .any(|x| point_seg_dist(v.at, x[0], x[1]) <= r + u.piece_width(x[0], x[1]) / 2.0)
    }) || geo
        .vias
        .iter()
        .any(|w| w.net == other && w.at.dist(v.at) <= r + w.diameter / 2.0)
        || board.obstacles.iter().any(|ob| {
            ob.net == Some(other)
                && v.layer.is_none_or(|l| ob.layers.has(l))
                && matches!(
                    ob.kind,
                    ObstacleKind::Pad | ObstacleKind::Track | ObstacleKind::Via | ObstacleKind::Pour
                )
                && ob.shape.dist_to_point(v.at) <= r
        })
}

/// A violation that is a short between two nets: nothing is left of the gap.
fn is_short(v: &crate::check::Violation) -> bool {
    v.actual <= 0.0 && !v.is_width() && v.nets.1.is_some_and(|o| o != v.nets.0)
}

/// Take out every track that shorts copper of another net; returns (net, other net) per track
/// and the number of shorts that stayed (no track of the search takes part in them).
fn drop_shorts(board: &Board, geo: &mut Geometry) -> (Vec<(NetId, NetId, bool)>, usize) {
    let mut dropped = Vec::new();
    // every round takes out one track; a board never has more shorts than tracks
    for _ in 0..geo.tracks.len().max(1) {
        let shorts: Vec<crate::check::Violation> =
            crate::check::check(board, geo).into_iter().filter(is_short).collect();
        let Some(v) = shorts.first() else {
            return (dropped, 0);
        };
        let (a, b) = (v.nets.0, v.nets.1.unwrap());
        // the tracks of the two nets that touch copper of the other one, or end on a via that
        // does; the one with the least copper goes
        let len = |t: &Track| t.pts.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>();
        let pick = geo
            .tracks
            .iter()
            .enumerate()
            .filter(|(_, t)| (t.net == a || t.net == b) && t.origin.is_none() && t.pts.len() >= 2)
            .filter(|(_, t)| {
                let other = if t.net == a { b } else { a };
                touches(board, geo, t, other)
                    || geo.vias.iter().any(|w| {
                        w.net == t.net
                            && w.origin.is_none()
                            && (on_via(w, t.pts[0]) || on_via(w, *t.pts.last().unwrap()))
                            && via_touches(board, geo, w, other)
                    })
            })
            .min_by(|x, y| len(x.1).total_cmp(&len(y.1)).then(x.0.cmp(&y.0)))
            .map(|(i, t)| (i, t.net, t.tap.is_some()));
        let Some((i, net, tap)) = pick else {
            return (dropped, shorts.len());
        };
        geo.tracks.remove(i);
        prune(board, geo, net);
        dropped.push((net, if net == a { b } else { a }, tap));
    }
    let left = crate::check::check(board, geo).iter().filter(|v| is_short(v)).count();
    (dropped, left)
}

/// The pairs of nets whose copper touches in `geo` on `board`: the lower net first, each pair
/// once.
pub fn shorts(board: &Board, geo: &Geometry) -> Vec<(NetId, NetId)> {
    let mut pairs: Vec<(NetId, NetId)> = crate::check::check(board, geo)
        .iter()
        .filter(|v| is_short(v))
        .map(|v| {
            let (a, b) = (v.nets.0, v.nets.1.unwrap());
            (a.min(b), a.max(b))
        })
        .collect();
    pairs.sort_unstable();
    pairs.dedup();
    pairs
}

/// The writer's own look at whatever geometry it is handed: `geo` without the tracks of the
/// route that short another net, or `None` if there is none to take out. [`enforce`] has done
/// this for a result and corrected its numbers; this is for a geometry that came another way.
pub fn without_shorts(board: &Board, geo: &Geometry) -> Option<Geometry> {
    if shorts(board, geo).is_empty() {
        return None;
    }
    let mut clean = geo.clone();
    let (dropped, left) = drop_shorts(board, &mut clean);
    if dropped.is_empty() {
        return None;
    }
    let name = |n: NetId| board.nets[n as usize].name.as_str();
    let pairs: Vec<String> = dropped
        .iter()
        .map(|&(a, b, _)| format!("{} x {}", name(a), name(b)))
        .collect();
    eprintln!(
        "last check before writing: {} tracks that short another net are not written [{}], {left} shorts left",
        dropped.len(),
        pairs.join(", ")
    );
    Some(clean)
}

/// The shorts of `after` (on `board_after`) that `before` (on `board_before`) does not have, as
/// `"A x B"`: for a step that changes copper it did not lay and may not take out. Such a step
/// must not write a result that this names.
pub fn new_shorts(board_before: &Board, before: &Geometry, board_after: &Board, after: &Geometry) -> Vec<String> {
    let old = shorts(board_before, before);
    shorts(board_after, after)
        .into_iter()
        .filter(|p| !old.contains(p))
        .map(|(a, b)| {
            format!(
                "{} x {}",
                board_after.nets[a as usize].name, board_after.nets[b as usize].name
            )
        })
        .collect()
}

/// The end of a route: shorts between two nets are taken out of `geo` (the connection is open
/// instead), and every net with copper in `geo` or with a dropped track is counted on the
/// copper: what is not connected there is open in `stats`, whatever the search believed.
pub fn enforce(board: &Board, geo: &mut Geometry, stats: &mut RunStats) -> Legal {
    let mut legal = Legal::default();
    // 1. shorts go, with what hangs on them
    let (dropped, left) = drop_shorts(board, geo);
    legal.shorts_left = left;
    let name = |n: NetId| board.nets[n as usize].name.clone();
    legal.dropped = dropped.iter().map(|&(a, b, _)| (name(a), name(b))).collect();
    legal.dropped_taps = dropped.iter().filter(|d| d.2).count();
    let mut nets: Vec<NetId> = geo
        .tracks
        .iter()
        .map(|t| t.net)
        .chain(geo.vias.iter().map(|v| v.net))
        .chain(dropped.iter().map(|d| d.0))
        .collect();
    nets.sort_unstable();
    nets.dedup();
    // 2. what is connected is counted on the copper
    count_open(board, geo, &nets, &dropped, stats, &mut legal);
    // 3. stubs are taken back: an end on nothing connects nothing
    legal.trimmed = trim_stubs(board, geo);
    // 4. and the copper is counted once more, as the final word
    if legal.trimmed > 0 {
        count_open(board, geo, &nets, &dropped, stats, &mut legal);
    }
    if !legal.dropped.is_empty() || legal.trimmed > 0 {
        stats.vias = geo.vias.iter().filter(|v| v.drill > 0.0).count();
        stats.length = geo
            .tracks
            .iter()
            .map(|t| t.pts.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>())
            .sum();
    }
    // 5. a track end on the middle of a line of its net becomes a corner of that line: KiCad
    // does not always take an end in the middle of a track for a connection
    legal.split = split_at_junctions(geo);
    legal.dangling = dangling(board, geo);
    if !legal.dropped.is_empty()
        || !legal.open.is_empty()
        || legal.shorts_left > 0
        || legal.trimmed > 0
        || !legal.dangling.is_empty()
    {
        let pairs: Vec<String> = legal.dropped.iter().map(|(a, b)| format!("{a} x {b}")).collect();
        eprintln!(
            "last check on the copper: {} shorts taken out ({} of them tracks that end on a track of their net) [{}], {} left; open in the copper: [{}]; track ends on nothing: {} taken back, left [{}]",
            legal.dropped.len(),
            legal.dropped_taps,
            pairs.join(", "),
            legal.shorts_left,
            legal.open.join(", "),
            legal.trimmed,
            legal.dangling.join(", ")
        );
    }
    legal
}

/// Count the connections on the copper: every piece of a net beyond its first that `stats` does
/// not know as open becomes open there and in `legal`. Counting twice adds nothing.
fn count_open(
    board: &Board,
    geo: &Geometry,
    nets: &[NetId],
    dropped: &[(NetId, NetId, bool)],
    stats: &mut RunStats,
    legal: &mut Legal,
) {
    let name = |n: NetId| board.nets[n as usize].name.clone();
    for (net, pins) in open_in_copper(board, geo, nets) {
        let net_name = name(net);
        let prefix = format!("{net_name} from ");
        let known = stats.unrouted.iter().filter(|u| u.starts_with(&prefix)).count();
        for &pin in pins.iter().skip(known) {
            // the plane's piece has no pin of its own: name the net's first pin
            let pin = if pin == usize::MAX {
                board.nets[net as usize].pins.first().copied().unwrap_or(0)
            } else {
                pin
            };
            let open = format!("{net_name} from {}", board.pins[pin].name);
            let with: Vec<String> = dropped.iter().filter(|d| d.0 == net).map(|d| name(d.1)).collect();
            stats.blocked.push((open.clone(), with));
            stats.unrouted.push(open.clone());
            stats.routed = stats.routed.saturating_sub(1);
            legal.open.push(open);
        }
    }
}

/// Split the lines of the route that a track end of their net meets in their middle, so that
/// every junction is one of ends: the foot of the end becomes a corner of the line (in its points
/// and elements), and both halves keep the width the line had. KiCad does not take every end in
/// the middle of a track for a connection (it reported a few of a hundred as dangling), ends that
/// coincide it always does. Arcs stay as they are, and so does copper of the file. Returns the
/// number of splits.
pub fn split_at_junctions(geo: &mut Geometry) -> usize {
    let ends: Vec<(u32, u8, Point)> = geo
        .tracks
        .iter()
        .filter(|t| t.pts.len() >= 2)
        .flat_map(|t| [(t.net, t.layer, t.pts[0]), (t.net, t.layer, *t.pts.last().unwrap())])
        .collect();
    let mut splits = 0;
    for t in geo.tracks.iter_mut().filter(|t| t.origin.is_none()) {
        for &(net, layer, e) in &ends {
            if net != t.net || layer != t.layer {
                continue;
            }
            // the line element the end lies on, away from its ends
            let Some(k) = t.elems.iter().position(|el| match *el {
                Elem::Line(a, b) => {
                    point_seg_dist(e, a, b) <= t.piece_width(a, b) / 2.0 + ON
                        && a.dist(e) > 10.0 * ON
                        && b.dist(e) > 10.0 * ON
                }
                Elem::Arc(..) => false,
            }) else {
                continue;
            };
            let Elem::Line(a, b) = t.elems[k] else { continue };
            let d = b - a;
            let s = ((e - a).dot(d) / d.dot(d)).clamp(0.0, 1.0);
            // the end itself where it lies on the centre line (KiCad joins ends that coincide)
            let foot = a.lerp(b, s);
            let f = if e.dist(foot) <= ON { e } else { foot };
            if a.dist(f) <= 10.0 * ON || b.dist(f) <= 10.0 * ON {
                continue;
            }
            let w = t.piece_width(a, b);
            t.elems.splice(k..=k, [Elem::Line(a, f), Elem::Line(f, b)]);
            // a half that no longer reaches the zone its width came from keeps that width
            for (p, q) in [(a, f), (f, b)] {
                if (t.piece_width(p, q) - w).abs() > 1e-12 {
                    let mut r = crate::geom::Rect::empty();
                    r.add(p.lerp(q, 0.5));
                    t.fitted.push((r.grow(1e-6), w));
                }
            }
            // the points: f goes between the two points of a-b
            if let Some(i) = t
                .pts
                .windows(2)
                .position(|w| w[0].dist(a) < 1e-9 && w[1].dist(b) < 1e-9)
            {
                t.pts.insert(i + 1, f);
            }
            splits += 1;
        }
    }
    splits
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{LayerSet, Net, Obstacle, Pin};
    use crate::geom::{Polygon, Shape, pt};

    /// A one layer board with a 0.4 mm pad per entry (net name, x, y).
    fn board(pins: &[(&str, f64, f64)]) -> Board {
        let mut b = crate::topo::tests::two_pad_board();
        b.outline = vec![Polygon::new(vec![pt(0., 0.), pt(20., 0.), pt(20., 20.), pt(0., 20.)])];
        b.nets.clear();
        b.pins.clear();
        b.obstacles.clear();
        for (i, &(net, x, y)) in pins.iter().enumerate() {
            let n = b.nets.iter().position(|n| n.name == net).unwrap_or_else(|| {
                b.nets.push(Net {
                    name: net.into(),
                    class: 0,
                    pins: Vec::new(),
                    planes: LayerSet::default(),
                    priority: 0,
                });
                b.nets.len() - 1
            });
            b.nets[n].pins.push(i);
            b.obstacles.push(Obstacle {
                shape: Shape::Polygon(crate::geom::roundrect_polygon(pt(x, y), 0.4, 0.4, 0.0, 0.0, 0.005)),
                layers: LayerSet::single(0),
                net: Some(n as NetId),
                kind: ObstacleKind::Pad,
                hole: None,
                clearance: 0.2,
                pin: Some(i),
                locked: true,
            });
            b.pins.push(Pin {
                name: format!("P.{i}"),
                footprint: "P".into(),
                net: n as NetId,
                pos: pt(x, y),
                layers: LayerSet::single(0),
                obstacles: vec![i],
                size: (0.4, 0.4),
            });
        }
        b
    }

    fn track(net: NetId, pts: &[(f64, f64)]) -> Track {
        let pts: Vec<Point> = pts.iter().map(|&(x, y)| pt(x, y)).collect();
        Track {
            elems: pts.windows(2).map(|w| Elem::Line(w[0], w[1])).collect(),
            pts,
            width: 0.2,
            layer: 0,
            net,
            necks: Vec::new(),
            neck_w: 0.2,
            origin: None,
            resting: false,
            fitted: Vec::new(),
            pattern: None,
            seg: u32::MAX,
            tap: None,
        }
    }

    fn stats(conns: usize) -> RunStats {
        RunStats {
            conns,
            routed: conns,
            ..Default::default()
        }
    }

    fn cross() -> Board {
        board(&[("A", 2., 10.), ("A", 18., 10.), ("B", 10., 2.), ("B", 10., 18.)])
    }

    #[test]
    fn a_clean_result_stays_as_it_is() {
        let b = board(&[("A", 2., 10.), ("A", 18., 10.), ("B", 2., 14.), ("B", 18., 14.)]);
        let mut geo = Geometry {
            tracks: vec![track(0, &[(2., 10.), (18., 10.)]), track(1, &[(2., 14.), (18., 14.)])],
            vias: Vec::new(),
        };
        let mut st = stats(2);
        let legal = enforce(&b, &mut geo, &mut st);
        assert!(legal.dropped.is_empty() && legal.open.is_empty() && legal.shorts_left == 0);
        assert_eq!((geo.tracks.len(), st.routed, st.unrouted.len()), (2, 2, 0));
    }

    #[test]
    fn a_track_that_crosses_another_net_is_taken_out_and_its_connection_is_open() {
        let b = cross();
        // the track of B is the shorter one: it goes
        let mut geo = Geometry {
            tracks: vec![
                track(0, &[(2., 10.), (10., 9.), (18., 10.)]),
                track(1, &[(10., 2.), (10., 18.)]),
            ],
            vias: Vec::new(),
        };
        assert!(crate::check::check(&b, &geo).iter().any(|v| v.actual <= 0.0));
        let mut st = stats(2);
        let legal = enforce(&b, &mut geo, &mut st);
        assert_eq!(legal.dropped, vec![("B".to_string(), "A".to_string())]);
        assert_eq!(legal.shorts_left, 0);
        assert!(crate::check::check(&b, &geo).iter().all(|v| v.actual > 0.0));
        assert_eq!(geo.tracks.len(), 1);
        assert_eq!(st.routed, 1);
        assert_eq!(st.unrouted, vec!["B from P.3".to_string()]);
        assert_eq!(legal.open, st.unrouted);
    }

    #[test]
    fn what_hangs_on_a_dropped_track_goes_with_it() {
        let b = cross();
        // B in two tracks that meet in the middle of the board; the first one crosses A
        let mut geo = Geometry {
            tracks: vec![
                track(0, &[(2., 10.), (18., 10.)]),
                track(1, &[(10., 2.), (10., 12.)]),
                track(1, &[(10., 12.), (10., 18.)]),
            ],
            vias: Vec::new(),
        };
        let mut st = stats(2);
        enforce(&b, &mut geo, &mut st);
        assert_eq!(geo.tracks.len(), 1, "the stub to the other pad of B is gone as well");
        assert_eq!(st.unrouted.len(), 1);
    }

    /// A track that ends on a track of its own net (a tap), with a track of another net laid
    /// across it: the search can produce this, because the tap takes no room between itself and
    /// the track it ends on. The last check takes the tap out, says that it was one, and its
    /// connection is open.
    #[test]
    fn a_tap_that_crosses_another_net_is_taken_out_and_named() {
        let b = board(&[
            ("A", 2., 10.),
            ("A", 18., 10.),
            ("A", 10., 14.),
            ("B", 6., 12.),
            ("B", 14., 12.),
        ]);
        let mut tap = track(0, &[(10., 14.), (10., 10.)]);
        tap.tap = Some(0);
        let mut geo = Geometry {
            tracks: vec![
                track(0, &[(2., 10.), (18., 10.)]),
                tap,
                track(1, &[(6., 12.), (14., 12.)]),
            ],
            vias: Vec::new(),
        };
        let mut st = stats(3);
        let legal = enforce(&b, &mut geo, &mut st);
        assert_eq!(legal.dropped, vec![("A".to_string(), "B".to_string())]);
        assert_eq!((legal.dropped_taps, legal.shorts_left), (1, 0));
        assert_eq!(legal.open, vec!["A from P.2".to_string()]);
        assert_eq!((st.routed, geo.tracks.len()), (2, 2));
        assert!(legal.dangling.is_empty());
    }

    /// The writer and the steps on copper they did not lay have their own look: the first gets
    /// the geometry without the track that shorts, the second the name of a short that is new.
    #[test]
    fn a_short_is_seen_before_it_is_written() {
        let b = board(&[("A", 2., 10.), ("A", 18., 10.), ("B", 10., 6.), ("B", 10., 14.)]);
        let clean = Geometry {
            tracks: vec![track(0, &[(2., 10.), (18., 10.)])],
            vias: Vec::new(),
        };
        let mut crossed = clean.clone();
        crossed.tracks.push(track(1, &[(10., 6.), (10., 14.)]));
        assert!(shorts(&b, &clean).is_empty() && without_shorts(&b, &clean).is_none());
        assert_eq!(shorts(&b, &crossed), vec![(0, 1)]);
        assert_eq!(new_shorts(&b, &clean, &b, &crossed), vec!["A x B".to_string()]);
        assert!(new_shorts(&b, &crossed, &b, &crossed).is_empty());
        let written = without_shorts(&b, &crossed).expect("a track has to go");
        assert_eq!(written.tracks.len(), 1);
        assert!(shorts(&b, &written).is_empty());
    }

    /// The pads of a power package lie against each other: such pins are one piece of copper
    /// without any track, as the router sees them too. One track to one of them connects both.
    #[test]
    fn pads_that_touch_are_one_piece_of_copper() {
        // the pads are 0.4 mm wide: P.1 and P.2 touch, their centres lie outside each other
        let b = board(&[("A", 2., 10.), ("A", 10., 10.), ("A", 10.4, 10.)]);
        let mut geo = Geometry {
            tracks: vec![track(0, &[(2., 10.), (10., 10.)])],
            vias: Vec::new(),
        };
        let mut st = stats(1);
        let legal = enforce(&b, &mut geo, &mut st);
        assert!(legal.open.is_empty(), "{:?}", legal.open);
        assert_eq!(st.routed, 1);
        // a gap between them, and the third pin is open
        let b = board(&[("A", 2., 10.), ("A", 10., 10.), ("A", 10.5, 10.)]);
        let mut st = stats(1);
        assert_eq!(enforce(&b, &mut geo, &mut st).open, vec!["A from P.2".to_string()]);
    }

    #[test]
    fn a_net_whose_copper_does_not_meet_counts_as_open() {
        let b = board(&[("A", 2., 10.), ("A", 18., 10.), ("A", 10., 16.)]);
        // the search says 2 of 2, but the second track ends in the air
        let mut geo = Geometry {
            tracks: vec![track(0, &[(2., 10.), (18., 10.)]), track(0, &[(10., 16.), (10., 12.)])],
            vias: Vec::new(),
        };
        let mut st = stats(2);
        let legal = enforce(&b, &mut geo, &mut st);
        assert_eq!(legal.open, vec!["A from P.2".to_string()]);
        assert_eq!((st.routed, st.unrouted.len()), (1, 1));
        // and the track that ends in the air is a stub: it goes, after the count and before
        // the final one, which finds the same
        assert_eq!((legal.trimmed, geo.tracks.len()), (1, 1));
        assert!(legal.dangling.is_empty());
        // a track that ends on the other track of its net is connected
        let mut geo = Geometry {
            tracks: vec![
                track(0, &[(2., 10.), (18., 10.)]),
                track(0, &[(10., 16.), (10., 10.05)]),
            ],
            vias: Vec::new(),
        };
        let mut st = stats(2);
        assert!(enforce(&b, &mut geo, &mut st).open.is_empty());
        assert_eq!(st.routed, 2);
    }

    /// A track runs on past the point where the other track of its net ends on it, and stops in
    /// the air: the tail is a stub. It is cut back to that point, in the middle of a piece, and
    /// both connections stay. A track that leaves a pad and ends on nothing goes as a whole, with
    /// the via it ended on if that joins nothing else; copper of the file is left alone.
    #[test]
    fn a_stub_is_taken_back_to_where_the_track_is_last_held() {
        let b = board(&[("A", 2., 10.), ("A", 10., 16.), ("B", 4., 4.), ("B", 16., 4.)]);
        let mut geo = Geometry {
            tracks: vec![
                // from its pad past the junction at (10, 10) to an end in the air, round a corner
                track(0, &[(2., 10.), (14., 10.), (14., 12.)]),
                track(0, &[(10., 16.), (10., 10.)]),
                // B: connected, and a second track from the pad to a via that holds nothing
                track(1, &[(4., 4.), (16., 4.)]),
                track(1, &[(4., 4.), (4., 7.)]),
            ],
            vias: vec![Via {
                at: pt(4., 7.),
                diameter: 0.6,
                drill: 0.3,
                net: 1,
                layer: None,
                origin: None,
                resting: false,
                half: Point::default(),
            }],
        };
        // the via end is held by the via, so that track is no stub while the via is there: take
        // the track on from the via into the air
        geo.tracks.push(track(1, &[(4., 7.), (6., 7.)]));
        let mut st = stats(2);
        let legal = enforce(&b, &mut geo, &mut st);
        assert!(legal.open.is_empty() && legal.dropped.is_empty(), "{legal:?}");
        assert_eq!(st.routed, 2);
        // A: one cut; B: the track into the air, and with the via gone the track to it
        assert_eq!(legal.trimmed, 3);
        assert!(legal.dangling.is_empty(), "{:?}", legal.dangling);
        assert_eq!(geo.tracks.len(), 3);
        assert_eq!(geo.tracks[0].pts, vec![pt(2., 10.), pt(10., 10.)]);
        assert_eq!(geo.tracks[0].elems, vec![Elem::Line(pt(2., 10.), pt(10., 10.))]);
        assert!(geo.vias.is_empty());
        assert_eq!((st.vias, st.length.round()), (0, 26.0));
        // the start of a track can be the loose end as well, and soft copper keeps its stub
        let mut geo = Geometry {
            tracks: vec![
                track(0, &[(14., 10.), (2., 10.)]),
                track(0, &[(10., 16.), (10., 10.)]),
                Track {
                    origin: Some(0),
                    ..track(1, &[(4., 4.), (4., 7.)])
                },
            ],
            vias: Vec::new(),
        };
        let mut st = stats(1);
        let legal = enforce(&b, &mut geo, &mut st);
        assert_eq!(legal.trimmed, 1);
        assert_eq!(geo.tracks[0].pts, vec![pt(10., 10.), pt(2., 10.)]);
        assert_eq!(legal.dangling, vec!["B at 4.000 7.000".to_string()]);
        assert_eq!(geo.tracks.len(), 3);
    }

    /// In a result `resting` says that a track was asleep in the last round of the relaxation,
    /// not that it is copper of the file: in the open end most tracks are. A short between two
    /// such tracks is taken out like any other; only copper of the file (with an origin) stays.
    #[test]
    fn a_short_between_tracks_that_rest_is_taken_out() {
        let b = cross();
        let rest = |t: Track| Track { resting: true, ..t };
        let mut geo = Geometry {
            tracks: vec![
                rest(track(0, &[(2., 10.), (10., 9.), (18., 10.)])),
                rest(track(1, &[(10., 2.), (10., 18.)])),
            ],
            vias: Vec::new(),
        };
        let mut st = stats(2);
        let legal = enforce(&b, &mut geo, &mut st);
        assert_eq!(legal.dropped, vec![("B".to_string(), "A".to_string())]);
        assert_eq!((legal.shorts_left, geo.tracks.len(), st.routed), (0, 1, 1));
        assert!(without_shorts(&b, &geo).is_none());
        // two tracks of the file that cross are not the route's to take out: the short is counted
        let file = |t: Track, o: u32| Track { origin: Some(o), ..t };
        let mut geo = Geometry {
            tracks: vec![
                file(track(0, &[(2., 10.), (10., 9.), (18., 10.)]), 0),
                file(track(1, &[(10., 2.), (10., 18.)]), 1),
            ],
            vias: Vec::new(),
        };
        let legal = enforce(&b, &mut geo, &mut stats(2));
        assert!(legal.dropped.is_empty() && legal.shorts_left > 0);
        assert_eq!(geo.tracks.len(), 2);
    }

    /// A track that ends on the middle of a line of its net: the line gets a corner there, so
    /// that the junction is one of ends. The copper is the same, and both halves keep the width.
    #[test]
    fn a_line_is_split_where_a_track_of_its_net_ends_on_it() {
        let b = board(&[("A", 2., 10.), ("A", 18., 10.), ("A", 10., 16.)]);
        let mut line = track(0, &[(2., 10.), (18., 10.)]);
        // the line is 0.15 wide by a zone around its middle
        let mut zone = crate::geom::Rect::empty();
        zone.add(pt(10., 10.));
        line.fitted.push((zone.grow(1e-6), 0.15));
        let mut geo = Geometry {
            // the end lies a hair beside the centre line, as the search leaves it
            tracks: vec![line, track(0, &[(10., 16.), (10., 10.0000005)])],
            vias: Vec::new(),
        };
        let mut st = stats(2);
        let legal = enforce(&b, &mut geo, &mut st);
        assert!(legal.open.is_empty() && legal.dangling.is_empty(), "{legal:?}");
        assert_eq!((legal.split, st.routed), (1, 2));
        let corner = pt(10., 10.0000005);
        assert_eq!(geo.tracks[0].pts, vec![pt(2., 10.), corner, pt(18., 10.)]);
        assert_eq!(
            geo.tracks[0].elems,
            vec![Elem::Line(pt(2., 10.), corner), Elem::Line(corner, pt(18., 10.))]
        );
        assert_eq!(geo.tracks[0].piece_width(pt(2., 10.), corner), 0.15);
        assert_eq!(geo.tracks[0].piece_width(corner, pt(18., 10.)), 0.15);
        // a second look splits nothing more, and copper of the file is never split
        assert_eq!(split_at_junctions(&mut geo), 0);
        let mut geo = Geometry {
            tracks: vec![
                Track {
                    origin: Some(0),
                    ..track(0, &[(2., 10.), (18., 10.)])
                },
                track(0, &[(10., 16.), (10., 10.)]),
            ],
            vias: Vec::new(),
        };
        assert_eq!(split_at_junctions(&mut geo), 0);
    }

    fn via(net: NetId, x: f64, y: f64) -> Via {
        Via {
            at: pt(x, y),
            diameter: 0.6,
            drill: 0.3,
            net,
            layer: None,
            origin: None,
            resting: false,
            half: Point::default(),
        }
    }

    /// A via whose centre lies beside a pad while its copper overlaps the pad joins the two, as
    /// in KiCad. And a via of the route must join on two layers: one that holds a track end and
    /// a pad on the same layer and nothing on another joins nothing, and goes.
    #[test]
    fn a_via_joins_the_pad_its_copper_overlaps_and_must_join_on_two_layers() {
        let mut b = board(&[("A", 2., 10.), ("A", 18., 10.)]);
        let second = b.layers[0].clone();
        b.layers.push(second);
        // the via stands 0.29 mm from the pad centre: 0.09 outside the pad, its copper 0.21 inside
        let on = |layer: u8, pts: &[(f64, f64)]| Track { layer, ..track(0, pts) };
        let geo = Geometry {
            tracks: vec![on(1, &[(2.29, 10.), (18., 10.)])],
            vias: vec![via(0, 2.29, 10.), via(0, 18., 10.)],
        };
        assert!(open_in_copper(&b, &geo, &[0]).is_empty());
        let mut geo = geo;
        let mut st = stats(1);
        let legal = enforce(&b, &mut geo, &mut st);
        assert!(legal.open.is_empty() && legal.dangling.is_empty(), "{legal:?}");
        assert_eq!((geo.vias.len(), geo.tracks.len(), st.routed), (2, 1, 1));
        // a via beside the first pad with a track end on the pad's layer only: nothing on layer 1
        let mut geo = Geometry {
            tracks: vec![on(0, &[(2., 10.), (18., 10.)]), on(0, &[(2.29, 10.), (2.29, 11.)])],
            vias: vec![via(0, 2.29, 10.)],
        };
        drop_idle_vias(&b, &mut geo, 0);
        assert!(geo.vias.is_empty());
    }

    #[test]
    fn an_open_connection_the_search_knows_is_not_counted_twice() {
        let b = board(&[("A", 2., 10.), ("A", 18., 10.), ("A", 10., 16.)]);
        let mut geo = Geometry {
            tracks: vec![track(0, &[(2., 10.), (18., 10.)])],
            vias: Vec::new(),
        };
        let mut st = stats(2);
        st.routed = 1;
        st.unrouted = vec!["A from P.2".to_string()];
        let legal = enforce(&b, &mut geo, &mut st);
        assert!(legal.open.is_empty());
        assert_eq!((st.routed, st.unrouted.len()), (1, 1));
    }
}
