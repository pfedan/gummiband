//! Geometric realization: turn topological segments into taut rubber bands. Each crossing gets a
//! portal on its edge (room for the neighbours and the clearance to the edge ends); the funnel
//! algorithm finds the shortest path through the portals. Every bend of that path wraps around
//! the edge end vertex of its portal, so the bends are replaced with arcs around those pivots and
//! straight tangents between them: neighbours around one pivot become concentric arcs.

use crate::math;
use rayon::prelude::*;

use crate::board::{LayerId, NetId, ObstacleKind};
use crate::geom::{Point, arc_polyline, orient, pt};
use crate::route::{End, Item, Router};

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Elem {
    Line(Point, Point),
    Arc(Point, Point, Point),
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Track {
    /// Polyline (arcs flattened), for checks and drawing.
    pub pts: Vec<Point>,
    /// Lines and arcs, for output.
    pub elems: Vec<Elem>,
    pub width: f64,
    pub layer: LayerId,
    pub net: NetId,
    /// Zones where the track narrows to `neck_w`.
    pub necks: Vec<crate::geom::Rect>,
    pub neck_w: f64,
    /// The soft chain this track comes from.
    pub origin: Option<u32>,
    /// Soft copper still exactly as the file has it.
    pub resting: bool,
    /// Widths fitted to the surroundings after the geometry step: (zone, width). A piece takes
    /// the smallest of those it touches, instead of the neck width.
    pub fitted: Vec<(crate::geom::Rect, f64)>,
    /// Part of a length tuning pattern (a KiCad `tuning_pattern` group): its index.
    pub pattern: Option<u32>,
    /// The router's segment of this track (`u32::MAX`: none).
    pub seg: u32,
    /// The segment its end lies on, when that is a track of its net and not a pad or a via.
    pub tap: Option<u32>,
}

impl Track {
    pub fn width_at(&self, p: Point) -> f64 {
        if self.necks.iter().any(|r| r.contains(p)) {
            self.neck_w
        } else {
            self.width
        }
    }

    /// Width of the arc a-m-b, which is one item in the file: the narrowest of its pieces. Not
    /// the width of its chord: the chord misses the widths fitted to the pieces of the arc and
    /// can touch a neck zone where the arc is wide.
    pub fn arc_width(&self, a: Point, m: Point, b: Point) -> f64 {
        flatten(&[Elem::Arc(a, m, b)])
            .windows(2)
            .map(|p| self.piece_width(p[0], p[1]))
            .fold(f64::INFINITY, f64::min)
            .min(self.width)
    }

    /// Width of the piece a-b: narrow if it touches a neck zone. The topology counts an edge
    /// as narrow only if the whole edge lies in a zone, so every piece that crosses such an edge
    /// touches the zone and is narrow as well.
    pub fn piece_width(&self, a: Point, b: Point) -> f64 {
        let fitted = self
            .fitted
            .iter()
            .filter(|(r, _)| r.touches_seg(a, b))
            .map(|(_, w)| *w)
            .fold(f64::INFINITY, f64::min);
        if fitted.is_finite() {
            return fitted;
        }
        if self.necks.iter().any(|r| r.touches_seg(a, b)) {
            self.neck_w
        } else {
            self.width
        }
    }
}

/// How far a joint between two lines may lie off the line through their far ends, in mm, and
/// still be no corner: the last digit of a board file.
const JOIN_TOL: f64 = 1e-6;

/// Neighbouring lines on one straight line and of one width become one line: a joint that is
/// no corner and no change of width is nothing the copper needs, and every piece is an item
/// of the file. `width` is the width of a piece; a joint that `keep` names stays (something
/// ends there). The joints go from `pts` as well. Returns how many went.
pub fn join_lines(
    pts: &mut Vec<Point>,
    elems: &mut Vec<Elem>,
    width: &dyn Fn(Point, Point) -> f64,
    keep: &dyn Fn(Point) -> bool,
) -> usize {
    let mut out: Vec<Elem> = Vec::with_capacity(elems.len());
    // the joints the last line of `out` has swallowed: they all stay on it
    let mut inside: Vec<Point> = Vec::new();
    let mut gone: Vec<Point> = Vec::new();
    for &e in elems.iter() {
        if let (Elem::Line(b, c), Some(&Elem::Line(a, b0))) = (e, out.last())
            && b0.dist(b) < 1e-9
            && (b - a).dot(c - b) > 0.0
            && (width(a, b) - width(b, c)).abs() < 1e-9
            && inside
                .iter()
                .chain([&b])
                .all(|j| crate::geom::point_seg_dist(*j, a, c) < JOIN_TOL)
            && !keep(b)
        {
            inside.push(b);
            gone.push(b);
            *out.last_mut().unwrap() = Elem::Line(a, c);
            continue;
        }
        inside.clear();
        out.push(e);
    }
    for j in &gone {
        if let Some(k) = (1..pts.len().saturating_sub(1)).find(|&k| pts[k].dist(*j) < 1e-9) {
            pts.remove(k);
        }
    }
    *elems = out;
    gone.len()
}

impl Geometry {
    /// [`join_lines`] on every track of the route (soft copper and tuning patterns keep the
    /// pieces they have). A joint stays where something ends: another track (KiCad does not
    /// take every end in the middle of a track for a connection, see
    /// `legal::split_at_junctions`), a via, or a pad with its centre there. Returns the joints
    /// that went.
    pub fn join_lines(&mut self, board: &crate::board::Board) -> usize {
        let mut stops: Vec<Point> = self
            .tracks
            .iter()
            .filter(|t| t.pts.len() >= 2)
            .flat_map(|t| [t.pts[0], *t.pts.last().unwrap()])
            .chain(self.vias.iter().map(|v| v.at))
            .chain(board.pins.iter().map(|p| p.pos))
            .collect();
        stops.sort_by(|a, b| a.x.total_cmp(&b.x));
        let keep = |j: Point| {
            let from = stops.partition_point(|s| s.x < j.x - 1e-6);
            stops[from..]
                .iter()
                .take_while(|s| s.x <= j.x + 1e-6)
                .any(|s| s.dist(j) < 1e-6)
        };
        let mut n = 0;
        for t in self
            .tracks
            .iter_mut()
            .filter(|t| t.origin.is_none() && t.pattern.is_none())
        {
            let (mut pts, mut elems) = (std::mem::take(&mut t.pts), std::mem::take(&mut t.elems));
            n += join_lines(&mut pts, &mut elems, &|a, b| t.piece_width(a, b), &keep);
            (t.pts, t.elems) = (pts, elems);
        }
        n
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Via {
    pub at: Point,
    pub diameter: f64,
    /// 0: not a via but a point on one layer where soft tracks branch or end.
    pub drill: f64,
    pub net: NetId,
    /// The layer of such a point.
    pub layer: Option<LayerId>,
    /// The soft node this via comes from.
    pub origin: Option<u32>,
    /// Soft copper still exactly as the file has it.
    pub resting: bool,
    /// Two vias side by side as one (the wide track of a pair): they sit at `at - half` and
    /// `at + half`, each `diameter - 2 |half|` wide, so that the copper is a capsule and
    /// `diameter` the circle around it. Zero for a round via.
    pub half: Point,
}

impl Via {
    /// Is this a pair of vias with a capsule as its copper?
    pub fn is_pair(&self) -> bool {
        self.half != Point::default()
    }
}

/// The half line of a via pair (length h) for the two tracks that leave it in directions `out`:
/// across the bisector of the band, as the split of a pair puts vias side by side. A band that
/// does not turn keeps `old`; the side stays that of `old`.
pub fn pair_half(out: [Point; 2], h: f64, old: Point) -> Point {
    let [e1, e2] = out;
    let t = if e1.dot(e2) < 0.0 { e2 - e1 } else { e1 + e2 };
    if t.len2() < 1e-18 {
        return old;
    }
    let t = t.norm();
    let n = pt(-t.y, t.x) * h;
    if n.dot(old) < 0.0 { n * -1.0 } else { n }
}

/// The direction in which a track leaves its end at the start (or at the end) point.
pub fn leaving(pts: &[Point], at_start: bool) -> Option<Point> {
    let mut it: Box<dyn Iterator<Item = &Point>> = if at_start {
        Box::new(pts.iter())
    } else {
        Box::new(pts.iter().rev())
    };
    let first = *it.next()?;
    it.find(|q| q.dist(first) > 1e-6).map(|&q| (q - first).norm())
}

/// Turn every via pair to the tracks that end on it (two of them; else it keeps its line).
pub fn turn_pairs(geo: &mut Geometry) {
    for vi in 0..geo.vias.len() {
        let v = &geo.vias[vi];
        if !v.is_pair() {
            continue;
        }
        let out: Vec<Point> = geo
            .tracks
            .iter()
            .filter(|t| t.net == v.net && t.pts.len() > 1)
            .flat_map(|t| {
                let (a, b) = (t.pts[0], *t.pts.last().unwrap());
                [(a, true), (b, false)]
                    .into_iter()
                    .filter(|(p, _)| p.dist(v.at) < 1e-6)
                    .filter_map(|(_, s)| leaving(&t.pts, s))
                    .collect::<Vec<_>>()
            })
            .collect();
        if let [e1, e2] = out[..] {
            let h = v.half.len();
            geo.vias[vi].half = pair_half([e1, e2], h, v.half);
        }
    }
}

#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct Geometry {
    pub tracks: Vec<Track>,
    pub vias: Vec<Via>,
}

/// One side of a portal: the point and the edge end vertex it is packed against.
#[derive(Debug, Clone, Copy)]
pub struct Side {
    pub p: Point,
    pub pivot: Point,
    pub pivot_id: u32,
    /// With [`Router::stack_band`]: what the track needs around the pivot, counted squarely
    /// from the vertex (0: nothing, the vertex belongs to its own net).
    pub rad: f64,
    /// With [`Router::stack_band`]: the track is the one next to the pivot on this edge, and
    /// the point lies where its need at that end puts it (no bound from an apex, no share of
    /// the slack).
    pub attached: bool,
}

/// How the band of a segment came about ([`Router::seg_path_info`]).
#[derive(Debug, Clone, Default)]
pub struct PathInfo {
    /// 0: a band of tangents and arcs. Else the funnel polyline, and why: 1 no chain of
    /// tangents around the wraps, 2 the band leaves its triangles, 3 it misses a portal, 4 arcs
    /// are off.
    pub fallback: u8,
    /// Fallback 1 without [`Router::stack_band`]: the kind of [`band_why`].
    pub why: u8,
    /// Bends of the funnel.
    pub bends: usize,
    /// Wraps of the band, and how many of them have the radius of the stack.
    pub wraps: usize,
    pub stack_wraps: usize,
    /// With [`Router::stack_band`]: corners of the band, where a wrap gave way or the vertex
    /// asks for no distance.
    pub corners: usize,
    /// Wraps that gave way, by reason: the kinds 1 to 5 of [`band_why`], then a missed portal,
    /// a piece outside the triangles, and an arc out of the lane of the track.
    pub gave_way: [usize; 8],
    /// Vertices the band passes too closely and cannot wrap: no tangents with them, or the wrap
    /// gave way again later.
    pub refused: usize,
    /// Wraps with the radius of the stack that went without a corner, because the band keeps
    /// its distance from the vertex without them.
    pub idle: usize,
    /// Bends around a vertex that asks for no distance (corners from the start).
    pub own: usize,
}

/// A disc of [`Router::discs`]: the band keeps `r` from the vertex `id` at `c` and passes it on
/// `side` (+1: the vertex on the left of travel). `first` and `last` are the crossings of the
/// track at whose edges the vertex is an end.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Disc {
    pub(crate) c: Point,
    pub(crate) r: f64,
    pub(crate) side: f64,
    pub(crate) id: u32,
    pub(crate) first: usize,
    pub(crate) last: usize,
    /// The vertex is an end of an edge the track crosses, so the track is known to pass it on
    /// `side`. Else the disc reaches in from beside the way of the track (a via in the next
    /// triangle, the disc of a neighbour): it counts only where the band comes within its
    /// radius, and is wrapped on `side` then.
    pub(crate) sure: bool,
}

/// Radius of the disc of a vertex that asks for no distance ([`Router::disc_path`]): small
/// enough that its arc is drawn as a corner.
const POINT_DISC: f64 = 1e-5;

/// What two neighbouring tracks keep between their discs beyond their gap, mm.
const DISC_SAG: f64 = 0.002;

/// How far the polyline `path` falls short of disc `d`: its radius less the distance of the
/// nearest piece from the centre, where the centre lies on the side the disc asks for; where it
/// lies on the other side, the radius plus that distance (the string has to come round).
fn disc_deficit(path: &[Point], d: &Disc) -> f64 {
    let mut best = (f64::INFINITY, 0.0);
    for w in path.windows(2) {
        let dist = crate::geom::point_seg_dist(d.c, w[0], w[1]);
        if dist < best.0 {
            best = (dist, orient(w[0], w[1], d.c));
        }
    }
    if !best.0.is_finite() {
        return 0.0;
    }
    let right_side = !d.sure || best.1 * d.side >= 0.0 || best.0 < 1e-9;
    if right_side { d.r - best.0 } else { d.r + best.0 }
}

/// Circle the band wraps around: `side` +1 = pivot on the left of travel (left turn).
#[derive(Debug, Clone, Copy)]
pub(crate) struct Wrap {
    pub(crate) c: Point,
    pub(crate) r: f64,
    pub(crate) side: f64,
    pub(crate) id: u32,
}

impl Router<'_> {
    pub(crate) fn end_point(&self, e: End) -> Point {
        match e {
            End::Pin(p) => self.board.pins[p].pos,
            End::Via(site) => self.site_point(site),
            End::Point(p) | End::Area(p, _) => p,
            End::Tap(sid, e) => {
                let s = self.segs[sid as usize].as_ref().unwrap();
                match s.crossings.iter().position(|c| c.e == e) {
                    Some(k) => {
                        let (l, r) = self.portal(sid, k);
                        (l.p + r.p) * 0.5
                    }
                    None => self.seg_end(sid, true),
                }
            }
        }
    }

    /// Where segment `sid` starts (`first`) or ends. At a pin that is the position of the pin,
    /// and for a segment that crosses the outline of the pad at an exit (`exit.rs`) the exit
    /// point: the piece from the pin to there belongs to the pad.
    pub(crate) fn seg_end(&self, sid: u32, first: bool) -> Point {
        let s = self.segs[sid as usize].as_ref().unwrap();
        let end = if first { s.start } else { s.end };
        let End::Pin(p) = end else {
            return self.end_point(end);
        };
        let pos = self.board.pins[p].pos;
        if self.board.exits.is_empty() {
            return pos;
        }
        let c = if first { s.crossings.first() } else { s.crossings.last() };
        match c.and_then(|c| self.sub.m[s.li as usize].exit_edges.get(&c.e)) {
            Some(&x)
                if x != crate::topo::NONE && self.pin_group[self.board.exits[x as usize].pin] == self.pin_group[p] =>
            {
                self.board.exits[x as usize].at
            }
            _ => pos,
        }
    }

    /// Portal (left, right) of segment `sid` on its crossing `k`, as seen in travel direction.
    fn portal(&self, sid: u32, k: usize) -> (Side, Side) {
        let s = self.segs[sid as usize].as_ref().unwrap();
        let li = s.li as usize;
        let c = s.crossings[k];
        let list = &self.cross[li][c.e as usize];
        let items: Vec<Item> = self.items_on(li, c.e);
        let idx = list.iter().position(|&x| x == sid).unwrap();
        let m = &self.sub.m[li];
        let [u, v] = m.edges[c.e as usize];
        let (pu, pv) = (m.verts[u as usize], m.verts[v as usize]);
        let len = pu.dist(pv);
        let dir = (pv - pu) * (1.0 / len);
        let packed = if self.soft_on {
            Err(self.soft_interval(li, c.e, idx, &items))
        } else {
            Ok(self.pack(li, c.e, &items, None))
        };
        let fits = matches!(packed, Ok(Some(_)));
        let (lo, hi) = match packed {
            Err(iv) => iv,
            Ok(Some(iv)) => {
                // Split the common slack so that the portals of neighbours are disjoint.
                let n = iv.len() as f64;
                let slack = iv.iter().map(|(a, b)| b - a).fold(f64::INFINITY, f64::min).max(0.0);
                let (a, _) = iv[idx];
                (a + slack * idx as f64 / n, a + slack * (idx as f64 + 1.0) / n)
            }
            Ok(None) => {
                // over capacity: share the edge evenly
                let f = (idx as f64 + 0.5) / items.len() as f64;
                (len * f, len * f)
            }
        };
        // the radius of the stack: for the track next to an edge end that lies where its need
        // puts it, the need without the factor of a slanted obstacle side. That factor is the
        // price of crossing the edge at a slant; an arc around the vertex keeps the plain need.
        let mut stack = [(0.0, false); 2];
        if self.stack_band && fits {
            let it = &items[idx];
            let extra = self.cfg.margin + self.hot_extra(pu, pv);
            for (end, at, next) in [(0, lo, idx == 0), (1, len - hi, idx + 1 == items.len())] {
                if next && (at - (self.end_need(li, c.e, end, it) + extra)).abs() < 1e-9 {
                    let need = self.end_need_square(li, c.e, end, it);
                    stack[end] = (if need > 0.0 { need + extra } else { 0.0 }, true);
                }
            }
        }
        let slo = Side {
            p: pu + dir * lo,
            pivot: pu,
            pivot_id: u,
            rad: stack[0].0,
            attached: stack[0].1,
        };
        let shi = Side {
            p: pu + dir * hi,
            pivot: pv,
            pivot_id: v,
            rad: stack[1].0,
            attached: stack[1].1,
        };
        // Leaving triangle `from` (counter-clockwise) through edge a->b: b is on the left.
        let t = m.tris[c.from as usize];
        let a = t[m.local_edge(c.from, c.e)];
        if a == u { (shi, slo) } else { (slo, shi) }
    }

    /// The way of segment `sid` through a geometry that has moved on from the topology, as a taut
    /// polyline: on every edge it crosses it goes between where its neighbours really are, so
    /// that it starts on the right side of each of them. `vertex` says where a mesh vertex of
    /// its layer is now, `place` where a segment crosses a line between two points, as a fraction
    /// of it (None for a segment without a place, which shares the room with this one).
    pub(crate) fn threaded(
        &self,
        sid: u32,
        vertex: &dyn Fn(usize, u32) -> Point,
        place: &dyn Fn(u32, Point, Point) -> Option<f64>,
    ) -> Vec<Point> {
        let s = self.segs[sid as usize].as_ref().unwrap();
        let li = s.li as usize;
        let m = &self.sub.m[li];
        let (start, end) = (self.seg_end(sid, true), self.seg_end(sid, false));
        let mut portals = vec![(start, start)];
        for c in &s.crossings {
            let list = &self.cross[li][c.e as usize];
            let items = self.items_on(li, c.e);
            let idx = list.iter().position(|&x| x == sid).unwrap();
            let [u, v] = m.edges[c.e as usize];
            let (pu, pv) = (vertex(li, u), vertex(li, v));
            let len = pu.dist(pv).max(1e-9);
            let dir = (pv - pu) * (1.0 / len);
            let margin = self.cfg.margin;
            let mut lo = (0.0, idx + 1, self.end_need(li, c.e, 0, &items[idx]) + margin);
            for i in (0..idx).rev() {
                if let Some(x) = place(list[i], pu, pv) {
                    lo = (x * len, idx - i, Self::gap(self.board, &items[i], &items[idx]));
                    break;
                }
            }
            let mut hi = (len, list.len() - idx, self.end_need(li, c.e, 1, &items[idx]) + margin);
            for i in idx + 1..list.len() {
                if let Some(x) = place(list[i], pu, pv) {
                    hi = (x * len, i - idx, Self::gap(self.board, &items[idx], &items[i]));
                    break;
                }
            }
            // this one and the others without a place share what is between those two
            let (a, b) = (lo.0 + lo.2, hi.0 - hi.2);
            let share = (lo.1 + hi.1 - 1) as f64;
            let (x, y) = if a < b {
                (
                    a + (b - a) * (lo.1 - 1) as f64 / share,
                    a + (b - a) * lo.1 as f64 / share,
                )
            } else {
                let mid = (lo.0 + hi.0) * 0.5;
                (mid, mid)
            };
            let (x, y) = (pu + dir * x, pu + dir * y);
            // leaving triangle `from` (counter-clockwise) through edge a->b: b is on the left
            let t = m.tris[c.from as usize];
            portals.push(if t[m.local_edge(c.from, c.e)] == u {
                (y, x)
            } else {
                (x, y)
            });
        }
        portals.push((end, end));
        let mut pts = vec![start];
        pts.extend(
            funnel_bends(&portals)
                .into_iter()
                .map(|(i, left)| if left { portals[i].0 } else { portals[i].1 }),
        );
        pts.push(end);
        pts.dedup_by(|a, b| a.dist(*b) < 1e-9);
        if pts.len() < 2 {
            pts.push(end);
        }
        pts
    }

    /// Rubber band of one segment: lines and arcs. Starts with the bends of the funnel, then
    /// adds every edge end vertex the band comes too close to as a further wrap, until the band
    /// keeps its distance from all of them.
    ///
    /// With [`Router::stack_band`] the radius of a wrap is the need of the stack where the
    /// track lies next to the vertex, and a wrap that cannot be kept becomes a corner on its
    /// own instead of taking all arcs of the track with it ([`stack_band`]).
    pub fn seg_path(&self, sid: u32) -> Vec<Elem> {
        self.seg_path_info(sid).0
    }

    /// The portals of segment `sid`: its start, one per crossing, its end.
    fn seg_portals(&self, sid: u32) -> Vec<(Side, Side)> {
        let s = self.segs[sid as usize].as_ref().unwrap();
        let start = self.seg_end(sid, true);
        let end = self.seg_end(sid, false);
        let ends = |p: Point| Side {
            p,
            pivot: p,
            pivot_id: u32::MAX,
            rad: 0.0,
            attached: false,
        };
        let mut portals = Vec::with_capacity(s.crossings.len() + 2);
        portals.push((ends(start), ends(start)));
        for k in 0..s.crossings.len() {
            portals.push(self.portal(sid, k));
        }
        portals.push((ends(end), ends(end)));
        portals
    }

    /// [`Router::seg_path`], and how the band came about.
    pub fn seg_path_info(&self, sid: u32) -> (Vec<Elem>, PathInfo) {
        self.seg_path_info_with(sid, self.disc_band)
    }

    /// [`Router::seg_path_info`] with the band of the discs or, `disc` false, without it.
    fn seg_path_info_with(&self, sid: u32, disc: bool) -> (Vec<Elem>, PathInfo) {
        let mut info = PathInfo::default();
        let portals = self.seg_portals(sid);
        let (start, end) = (portals[0].0.p, portals[portals.len() - 1].0.p);
        let pts: Vec<(Point, Point)> = portals.iter().map(|(l, r)| (l.p, r.p)).collect();
        let bends = funnel_bends(&pts);
        info.bends = bends.len();
        let polyline = || -> Vec<Elem> {
            let mut p = vec![start];
            for &(i, left) in &bends {
                p.push(if left { portals[i].0.p } else { portals[i].1.p });
            }
            p.push(end);
            p.dedup_by(|a, b| a.dist(*b) < 1e-9);
            p.windows(2).map(|w| Elem::Line(w[0], w[1])).collect()
        };
        // arcs around the edge ends know nothing of the imported tracks in the way
        if crate::knobs::NO_ARCS.on() || self.soft_on {
            info.fallback = 4;
            return (polyline(), info);
        }
        if disc {
            return match self.disc_path(sid, &mut info) {
                Some(b) => (b, info),
                None => (polyline(), info),
            };
        }
        if self.stack_band {
            let leaves = |path: &[Point]| self.corridor_exit(sid, path);
            return match stack_band(&portals, &bends, &leaves, &mut info) {
                Some(b) => (b, info),
                None => (polyline(), info),
            };
        }
        // wraps with the portal index they belong to
        let mut wraps: Vec<(usize, Wrap)> = Vec::new();
        let add = |wraps: &mut Vec<(usize, Wrap)>, i: usize, left: bool| {
            let side = if left { portals[i].0 } else { portals[i].1 };
            let r = side.p.dist(side.pivot);
            let sgn = if left { 1.0 } else { -1.0 };
            if let Some(w) = wraps.iter_mut().find(|(_, w)| w.id == side.pivot_id && w.side == sgn) {
                w.1.r = w.1.r.max(r);
                return;
            }
            let pos = wraps.iter().position(|(k, _)| *k > i).unwrap_or(wraps.len());
            wraps.insert(
                pos,
                (
                    i,
                    Wrap {
                        c: side.pivot,
                        r,
                        side: sgn,
                        id: side.pivot_id,
                    },
                ),
            );
        };
        for &(i, left) in &bends {
            add(&mut wraps, i, left);
        }
        let first = band_why(start, end, &wraps.iter().map(|w| w.1).collect::<Vec<_>>());
        if let Err((_, kind)) = first {
            info.why = kind;
        }
        let mut best = first.ok();
        for _ in 0..12 {
            let Some(elems) = &best else { break };
            let path = flatten(elems);
            // the worst violated pivot
            let mut worst: Option<(f64, usize, bool)> = None;
            for i in 1..portals.len() - 1 {
                for left in [true, false] {
                    let side = if left { portals[i].0 } else { portals[i].1 };
                    let r = side.p.dist(side.pivot);
                    if r < 1e-6 {
                        continue;
                    }
                    let d = path
                        .windows(2)
                        .map(|w| crate::geom::point_seg_dist(side.pivot, w[0], w[1]))
                        .fold(f64::INFINITY, f64::min);
                    let deficit = r - d;
                    if deficit > 1e-4 && worst.is_none_or(|w| deficit > w.0) {
                        worst = Some((deficit, i, left));
                    }
                }
            }
            let Some((_, i, left)) = worst else { break };
            let side = if left { portals[i].0 } else { portals[i].1 };
            let known = wraps.iter().any(|(_, w)| {
                w.id == side.pivot_id
                    && w.side == if left { 1.0 } else { -1.0 }
                    && w.r >= side.p.dist(side.pivot) - 1e-9
            });
            if known {
                break;
            }
            add(&mut wraps, i, left);
            match band(start, end, &wraps.iter().map(|w| w.1).collect::<Vec<_>>()) {
                Some(b) => best = Some(b),
                None => {
                    info.refused += 1;
                    break;
                }
            }
        }
        // the band must stay inside the triangles its topology passes through, and it must keep
        // to its portals: where the wraps would leave them (and could cross a neighbour), the
        // funnel polyline is the safe choice. A wrap takes the largest radius any of its portals
        // has at that pivot, and on a long edge with slack that is far more than the track needs
        // there: the band then swings wide around the pivot, inside its triangles but across the
        // track next to it (two crossings, a short on the board). The portals of neighbours are
        // disjoint, so a band that passes through all of its own crosses none of them.
        let Some(b) = best else {
            info.fallback = 1;
            return (polyline(), info);
        };
        let flat = flatten(&b);
        if !self.in_corridor(sid, &flat) {
            info.fallback = 2;
            return (polyline(), info);
        }
        if missed_portal(&portals, &flat, false).is_some() {
            info.fallback = 3;
            return (polyline(), info);
        }
        info.wraps = wraps.len();
        (b, info)
    }

    /// The discs of segment `sid` ([`Router::disc_band`]): for every vertex at an end of an
    /// edge it crosses, on the side it passes it, the radius the stack asks for there. On an
    /// edge, a track keeps from the vertex at either end what the copper of the vertex asks
    /// for (counted squarely, as an arc around the vertex keeps it) plus the gaps of the tracks
    /// between it and that vertex; a vertex passed over several edges takes the largest. The
    /// radius follows from the topology alone, so the band of one track does not depend on the
    /// geometry of another: two tracks around the same vertices have radii that differ by the
    /// gap between them, and their bands run parallel. Sorted by where along the track the
    /// vertex is passed.
    pub(crate) fn discs(&self, sid: u32) -> Vec<Disc> {
        self.discs_and_portals(sid).0
    }

    /// The discs of a segment as (centre, radius, side) for a drawing.
    pub fn debug_discs(&self, sid: u32) -> Vec<(Point, f64, f64)> {
        self.discs(sid).iter().map(|d| (d.c, d.r, d.side)).collect()
    }

    /// [`Router::discs`], and per crossing the portal those radii leave on its edge (left,
    /// right in the direction of travel): from the radius at one end to the radius at the
    /// other, the point in between in proportion where the two take more than the edge has.
    fn discs_and_portals(&self, sid: u32) -> (Vec<Disc>, Vec<(Point, Point)>) {
        let (mut discs, portals) = self.own_discs(sid);
        if self.disc_inherit {
            self.inherit_discs(sid, &mut discs);
        }
        if let Some(extra) = self.extra_discs.get(&sid) {
            for x in extra {
                match discs.iter_mut().find(|d| d.id == x.id && d.side == x.side) {
                    Some(d) => d.r = d.r.max(x.r),
                    None => discs.push(*x),
                }
            }
        }
        discs.sort_by(|a, b| (a.first + a.last).cmp(&(b.first + b.last)).then(a.first.cmp(&b.first)));
        (discs, portals)
    }

    /// The discs a track takes over from the tracks inside it. Its own discs come from the
    /// vertices of its triangles. Where it cuts one corner of a triangle, a track that cuts
    /// another corner leaves through the third edge and may bend around the vertex beyond it:
    /// its band there bulges back into the triangle, into the way of the outer track, which
    /// has no edge at that vertex and so no disc. The outer track keeps, around that vertex,
    /// the radius of the inner one plus what lies between the two at the corner they share.
    fn inherit_discs(&self, sid: u32, discs: &mut Vec<Disc>) {
        let s = self.segs[sid as usize].as_ref().unwrap();
        let li = s.li as usize;
        let m = &self.sub.m[li];
        let own = discs.clone();
        let mut theirs: std::collections::HashMap<u32, Vec<Disc>> = std::collections::HashMap::new();
        let mut new: Vec<Disc> = Vec::new();
        for (k, w) in s.crossings.windows(2).enumerate() {
            // the triangle between two crossings, its third edge and the vertex beyond it
            let t = w[1].from as usize;
            let Some(j) = (0..3).find(|&j| m.tri_edges[t][j] != w[0].e && m.tri_edges[t][j] != w[1].e) else {
                continue;
            };
            let (c, n) = (m.tri_edges[t][j], m.tri_nbrs[t][j]);
            if n == crate::topo::NONE {
                continue;
            }
            let [q, r] = m.edges[c as usize];
            let Some(&apex) = m.tris[n as usize].iter().find(|&&v| v != q && v != r) else {
                continue;
            };
            if own.iter().any(|d| d.id == apex) {
                continue;
            }
            for &o in &self.cross[li][c as usize] {
                if o == sid {
                    continue;
                }
                let Some(os) = self.segs[o as usize].as_ref() else {
                    continue;
                };
                // the same way through the shared edge, or against it
                let Some(dir) = os.crossings.iter().find_map(|x| {
                    if x.e == w[0].e {
                        Some(if x.from == w[0].from { 1.0 } else { -1.0 })
                    } else if x.e == w[1].e {
                        Some(if x.from == w[1].from { 1.0 } else { -1.0 })
                    } else {
                        None
                    }
                }) else {
                    continue;
                };
                let od = theirs.entry(o).or_insert_with(|| self.own_discs(o).0);
                for far in od.iter().filter(|d| d.id == apex && d.r > POINT_DISC) {
                    for corner in od.iter().filter(|d| (d.id == q || d.id == r) && d.side == far.side) {
                        let Some(me) = own.iter().find(|x| x.id == corner.id && x.side == corner.side * dir) else {
                            continue;
                        };
                        let off = me.r - corner.r;
                        if off <= 1e-9 {
                            continue;
                        }
                        let rad = far.r + off;
                        match new.iter_mut().find(|d| d.id == apex && d.side == me.side) {
                            Some(d) => d.r = d.r.max(rad),
                            None => new.push(Disc {
                                c: far.c,
                                r: rad,
                                side: me.side,
                                id: apex,
                                first: k,
                                last: k + 1,
                                sure: false,
                            }),
                        }
                    }
                }
            }
        }
        discs.extend(new);
    }

    /// Radius of the disc of the track at place `idx` on edge `e` around end `end` of the edge:
    /// what the track itself needs from the vertex, or the whole disc of the track inside it
    /// there and the gap of the two. The whole disc of that track, not its radius on this
    /// edge: on another edge at the vertex it may lie around a track this edge does not carry,
    /// and its band is a circle around the vertex, as far out here as there.
    fn stack_radius(
        &self,
        li: usize,
        e: u32,
        end: usize,
        idx: usize,
        memo: &mut std::collections::HashMap<(u32, u32), f64>,
        depth: usize,
    ) -> f64 {
        let m = &self.sub.m[li];
        let list = &self.cross[li][e as usize];
        let [u, v] = m.edges[e as usize];
        let hot = self.hot_extra(m.verts[u as usize], m.verts[v as usize]);
        let it = self.item_on(li, e, list[idx]);
        let need = self.end_need_square(li, e, end, &it);
        let base = if need > 0.0 { need + self.cfg.margin + hot } else { 0.0 };
        let inner = if end == 0 {
            idx.checked_sub(1)
        } else {
            (idx + 1 < list.len()).then_some(idx + 1)
        };
        let Some(j) = inner else {
            return base;
        };
        let jt = self.item_on(li, e, list[j]);
        let gap = if end == 0 {
            Self::gap(self.board, &jt, &it)
        } else {
            Self::gap(self.board, &it, &jt)
        };
        // between two tracks the arcs of both lie a little apart, so that what the pieces of
        // the arcs cut off does not count against the gap
        let below = self.whole_radius(li, list[j], [u, v][end], (e, end, j), memo, depth + 1);
        base.max(below + gap + hot + DISC_SAG)
    }

    /// The disc of segment `sid` around vertex `vert`: the largest radius over its crossings
    /// of the edges at the vertex ([`Router::stack_radius`]). `here` is one of them.
    fn whole_radius(
        &self,
        li: usize,
        sid: u32,
        vert: u32,
        here: (u32, usize, usize),
        memo: &mut std::collections::HashMap<(u32, u32), f64>,
        depth: usize,
    ) -> f64 {
        if let Some(&r) = memo.get(&(sid, vert)) {
            return r;
        }
        // a track that winds around the vertex more than once meets itself: it counts as it
        // stands on the edge asked for
        memo.insert((sid, vert), 0.0);
        let mut r = self.stack_radius(li, here.0, here.1, here.2, memo, depth);
        if depth < 64
            && let Some(s) = self.segs[sid as usize].as_ref()
            && s.li as usize == li
        {
            let m = &self.sub.m[li];
            for c in &s.crossings {
                if c.e == here.0 {
                    continue;
                }
                let [u, v] = m.edges[c.e as usize];
                let end = if u == vert {
                    0
                } else if v == vert {
                    1
                } else {
                    continue;
                };
                if let Some(idx) = self.cross[li][c.e as usize].iter().position(|&x| x == sid) {
                    r = r.max(self.stack_radius(li, c.e, end, idx, memo, depth));
                }
            }
        }
        memo.insert((sid, vert), r);
        r
    }

    /// The discs of the vertices of the track's own triangles, and the portals of
    /// [`Router::discs_and_portals`].
    fn own_discs(&self, sid: u32) -> (Vec<Disc>, Vec<(Point, Point)>) {
        let mut portals = Vec::new();
        let s = self.segs[sid as usize].as_ref().unwrap();
        let li = s.li as usize;
        let m = &self.sub.m[li];
        let mut memo = std::collections::HashMap::new();
        let mut discs: Vec<Disc> = Vec::new();
        for (k, c) in s.crossings.iter().enumerate() {
            let list = &self.cross[li][c.e as usize];
            let idx = list.iter().position(|&x| x == sid).unwrap();
            let [u, v] = m.edges[c.e as usize];
            let (pu, pv) = (m.verts[u as usize], m.verts[v as usize]);
            let ru = self.stack_radius(li, c.e, 0, idx, &mut memo, 0);
            let rv = self.stack_radius(li, c.e, 1, idx, &mut memo, 0);
            // leaving triangle `from` (counter-clockwise) through edge a->b: b is on the left
            let t = m.tris[c.from as usize];
            let left_is_v = t[m.local_edge(c.from, c.e)] == u;
            let len = pu.dist(pv).max(1e-12);
            let (a, b) = if ru + rv > len {
                (len * ru / (ru + rv), len * ru / (ru + rv))
            } else {
                (ru, len - rv)
            };
            let (qa, qb) = (pu.lerp(pv, a / len), pu.lerp(pv, b / len));
            portals.push(if left_is_v { (qb, qa) } else { (qa, qb) });
            for (vid, at, r, left) in [(u, pu, ru, !left_is_v), (v, pv, rv, left_is_v)] {
                let side = if left { 1.0 } else { -1.0 };
                match discs.iter_mut().find(|d| d.id == vid && d.side == side) {
                    Some(d) => {
                        d.r = d.r.max(r);
                        d.last = k;
                    }
                    None => discs.push(Disc {
                        c: at,
                        r,
                        side,
                        id: vid,
                        first: k,
                        last: k,
                        sure: true,
                    }),
                }
            }
        }
        (discs, portals)
    }

    /// The band of segment `sid` as the taut string against its discs ([`Router::discs`]):
    /// starting from the straight line, the disc the string violates most is wrapped, and a
    /// wrap the string has lifted off (it would turn the wrong way round its disc) is dropped,
    /// until the string keeps every disc. None where that does not settle, where two discs
    /// leave no tangent, or where the string leaves its triangles.
    fn disc_path(&self, sid: u32, info: &mut PathInfo) -> Option<Vec<Elem>> {
        self.disc_path_traced(sid, info, &mut None)
    }

    /// What [`Router::disc_path`] did for one segment, step by step, for a reader: its
    /// crossings, its discs, every wrap put on and taken off, and the wraps of the result.
    pub fn debug_disc_trace(&self, sid: u32) -> String {
        let mut trace = Some(String::new());
        self.disc_path_traced(sid, &mut PathInfo::default(), &mut trace);
        trace.unwrap_or_default()
    }

    fn disc_path_traced(&self, sid: u32, info: &mut PathInfo, trace: &mut Option<String>) -> Option<Vec<Elem>> {
        use std::fmt::Write;
        let start = self.seg_end(sid, true);
        let end = self.seg_end(sid, false);
        // a vertex that asks for no distance (its own net, a free Steiner point) is still passed
        // on its side: a point the string may touch but not cross
        let (discs, tight) = self.discs_and_portals(sid);
        let mut discs: Vec<Disc> = discs
            .into_iter()
            .map(|d| Disc {
                r: d.r.max(POINT_DISC),
                ..d
            })
            .collect();
        // an end of the track inside a disc: the disc reaches as far as the end, there is no
        // tangent from a point inside a circle
        for d in &mut discs {
            let near = start.dist(d.c).min(end.dist(d.c));
            if d.r > near * 0.999 {
                d.r = (near * 0.999).max(POINT_DISC / 2.0);
                info.gave_way[0] += 1;
            }
        }
        // a gate: a short edge between two vertices that ask for nothing, the exit of a pad of
        // the track's own net. The track goes through its middle, so no disc reaches that far.
        let s = self.segs[sid as usize].as_ref().unwrap();
        let m = &self.sub.m[s.li as usize];
        {
            let small = |id: u32, discs: &[Disc]| discs.iter().filter(|d| d.id == id).all(|d| d.r <= POINT_DISC);
            let gates: Vec<(u32, u32, [Point; 3])> = s
                .crossings
                .iter()
                .filter_map(|c| {
                    let [u, v] = m.edges[c.e as usize];
                    let (pu, pv) = (m.verts[u as usize], m.verts[v as usize]);
                    (pu.dist(pv) < 0.05 && small(u, &discs) && small(v, &discs))
                        .then(|| (u, v, [pu.lerp(pv, 0.5), pu, pv]))
                })
                .collect();
            for d in &mut discs {
                for (u, v, at) in &gates {
                    if d.id == *u || d.id == *v {
                        continue;
                    }
                    // nor as far as the corners of the gate: the track passes each on its side
                    let near = at.iter().map(|g| g.dist(d.c)).fold(f64::INFINITY, f64::min);
                    if d.r > POINT_DISC && d.r > near - 1e-6 {
                        d.r = (near - 1e-6).max(POINT_DISC / 2.0);
                        info.gave_way[0] += 1;
                    }
                }
            }
        }
        if let Some(t) = trace {
            let _ = writeln!(
                t,
                "seg {sid} layer {} net {} from {:?} ({:.3},{:.3}) to {:?} ({:.3},{:.3})",
                s.li, s.net, s.start, start.x, start.y, s.end, end.x, end.y
            );
            for e in [s.start, s.end] {
                if let End::Via(site) = e {
                    let _ = match m.site_vert.get(&site) {
                        Some(&v) => writeln!(
                            t,
                            "  via {site} is vertex {v} ({:.3},{:.3})",
                            m.verts[v as usize].x, m.verts[v as usize].y
                        ),
                        None => writeln!(t, "  via {site} is no vertex of the layer"),
                    };
                }
            }
            for (k, c) in s.crossings.iter().enumerate() {
                let [u, v] = m.edges[c.e as usize];
                let (pu, pv) = (m.verts[u as usize], m.verts[v as usize]);
                let _ = writeln!(
                    t,
                    "  crossing {k}: {u} ({:.3},{:.3}) - {v} ({:.3},{:.3})",
                    pu.x, pu.y, pv.x, pv.y
                );
            }
            for d in &discs {
                let _ = writeln!(
                    t,
                    "  disc {} ({:.3},{:.3}) r {:.4} side {:+} crossings {}..{}{}",
                    d.id,
                    d.c.x,
                    d.c.y,
                    d.r,
                    d.side,
                    d.first,
                    d.last,
                    if d.sure { "" } else { " taken over" }
                );
            }
        }
        // discs that are out of the game: inside another on the same side, or on the same centre
        let mut dead = vec![false; discs.len()];
        // the wraps of the string, each with its place along the track (in crossings). The
        // funnel through the portals of the radii says which vertices the string bends around
        // and in which order, and that it passes every vertex on its side.
        let mut portals = vec![(start, start)];
        portals.extend(tight);
        portals.push((end, end));
        let mut on: Vec<usize> = Vec::new();
        let mut place: Vec<f64> = Vec::new();
        for (i, left) in funnel_bends(&portals) {
            let c = s.crossings[i - 1];
            let [u, v] = m.edges[c.e as usize];
            let t = m.tris[c.from as usize];
            let left_is_v = t[m.local_edge(c.from, c.e)] == u;
            let (vid, side) = (if left == left_is_v { v } else { u }, if left { 1.0 } else { -1.0 });
            if let Some(j) = discs.iter().position(|d| d.id == vid && d.side == side)
                && !on.contains(&j)
            {
                on.push(j);
                place.push(i as f64 - 1.0);
            }
        }
        let limit = 6 * discs.len() + 8;
        // what two discs gave each other in a full passage, and the pairs that got it back once
        let mut cuts: Vec<(usize, usize, f64, f64)> = Vec::new();
        let mut given_back: Vec<(usize, usize)> = Vec::new();
        let mut best: Option<Vec<Elem>> = None;
        for _ in 0..limit {
            let wraps: Vec<Wrap> = on
                .iter()
                .map(|&i| Wrap {
                    c: discs[i].c,
                    r: discs[i].r,
                    side: discs[i].side,
                    id: discs[i].id,
                })
                .collect();
            let elems = match band_why(start, end, &wraps) {
                Ok(e) => e,
                Err((i, 5)) if i >= 1 && i <= on.len() => {
                    // lifted off: the string turns the wrong way round this disc
                    if let Some(t) = trace {
                        let _ = writeln!(t, "  off {}: more than 270 degrees", discs[on[i - 1]].id);
                    }
                    // a disc that gave way to it has its room again
                    let gone = on[i - 1];
                    cuts.retain(|&(x, y, rx, ry)| {
                        if x != gone && y != gone {
                            return true;
                        }
                        if !given_back.contains(&(x, y)) {
                            given_back.push((x, y));
                            discs[x].r = discs[x].r.max(rx);
                            discs[y].r = discs[y].r.max(ry);
                        }
                        false
                    });
                    on.remove(i - 1);
                    place.remove(i - 1);
                    continue;
                }
                Err((i, kind)) => {
                    // circles i and i + 1 of the chain have no tangent; 0 is the start and
                    // `on.len() + 1` the end. Where two discs take more room than there is
                    // between their vertices, the passage is over-full: both give way in
                    // proportion, the string passes square to the line of centres, and the
                    // check reports what is missing.
                    if !self.disc_local {
                        info.why = kind;
                        info.fallback = 1;
                        return None;
                    }
                    let a = (i >= 1).then(|| on[i - 1]);
                    let b = (i < on.len()).then(|| on[i]);
                    if let Some(t) = trace {
                        let name = |x: Option<usize>| {
                            x.map_or("end".to_string(), |x| format!("{} r {:.4}", discs[x].id, discs[x].r))
                        };
                        let _ = writeln!(t, "  no tangent, kind {kind}: {} | {}", name(a), name(b));
                    }
                    info.gave_way[(kind as usize).clamp(1, 4) - 1] += 1;
                    match (kind, a, b) {
                        (1, None, Some(j)) => discs[j].r = (start.dist(discs[j].c) * 0.999).max(POINT_DISC / 2.0),
                        (1, Some(j), None) => discs[j].r = (end.dist(discs[j].c) * 0.999).max(POINT_DISC / 2.0),
                        (2 | 4, Some(x), Some(y)) => {
                            let small = if discs[x].r <= discs[y].r { i - 1 } else { i };
                            dead[on[small]] = true;
                            on.remove(small);
                            place.remove(small);
                        }
                        (_, Some(x), Some(y)) => {
                            let d = discs[x].c.dist(discs[y].c);
                            let f = d * 0.999 / (discs[x].r + discs[y].r);
                            if f >= 1.0 {
                                info.why = kind;
                                info.fallback = 1;
                                return None;
                            }
                            // both give the same: the tracks of a bundle through the passage
                            // all lose it at the walls and keep their gaps to each other
                            let cut = (discs[x].r + discs[y].r - d) / 2.0 + 1e-6;
                            cuts.push((x, y, discs[x].r, discs[y].r));
                            if discs[x].r.min(discs[y].r) > cut + POINT_DISC {
                                discs[x].r -= cut;
                                discs[y].r -= cut;
                            } else {
                                discs[x].r *= f;
                                discs[y].r *= f;
                            }
                        }
                        _ => {
                            info.why = kind;
                            info.fallback = 1;
                            return None;
                        }
                    }
                    continue;
                }
            };
            let path = flatten(&elems);
            // the straight stretches of the string, before and after each wrap
            let free = |c: Point| Wrap {
                c,
                r: 0.0,
                side: 1.0,
                id: u32::MAX,
            };
            let mut chain = vec![free(start)];
            chain.extend(wraps.iter().copied());
            chain.push(free(end));
            let stretch: Vec<(Point, Point)> = chain
                .windows(2)
                .map(|w| tangent(&w[0], &w[1]).unwrap_or((w[0].c, w[1].c)))
                .collect();
            // where each begins and ends in the drawn string
            let mut from = 0;
            let cut: Vec<(usize, usize)> = stretch
                .iter()
                .map(|&(a, b)| {
                    let i = (from..path.len())
                        .find(|&i| path[i].dist(a) < 1e-7)
                        .unwrap_or(usize::MAX);
                    let j = (i.min(path.len())..path.len())
                        .find(|&j| path[j].dist(b) < 1e-7)
                        .unwrap_or(usize::MAX);
                    if j != usize::MAX {
                        from = j;
                    }
                    (i, j)
                })
                .collect();
            // vertices on one side of the track are wrapped in the order the track passes them:
            // after every one it has left before it reaches this one, before every one it
            // reaches only after it has left this one (a disc taken over lies beyond the
            // triangle between its two crossings: after the corner the track leaves there,
            // before the one it comes to). Between those two lies the part of the string the
            // disc is about: what comes near it elsewhere passes it beyond other vertices.
            let span = |d: &Disc| {
                if d.sure {
                    (2 * d.first, 2 * d.last)
                } else {
                    (d.first + d.last, d.first + d.last)
                }
            };
            let bounds = |i: usize, on: &[usize]| {
                let me = span(&discs[i]);
                let same = |w: usize| discs[w].side == discs[i].side;
                let lo = on
                    .iter()
                    .rposition(|&w| same(w) && span(&discs[w]).1 < me.0)
                    .map_or(0, |x| x + 1);
                let hi = on
                    .iter()
                    .position(|&w| same(w) && span(&discs[w]).0 > me.1)
                    .unwrap_or(on.len());
                if lo <= hi { (lo, hi) } else { (0, on.len()) }
            };
            // the disc the string violates most, with the side it must pass on
            let mut worst: Option<(f64, usize)> = None;
            for i in 0..discs.len() {
                if dead[i] || on.contains(&i) {
                    continue;
                }
                let (lo, hi) = bounds(i, &on);
                let part = match (cut[lo].0, cut[hi].1) {
                    (a, b) if a <= b && b < path.len() => &path[a..=b],
                    _ => &path[..],
                };
                let deficit = disc_deficit(part, &discs[i]);
                if deficit > 1e-6 && worst.is_none_or(|w| deficit > w.0) {
                    worst = Some((deficit, i));
                }
            }
            match worst {
                None => {
                    best = Some(elems);
                    break;
                }
                Some((_, i)) => {
                    // where along the string the disc comes in: at the stretch that passes it
                    // nearest. Next to a wrap two stretches are about as near, and the
                    // shorter string is the one in the right order.
                    let at = (discs[i].first + discs[i].last) as f64 / 2.0;
                    let (lo, hi) = bounds(i, &on);
                    let mut lines: Vec<(f64, usize)> = (lo..=hi)
                        .map(|k| (crate::geom::point_seg_dist(discs[i].c, stretch[k].0, stretch[k].1), k))
                        .collect();
                    lines.sort_by(|x, y| x.0.total_cmp(&y.0));
                    let new = Wrap {
                        c: discs[i].c,
                        r: discs[i].r,
                        side: discs[i].side,
                        id: discs[i].id,
                    };
                    // a place is tried as the string would settle there: a wrap it lifts off
                    // by taking the new one on is dropped with it
                    let mut pick: Option<(f64, usize, Vec<usize>, Vec<usize>)> = None;
                    // the new wrap itself is the one the string turns the wrong way round, at
                    // every place it may come in
                    let mut lifts_itself = true;
                    // only a stretch the disc reaches into can be the one it bends
                    let reach = discs[i].r + 1e-6;
                    let nearest = lines[0].0;
                    for &(far, k) in lines.iter().take(3).filter(|l| l.0 <= reach.max(nearest)) {
                        let mut w = wraps.clone();
                        w.insert(k, new);
                        // indices into `on` of the wraps that lifted off
                        let mut lifted: Vec<usize> = Vec::new();
                        // and of those that lie in the new disc but for a sliver
                        let mut inside: Vec<usize> = Vec::new();
                        let mut live: Vec<Option<usize>> = (0..on.len()).map(Some).collect();
                        live.insert(k, None);
                        let tried = loop {
                            match band_why(start, end, &w) {
                                Err((i, 5)) if i >= 1 && i <= w.len() && live[i - 1].is_some() => {
                                    lifted.push(live[i - 1].unwrap());
                                    w.remove(i - 1);
                                    live.remove(i - 1);
                                }
                                Err((i, 5)) if i >= 1 && i <= w.len() => {
                                    // the new wrap itself turns the wrong way: next to it lies
                                    // a disc of its side whose centre it covers, one that
                                    // looks out of it by a sliver. The string would have to
                                    // leave the large disc for the sliver and come back to
                                    // it; it keeps to the large one.
                                    let p = i - 1;
                                    let poke = |q: usize| {
                                        let x = &w[q];
                                        (live[q].is_some() && x.side == new.side && x.c.dist(new.c) < new.r)
                                            .then(|| x.c.dist(new.c) + x.r - new.r)
                                    };
                                    let before = p.checked_sub(1).and_then(|q| poke(q).map(|x| (x, q)));
                                    let after = (p + 1 < w.len()).then(|| poke(p + 1).map(|x| (x, p + 1))).flatten();
                                    let Some((_, q)) =
                                        [before, after].into_iter().flatten().min_by(|x, y| x.0.total_cmp(&y.0))
                                    else {
                                        break Err((i, 5));
                                    };
                                    lifted.push(live[q].unwrap());
                                    inside.push(live[q].unwrap());
                                    w.remove(q);
                                    live.remove(q);
                                }
                                other => break other,
                            }
                        };
                        if let Some(t) = trace {
                            let _ = writeln!(
                                t,
                                "    place {k}: {:.4} from the stretch, lifts {:?}, {}",
                                far,
                                lifted.iter().map(|&x| discs[on[x]].id).collect::<Vec<_>>(),
                                match &tried {
                                    Ok(e) => format!(
                                        "length {:.3}",
                                        flatten(e).windows(2).map(|p| p[0].dist(p[1])).sum::<f64>()
                                    ),
                                    Err((i, kind)) => format!("no band (circle {i}, kind {kind})"),
                                }
                            );
                        }
                        lifts_itself &= matches!(&tried, Err((_, 5)));
                        if let Ok(e) = tried {
                            let len: f64 = flatten(&e).windows(2).map(|p| p[0].dist(p[1])).sum();
                            if pick.as_ref().is_none_or(|p| len < p.0 - 1e-9) {
                                pick = Some((len, k, lifted, inside));
                            }
                        }
                    }
                    if pick.is_none() && lifts_itself {
                        // put on, it would only lift off again: the string passes inside
                        // the disc, and the check says by how much
                        if let Some(t) = trace {
                            let _ = writeln!(t, "  gives up {}: lifts off at every place", discs[i].id);
                        }
                        dead[i] = true;
                        info.gave_way[4] += 1;
                        continue;
                    }
                    let pos = pick.as_ref().map_or(lines[0].1, |p| p.1);
                    if let Some(t) = trace {
                        let _ = writeln!(
                            t,
                            "  on {} r {:.4} side {:+} at {pos} of {:?}, missing {:.4}",
                            discs[i].id,
                            discs[i].r,
                            discs[i].side,
                            on.iter().map(|&w| discs[w].id).collect::<Vec<_>>(),
                            worst.map_or(0.0, |w| w.0)
                        );
                    }
                    on.insert(pos, i);
                    place.insert(pos, at);
                    if let Some((_, _, lifted, inside)) = pick {
                        for &x in &inside {
                            dead[on[if x >= pos { x + 1 } else { x }]] = true;
                            info.gave_way[1] += 1;
                        }
                        // the indices are those before the insert
                        let mut gone: Vec<usize> = lifted.iter().map(|&x| if x >= pos { x + 1 } else { x }).collect();
                        gone.sort_unstable_by(|a, b| b.cmp(a));
                        for x in gone {
                            on.remove(x);
                            place.remove(x);
                        }
                    }
                }
            }
        }
        if let Some(t) = trace {
            let _ = writeln!(
                t,
                "  {}: wraps {:?}",
                if best.is_some() { "band" } else { "no band" },
                on.iter().map(|&w| (discs[w].id, discs[w].side)).collect::<Vec<_>>()
            );
            for e in best.iter().flatten() {
                let _ = match e {
                    Elem::Line(a, b) => writeln!(t, "  line ({:.3},{:.3}) ({:.3},{:.3})", a.x, a.y, b.x, b.y),
                    Elem::Arc(a, mid, b) => writeln!(
                        t,
                        "  arc ({:.3},{:.3}) ({:.3},{:.3}) ({:.3},{:.3})",
                        a.x, a.y, mid.x, mid.y, b.x, b.y
                    ),
                };
            }
        }
        let Some(b) = best else {
            info.fallback = 1;
            info.refused += 1;
            return None;
        };
        if self.disc_corridor && !self.in_corridor(sid, &flatten(&b)) {
            info.fallback = 2;
            return None;
        }
        info.wraps = on.len();
        info.stack_wraps = on.len();
        Some(b)
    }

    /// Gives the bands of [`Router::disc_band`] the discs they did not know of. The discs of
    /// a track come from the vertices of its own triangles, but a disc can reach into its way
    /// from a vertex none of them touches: a bundle that goes around a vertex two triangles
    /// away. Where the check finds two tracks too close, the outer one takes on the disc the
    /// inner one is pulled against there, a gap wider. Up to `rounds` times; returns the weight
    /// of what the check finds before and after (spots, shorts and crossings fivefold), and
    /// keeps the discs only if they left less. Not part of a route: the relaxation follows the
    /// construction there; `examples/band_stage.rs` measures it.
    pub fn repair_discs(&mut self, rounds: usize) -> (usize, usize) {
        let count = |r: &Router| {
            let geo = r.geometry();
            let v: Vec<_> = crate::check::check(r.board, &geo)
                .into_iter()
                .filter(|v| !v.is_width())
                .collect();
            // a short weighs as five spots, and so does a crossing of two bands: a disc taken
            // on in the wrong place can push a band across its neighbour
            let spots = crate::check::spots(&v);
            spots.len() + 5 * spots.iter().filter(|s| s.short).count() + 5 * r.debug_intersections(usize::MAX).len()
        };
        let before = count(self);
        let mut best = (before, self.extra_discs.clone());
        for _ in 0..rounds {
            let (geo, segs, _) = self.tagged_geometry();
            let found: Vec<_> = crate::check::check(self.board, &geo)
                .into_iter()
                .filter(|v| !v.is_width() && v.what.starts_with("track"))
                .collect();
            let near = |net: NetId, at: Point| -> Option<(usize, f64)> {
                geo.tracks
                    .iter()
                    .enumerate()
                    .filter(|(_, t)| t.net == net && t.pts.len() >= 2)
                    .map(|(i, t)| {
                        let d = t
                            .pts
                            .windows(2)
                            .map(|w| crate::geom::point_seg_dist(at, w[0], w[1]))
                            .fold(f64::INFINITY, f64::min);
                        (i, d)
                    })
                    .min_by(|a, b| a.1.total_cmp(&b.1))
            };
            let dist_to = |i: usize, c: Point| {
                geo.tracks[i]
                    .pts
                    .windows(2)
                    .map(|w| (crate::geom::point_seg_dist(c, w[0], w[1]), orient(w[0], w[1], c)))
                    .min_by(|a, b| a.0.total_cmp(&b.0))
                    .unwrap()
            };
            let mut added = 0;
            for x in &found {
                let Some(other) = x.nets.1 else { continue };
                let (Some((ia, da)), Some((ib, db))) = (near(x.nets.0, x.at), near(other, x.at)) else {
                    continue;
                };
                if da > 1.0 || db > 1.0 || geo.tracks[ia].layer != geo.tracks[ib].layer {
                    continue;
                }
                let (sa, sb) = (segs[ia], segs[ib]);
                let gap = Self::gap(self.board, &self.item_of(sa), &self.item_of(sb)) + self.cfg.margin;
                // the disc of either track nearest to the spot, with the other track outside it
                let mut pick: Option<(f64, u32, Disc, f64, f64)> = None;
                for (inner, i_in, outer, i_out) in [(sa, ia, sb, ib), (sb, ib, sa, ia)] {
                    for d in self.discs(inner) {
                        if d.r <= POINT_DISC {
                            continue;
                        }
                        let (din, _) = dist_to(i_in, d.c);
                        let (dout, side) = dist_to(i_out, d.c);
                        let reach = d.c.dist(x.at);
                        if dout <= din || din > d.r + 0.05 || reach > d.r + gap + 0.2 {
                            continue;
                        }
                        if pick.as_ref().is_none_or(|p| reach < p.0) {
                            pick = Some((
                                reach,
                                outer,
                                d,
                                din.max(d.r) + gap,
                                if side >= 0.0 { 1.0 } else { -1.0 },
                            ));
                        }
                    }
                }
                let Some((_, outer, d, r, side)) = pick else { continue };
                let s = self.segs[outer as usize].as_ref().unwrap();
                let m = &self.sub.m[s.li as usize];
                let k = s
                    .crossings
                    .iter()
                    .enumerate()
                    .map(|(k, c)| {
                        let [u, v] = m.edges[c.e as usize];
                        (
                            k,
                            crate::geom::point_seg_dist(d.c, m.verts[u as usize], m.verts[v as usize]),
                        )
                    })
                    .min_by(|a, b| a.1.total_cmp(&b.1))
                    .map_or(0, |x| x.0);
                let known = self
                    .discs(outer)
                    .iter()
                    .any(|o| o.id == d.id && o.side == side && o.r >= r - 1e-9);
                if known {
                    continue;
                }
                self.extra_discs.entry(outer).or_default().push(Disc {
                    c: d.c,
                    r,
                    side,
                    id: d.id,
                    first: k,
                    last: k,
                    sure: false,
                });
                added += 1;
            }
            if added == 0 {
                break;
            }
            let now = count(self);
            if now < best.0 {
                best = (now, self.extra_discs.clone());
            }
        }
        self.extra_discs = best.1;
        (before, best.0)
    }

    /// Forgets what [`Router::repair_discs`] added.
    pub fn forget_repairs(&mut self) {
        self.extra_discs.clear();
    }

    /// True if the polyline stays in the corridor of seg `sid` (its crossed triangles and the
    /// copper of its own net).
    fn in_corridor(&self, sid: u32, path: &[Point]) -> bool {
        self.corridor_exit(sid, path).is_none()
    }

    /// The first sample of the polyline outside the corridor of seg `sid`.
    fn corridor_exit(&self, sid: u32, path: &[Point]) -> Option<Point> {
        let s = self.segs[sid as usize].as_ref().unwrap();
        let m = &self.sub.m[s.li as usize];
        let mut tris: Vec<u32> = Vec::new();
        for c in &s.crossings {
            tris.push(c.from);
            tris.push(m.other_tri(c.e, c.from));
        }
        let in_tri = |t: u32, p: Point| {
            let v = m.tris[t as usize];
            let (a, b, c) = (m.verts[v[0] as usize], m.verts[v[1] as usize], m.verts[v[2] as usize]);
            orient(a, b, p) >= -1e-9 && orient(b, c, p) >= -1e-9 && orient(c, a, p) >= -1e-9
        };
        let own = |p: Point| {
            self.sub.shared.obs_index.query_point(p, 0.0).iter().any(|&o| {
                let ob = &self.board.obstacles[o as usize];
                ob.net == Some(s.net)
                    && !matches!(ob.kind, ObstacleKind::Keepout { .. })
                    && self.sub.shared.obs_polys[o as usize].contains(p)
            })
        };
        path.windows(2).find_map(|w| {
            let n = ((w[0].dist(w[1]) / 0.05).ceil() as usize).clamp(1, 200);
            (0..=n)
                .map(|k| w[0].lerp(w[1], k as f64 / n as f64))
                .find(|&p| !(tris.iter().any(|&t| in_tri(t, p)) || own(p)))
        })
    }

    pub fn geometry(&self) -> Geometry {
        if self.soft_on {
            return self.soft_geometry();
        }
        self.tagged_geometry().0
    }

    /// The geometry of a router without soft copper, with the segment of each track and the
    /// site of each via.
    pub(crate) fn tagged_geometry(&self) -> (Geometry, Vec<u32>, Vec<u32>) {
        self.tagged_from(0)
    }

    /// The same with only the tracks of the segments from `first` on.
    pub(crate) fn tagged_from(&self, first: u32) -> (Geometry, Vec<u32>, Vec<u32>) {
        let ids: Vec<u32> = (first..self.segs.len() as u32)
            .filter(|&i| self.segs[i as usize].is_some())
            .collect();
        let mut built: Vec<Vec<Track>> = ids.par_iter().map(|&sid| self.seg_tracks(sid)).collect();
        if self.disc_band && self.disc_planar {
            // The band of the discs is not held between the neighbours of its track on every
            // edge, as the band along the edges is. Where two of them cross (a passage with
            // less room than its tracks ask for), both go back to the band along the edges,
            // and so does what crosses that, until nothing crosses.
            let mut plain = vec![false; ids.len()];
            for _ in 0..6 {
                let crossed = crossing_segments(&built);
                let again: Vec<usize> = crossed.into_iter().filter(|&i| !plain[i]).collect();
                if again.is_empty() {
                    break;
                }
                let fresh: Vec<Vec<Track>> = again.par_iter().map(|&i| self.seg_tracks_with(ids[i], false)).collect();
                for (i, t) in again.into_iter().zip(fresh) {
                    plain[i] = true;
                    built[i] = t;
                }
            }
        }
        let mut vias: Vec<(Via, u32)> = self
            .vias
            .iter()
            .filter_map(|(&v, info)| Some((self.placed_via(v, info)?, v)))
            .collect();
        vias.sort_by(|a, b| a.0.at.x.total_cmp(&b.0.at.x).then(a.0.at.y.total_cmp(&b.0.at.y)));
        let (vias, sites): (Vec<Via>, Vec<u32>) = vias.into_iter().unzip();
        let (tracks, segs): (Vec<Track>, Vec<u32>) = built
            .into_iter()
            .zip(&ids)
            .flat_map(|(t, &sid)| t.into_iter().map(move |t| (t, sid)))
            .unzip();
        (Geometry { tracks, vias }, segs, sites)
    }

    /// The tracks of a routed segment: its band without the stretches on kept copper.
    pub(crate) fn seg_tracks(&self, sid: u32) -> Vec<Track> {
        self.seg_tracks_with(sid, self.disc_band)
    }

    fn seg_tracks_with(&self, sid: u32, disc: bool) -> Vec<Track> {
        let mut tracks = self.seg_pieces(sid, disc);
        tracks.retain(|t| t.pts.len() >= 2);
        tracks
    }

    fn seg_pieces(&self, sid: u32, disc: bool) -> Vec<Track> {
        {
            {
                let s = self.segs[sid as usize].as_ref().unwrap();
                let mut elems = self.seg_path_info_with(sid, disc).0;
                if self.board.class_of(s.net).min_width.is_some() && !s.necks.is_empty() {
                    elems = elems
                        .into_iter()
                        .flat_map(|e| match e {
                            Elem::Line(a, b) => {
                                let mut pts = vec![a];
                                pts.extend(zone_cuts(a, b, &s.necks, s.width, neck_reach(s.neck_w, s.clearance)));
                                pts.push(b);
                                pts.windows(2).map(|w| Elem::Line(w[0], w[1])).collect()
                            }
                            e => vec![e],
                        })
                        .collect();
                }
                let pts = flatten(&elems);
                let layer = self.sub.layers[s.li as usize];
                // the stretches on kept copper of the net need no new track
                let on_kept = |p: Point| {
                    self.sub.shared.obs_index.query_point(p, 0.0).into_iter().any(|o| {
                        let o = &self.board.obstacles[o as usize];
                        o.net == Some(s.net)
                            && o.layers.has(layer)
                            && matches!(o.kind, ObstacleKind::Track | ObstacleKind::Via)
                            && o.shape.dist_to_point(p) <= 0.0
                    })
                };
                let track = |pts: Vec<Point>, elems: Vec<Elem>| Track {
                    seg: sid,
                    tap: match s.end {
                        End::Tap(on, _) => Some(on),
                        _ => None,
                    },
                    pts,
                    elems,
                    width: s.width,
                    layer,
                    net: s.net,
                    necks: s.necks.clone(),
                    neck_w: s.neck_w,
                    origin: None,
                    resting: false,
                    fitted: Vec::new(),
                    pattern: None,
                };
                let runs = if self.has_kept { off_kept(&pts, on_kept) } else { None };
                match runs {
                    None => vec![track(pts, elems)],
                    Some(runs) => runs
                        .into_iter()
                        .map(|run| {
                            let e = run.windows(2).map(|w| Elem::Line(w[0], w[1])).collect();
                            track(run, e)
                        })
                        .collect(),
                }
            }
        }
    }
}

/// The segments (indices into `built`) whose tracks cross a track of another net on their
/// layer.
fn crossing_segments(built: &[Vec<Track>]) -> Vec<usize> {
    let flat: Vec<(usize, &Track, Point, Point)> = built
        .iter()
        .enumerate()
        .flat_map(|(i, ts)| ts.iter().map(move |t| (i, t)))
        .map(|(i, t)| {
            let lo = t
                .pts
                .iter()
                .fold(pt(f64::INFINITY, f64::INFINITY), |a, p| pt(a.x.min(p.x), a.y.min(p.y)));
            let hi = t.pts.iter().fold(pt(f64::NEG_INFINITY, f64::NEG_INFINITY), |a, p| {
                pt(a.x.max(p.x), a.y.max(p.y))
            });
            (i, t, lo, hi)
        })
        .collect();
    let hits: Vec<(usize, usize)> = (0..flat.len())
        .into_par_iter()
        .flat_map_iter(|a| {
            let (ia, ta, alo, ahi) = flat[a];
            let flat = &flat;
            (a + 1..flat.len()).filter_map(move |b| {
                let (ib, tb, blo, bhi) = flat[b];
                if ta.net == tb.net
                    || ta.layer != tb.layer
                    || alo.x > bhi.x
                    || blo.x > ahi.x
                    || alo.y > bhi.y
                    || blo.y > ahi.y
                {
                    return None;
                }
                let hit = ta.pts.windows(2).any(|p| {
                    let (plo, phi) = (
                        pt(p[0].x.min(p[1].x), p[0].y.min(p[1].y)),
                        pt(p[0].x.max(p[1].x), p[0].y.max(p[1].y)),
                    );
                    if plo.x > bhi.x || blo.x > phi.x || plo.y > bhi.y || blo.y > phi.y {
                        return false;
                    }
                    tb.pts
                        .windows(2)
                        .any(|q| crate::geom::segs_intersect(p[0], p[1], q[0], q[1]))
                });
                hit.then_some((ia, ib))
            })
        })
        .collect();
    let mut out: Vec<usize> = hits.into_iter().flat_map(|(a, b)| [a, b]).collect();
    out.sort_unstable();
    out.dedup();
    out
}

/// The pieces of a route `pts` off the kept copper of its net (`on_kept`), each with one point
/// on the copper at either end where it meets it; None if no point lies on kept copper. A
/// straight piece from one piece of kept copper to another is new copper between them: a point
/// of it off the copper keeps it.
fn off_kept(pts: &[Point], on_kept: impl Fn(Point) -> bool) -> Option<Vec<Vec<Point>>> {
    let mut with = Vec::with_capacity(pts.len());
    for (i, &p) in pts.iter().enumerate() {
        if i > 0 && on_kept(pts[i - 1]) && on_kept(p) {
            let a = pts[i - 1];
            let n = ((a.dist(p) / 0.05).ceil() as usize).clamp(2, 200);
            if let Some(q) = (1..n).map(|k| a.lerp(p, k as f64 / n as f64)).find(|&q| !on_kept(q)) {
                with.push(q);
            }
        }
        with.push(p);
    }
    let pts = with;
    let kept: Vec<bool> = pts.iter().map(|&p| on_kept(p)).collect();
    if !kept.iter().any(|&k| k) {
        return None;
    }
    // runs of points off the copper, with one point on it at each end
    let mut out = Vec::new();
    let mut run: Vec<Point> = Vec::new();
    for (i, &p) in pts.iter().enumerate() {
        let free = !kept[i] || (i + 1 < pts.len() && !kept[i + 1]) || (i > 0 && !kept[i - 1]);
        if free {
            run.push(p);
        }
        if !free || (kept[i] && i > 0 && !kept[i - 1]) {
            if run.len() >= 2 {
                out.push(std::mem::take(&mut run));
            } else {
                run.clear();
            }
        }
    }
    if run.len() >= 2 {
        out.push(run);
    }
    Some(out)
}

/// How far beyond its zone a neck reaches: as far as a narrow track stands off a corner on the
/// border of the zone, so that the track can round such a corner before it widens.
pub fn neck_reach(neck_w: f64, clearance: f64) -> f64 {
    clearance + neck_w / 2.0
}

/// Where the segment a-b, on its way into or out of one of the zones, leaves the neck of a
/// track of `width`: `reach` beyond the zone, then half the width and a micrometre (in order
/// from a). Cut there, a line keeps its width up to its neck, and its round end stays out of
/// it. Under a minimum width rule a piece may be narrow if it touches the courtyard, the way
/// KiCad's rule sees it: only a line that touches the zone itself is cut, and the piece from
/// the zone to the cut is the narrow one.
pub fn zone_cuts(a: Point, b: Point, zones: &[crate::geom::Rect], width: f64, reach: f64) -> Vec<Point> {
    const OUT: f64 = 1e-3;
    let len = a.dist(b);
    if len < 4.0 * OUT {
        return Vec::new();
    }
    let mut ts: Vec<f64> = Vec::new();
    for r in zones.iter().filter(|r| r.touches_seg(a, b)) {
        if let Some((t0, t1)) = r.grow(reach + width / 2.0 + OUT).clip_seg(a, b) {
            ts.push(t0);
            ts.push(t1);
        }
    }
    ts.retain(|&t| t * len > OUT && (1.0 - t) * len > OUT);
    ts.sort_by(f64::total_cmp);
    ts.dedup_by(|x, y| (*x - *y).abs() * len < 1e-6);
    ts.into_iter().map(|t| a + (b - a) * t).collect()
}

/// A polyline with the cuts of [`zone_cuts`] in its pieces.
pub fn cut_at_zones(pts: &[Point], zones: &[crate::geom::Rect], width: f64, reach: f64) -> Vec<Point> {
    let mut out: Vec<Point> = Vec::with_capacity(pts.len());
    for (i, &p) in pts.iter().enumerate() {
        if i > 0 {
            out.extend(zone_cuts(pts[i - 1], p, zones, width, reach));
        }
        out.push(p);
    }
    out
}

/// Tangent path from `start` around the wraps to `end`, with arcs on the wraps.
fn band(start: Point, end: Point, wraps: &[Wrap]) -> Option<Vec<Elem>> {
    band_why(start, end, wraps).ok()
}

/// [`band`], or why there is none: the index of the first circle of the pair without a tangent
/// (or of the wrap that turns too far) and the kind. Circles count from 0, the start. Kinds: 1
/// an end of the track or a corner lies inside the circle next to it, 2 two wraps on the same
/// side, one circle inside the other, 3 two wraps on opposite sides that overlap, 4 the same
/// centre, 5 a wrap of more than 270 degrees.
fn band_why(start: Point, end: Point, wraps: &[Wrap]) -> Result<Vec<Elem>, (usize, u8)> {
    let mut circles = vec![Wrap {
        c: start,
        r: 0.0,
        side: 1.0,
        id: u32::MAX,
    }];
    circles.extend_from_slice(wraps);
    circles.push(Wrap {
        c: end,
        r: 0.0,
        side: 1.0,
        id: u32::MAX,
    });
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
                // arc around c from prev_b to a, turning to c.side
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
                if sweep.abs() > std::f64::consts::PI * 1.5 {
                    return Err((i, 5));
                }
                let mid = c.c + pt(c.r, 0.0).rotate(a0 + sweep / 2.0);
                if sweep.abs() * c.r > 0.002 {
                    out.push(Elem::Arc(prev_b, mid, a));
                } else {
                    out.push(Elem::Line(prev_b, a));
                }
            }
        }
        if a.dist(b) > 1e-6 {
            out.push(Elem::Line(a, b));
        }
    }
    if out.is_empty() {
        out.push(Elem::Line(start, end));
    }
    Ok(out)
}

/// The first portal between the ends that the path does not pass through.
///
/// On a side the track is attached to ([`Side::attached`]), the portal reaches from its point
/// down to the radius of the stack. The point lies farther out where the edge runs along an
/// obstacle side, by the factor that side would ask for if it went on for ever; around a
/// rounded pad corner, whose sides are 50 µm long, a band at the right distance from every
/// vertex crosses the edge below that point. No neighbour lies between the track and the
/// vertex it is attached to, so the longer portal is still disjoint from those of all others.
///
/// `strict`: a portal also counts as missed when the path crosses its edge anywhere else than
/// in the portals the track has on that edge. A chain of arcs and corners can pass through a
/// portal and still swing across the same edge beyond the track next to it and back; only a
/// path that crosses each of its edges in its own place there crosses no neighbour.
fn missed_portal(portals: &[(Side, Side)], path: &[Point], strict: bool) -> Option<usize> {
    let reach = |s: &Side| {
        let d = s.p.dist(s.pivot);
        if s.attached && s.rad < d {
            s.pivot.lerp(s.p, s.rad / d)
        } else {
            s.p
        }
    };
    let inner = &portals[1..portals.len() - 1];
    (1..portals.len() - 1).find(|&i| {
        let (l, r) = (reach(&portals[i].0), reach(&portals[i].1));
        let passes = path.windows(2).any(|w| {
            crate::geom::segs_intersect(w[0], w[1], l, r)
                || crate::geom::point_seg_dist(l, w[0], w[1]) < 1e-6
                || crate::geom::point_seg_dist(r, w[0], w[1]) < 1e-6
        });
        if !passes || !strict {
            return !passes;
        }
        let (a, b) = portals[i];
        let len = a.pivot.dist(b.pivot);
        if len < 1e-9 {
            return false;
        }
        let dir = (b.pivot - a.pivot) * (1.0 / len);
        // the places of the track on this edge, as stretches from the first vertex
        let ids = |x: &Side, y: &Side| (x.pivot_id.min(y.pivot_id), x.pivot_id.max(y.pivot_id));
        let own: Vec<(f64, f64)> = inner
            .iter()
            .filter(|(x, y)| ids(x, y) == ids(&a, &b))
            .map(|(x, y)| {
                let (s, t) = ((reach(x) - a.pivot).dot(dir), (reach(y) - a.pivot).dot(dir));
                (s.min(t) - 1e-6, s.max(t) + 1e-6)
            })
            .collect();
        path.windows(2).any(|w| {
            let (da, db) = (dir.cross(w[0] - a.pivot), dir.cross(w[1] - a.pivot));
            if da * db > 0.0 || da == db {
                return false;
            }
            let at = (w[0].lerp(w[1], da / (da - db)) - a.pivot).dot(dir);
            at > 1e-6 && at < len - 1e-6 && !own.iter().any(|&(s, t)| at >= s && at <= t)
        })
    })
}

/// True if the arc of a wrap stays in the lane of its track: on the vertex side of the line
/// through the far ends of the track's portals, from the portal before the edges of the vertex
/// to the one after them.
///
/// Two tracks that both go straight from portal to portal stay in their lanes and cannot cross.
/// An arc swings out of the lane, and that is right where the track next to it swings out
/// around the same vertex as well. If that one has lost its wrap there and goes straight, an
/// arc beyond the lane crosses it, and both pass through all their portals. A radius that is
/// the need of the stack is what the track has to keep from the vertex anyway. A radius with a
/// share of the slack in it is not, so such an arc has to stay in the lane.
fn in_lane(portals: &[(Side, Side)], w: &Link) -> bool {
    let near = |i: usize| if w.w.side > 0.0 { portals[i].0 } else { portals[i].1 };
    let far = |i: usize| if w.w.side > 0.0 { portals[i].1.p } else { portals[i].0.p };
    let (mut first, mut last) = (w.at, w.to);
    while first > 1 && near(first - 1).pivot_id == w.w.id {
        first -= 1;
    }
    while last + 2 < portals.len() && near(last + 1).pivot_id == w.w.id {
        last += 1;
    }
    (first - 1..=last).all(|i| crate::geom::point_seg_dist(w.w.c, far(i), far(i + 1)) >= w.w.r - 1e-6)
}

/// A circle in the chain of [`stack_band`]: a wrap around a vertex, or a corner (radius 0).
#[derive(Clone)]
struct Link {
    /// The first and the last portal it stands for.
    at: usize,
    to: usize,
    w: Wrap,
    /// The corners of the funnel it stands for, with their portals.
    pts: Vec<(usize, Point)>,
    /// Its radius is the need of the stack.
    stack: bool,
}

/// Vertex id of a corner in the chain.
const CORNER: u32 = u32::MAX - 1;

/// The band of a track through its `portals` (start, one per crossing, end) with the `bends`
/// of the funnel, built so that one wrap that cannot be kept does not cost the others their
/// arcs. `leaves` gives the first point of a path outside the triangles of the track.
///
/// - A wrap around a vertex the track lies next to has the radius the stack needs there
///   ([`Side::rad`]); any other wrap reaches to its portal point, as the share of the slack in
///   that point is what keeps neighbours in order. A bend around a vertex that asks for no
///   distance is a corner at the portal point.
/// - Where two circles have no tangent, the wider one becomes the corners of the funnel it
///   stands for: points of its own portals. A wrap with the radius of the stack that turns
///   the wrong way, or one inside such a circle on the same side, holds nothing and goes
///   without a corner.
/// - Every vertex the band then passes too closely becomes a further wrap, if the chain has
///   its tangents with it.
/// - Where the band misses one of its portals or leaves its triangles, the wrap nearest to
///   that place becomes corners, and the tests run again.
/// - A wrap whose radius is a share of the slack keeps its arc only inside the lane of the
///   track ([`in_lane`]).
///
/// The band that comes back has passed both tests as a whole: it crosses each of its edges in
/// its portal there and nowhere else ([`missed_portal`], strict), the portals are disjoint
/// from those of its neighbours, and it stays in its triangles. None
/// if no such band is left: the caller then takes the funnel polyline.
fn stack_band(
    portals: &[(Side, Side)],
    bends: &[(usize, bool)],
    leaves: &dyn Fn(&[Point]) -> Option<Point>,
    info: &mut PathInfo,
) -> Option<Vec<Elem>> {
    let (start, end) = (portals[0].0.p, portals[portals.len() - 1].0.p);
    let side_of = |i: usize, left: bool| if left { portals[i].0 } else { portals[i].1 };
    let sign = |left: bool| if left { 1.0 } else { -1.0 };
    let radius = |s: &Side| if s.attached { s.rad } else { s.p.dist(s.pivot) };
    let chain = |ws: &[Link]| band_why(start, end, &ws.iter().map(|w| w.w).collect::<Vec<_>>());
    // a corner goes in by its portal, unless a corner or an end of the track lies there already
    let put_corner = |ws: &mut Vec<Link>, at: usize, p: Point, side: f64| {
        let pos = ws.iter().position(|x| x.at > at).unwrap_or(ws.len());
        let here = |x: Option<&Link>| x.is_some_and(|x| x.w.r == 0.0 && x.w.c.dist(p) < 1e-9);
        if here(ws.get(pos)) || (pos > 0 && here(ws.get(pos - 1))) || p.dist(start) < 1e-9 || p.dist(end) < 1e-9 {
            return;
        }
        ws.insert(
            pos,
            Link {
                at,
                to: at,
                w: Wrap {
                    c: p,
                    r: 0.0,
                    side,
                    id: CORNER,
                },
                pts: Vec::new(),
                stack: false,
            },
        );
    };
    // wrap k gives way: the corners it stands for take its place, and it does not come back
    let demote = |ws: &mut Vec<Link>, k: usize, banned: &mut Vec<(u32, f64)>| {
        let w = ws.remove(k);
        if w.w.id != CORNER {
            banned.push((w.w.id, w.w.side));
            for (pi, p) in w.pts {
                put_corner(ws, pi, p, w.w.side);
            }
        }
    };
    // wraps give way until the chain has its tangents (`gave`: by reason as in `PathInfo`, then
    // 8 gone without a corner, 9 a wrap of the second step undone)
    let repair = |ws: &mut Vec<Link>, banned: &mut Vec<(u32, f64)>, gave: &mut [usize; 10]| {
        for _ in 0..4 * (ws.len() + bends.len()) + 8 {
            // two wraps of one vertex on one side with nothing left between them are one
            let mut k = 0;
            while k + 1 < ws.len() {
                let (a, b) = (&ws[k].w, &ws[k + 1].w);
                if a.id != CORNER && a.id == b.id && a.side == b.side {
                    let b = ws.remove(k + 1);
                    let a = &mut ws[k];
                    if b.w.r > a.w.r {
                        a.w.r = b.w.r;
                        a.stack = b.stack;
                    }
                    a.to = a.to.max(b.to);
                    a.pts.extend(b.pts);
                } else {
                    k += 1;
                }
            }
            let (i, kind) = match chain(ws) {
                Ok(b) => return Some(b),
                Err(e) => e,
            };
            // wrap k is circle k + 1. The wider circle of the pair goes; of two corners on
            // one spot, one.
            let pair = if kind == 5 { i..=i } else { i..=i + 1 };
            let pair = pair.filter(|&c| c >= 1 && c <= ws.len()).map(|c| c - 1);
            let k = pair.clone().max_by(|&a, &b| ws[a].w.r.total_cmp(&ws[b].w.r))?;
            // A wrap with the radius of the stack that turns the wrong way holds nothing: the
            // band passes its vertex at more than the need. Nor does a wrap inside such a
            // circle on the same side. Those go without a corner.
            let idle = match kind {
                5 if ws[k].stack => Some(k),
                2 if ws[k].stack => pair.min_by(|&a, &b| ws[a].w.r.total_cmp(&ws[b].w.r)),
                _ => None,
            };
            if let Some(k) = idle {
                let w = ws.remove(k);
                banned.push((w.w.id, w.w.side));
                gave[8] += 1;
                continue;
            }
            if ws[k].w.id != CORNER {
                gave[if ws[k].pts.is_empty() { 9 } else { kind as usize - 1 }] += 1;
            }
            demote(ws, k, banned);
        }
        None
    };
    let mut ws: Vec<Link> = Vec::new();
    for &(i, left) in bends {
        let s = side_of(i, left);
        let r = radius(&s);
        if r < 1e-9 {
            info.own += 1;
            put_corner(&mut ws, i, s.p, sign(left));
            continue;
        }
        if let Some(last) = ws.last_mut()
            && last.w.id == s.pivot_id
            && last.w.side == sign(left)
        {
            if r > last.w.r {
                last.w.r = r;
                last.stack = s.attached;
            }
            last.to = i;
            last.pts.push((i, s.p));
            continue;
        }
        ws.push(Link {
            at: i,
            to: i,
            w: Wrap {
                c: s.pivot,
                r,
                side: sign(left),
                id: s.pivot_id,
            },
            pts: vec![(i, s.p)],
            stack: s.attached,
        });
    }
    let mut gave = [0usize; 10];
    let mut banned: Vec<(u32, f64)> = Vec::new();
    let mut best = repair(&mut ws, &mut banned, &mut gave);
    let mut fallback = if best.is_some() { 0 } else { 1 };
    for _ in 0..2 * portals.len() + 2 {
        let Some(elems) = &mut best else { break };
        // every edge end the band passes too closely becomes a further wrap
        for _ in 0..48 {
            let path = flatten(elems);
            let mut worst: Option<(f64, usize, bool)> = None;
            for i in 1..portals.len() - 1 {
                for left in [true, false] {
                    let s = side_of(i, left);
                    let (r, sgn) = (radius(&s), sign(left));
                    if r < 1e-6
                        || banned.contains(&(s.pivot_id, sgn))
                        || ws
                            .iter()
                            .any(|w| w.w.id == s.pivot_id && w.w.side == sgn && w.w.r >= r - 1e-9)
                    {
                        continue;
                    }
                    let d = path
                        .windows(2)
                        .map(|w| crate::geom::point_seg_dist(s.pivot, w[0], w[1]))
                        .fold(f64::INFINITY, f64::min);
                    let deficit = r - d;
                    if deficit > 1e-4 && worst.is_none_or(|w| deficit > w.0) {
                        worst = Some((deficit, i, left));
                    }
                }
            }
            let Some((_, i, left)) = worst else { break };
            let s = side_of(i, left);
            let (r, sgn) = (radius(&s), sign(left));
            let mut next = ws.clone();
            let pos = next.iter().position(|w| w.at > i).unwrap_or(next.len());
            let same = |w: Option<&Link>| w.is_some_and(|w| w.w.id == s.pivot_id && w.w.side == sgn);
            let grows = if pos > 0 && same(next.get(pos - 1)) {
                Some(pos - 1)
            } else if same(next.get(pos)) {
                Some(pos)
            } else {
                None
            };
            match grows {
                // a wrap of this vertex right there grows
                Some(k) => {
                    let w = &mut next[k];
                    w.w.r = r;
                    w.stack = s.attached;
                    w.at = w.at.min(i);
                    w.to = w.to.max(i);
                }
                None => {
                    // a wrap that goes on around its vertex past this portal has the new one
                    // in its middle: it is two wraps then, one before and one after
                    if pos > 0 && next[pos - 1].w.id != CORNER && next[pos - 1].to > i {
                        let mut rest = next[pos - 1].clone();
                        rest.pts.retain(|&(pi, _)| pi > i);
                        rest.at = i + 1;
                        next[pos - 1].pts.retain(|&(pi, _)| pi <= i);
                        next[pos - 1].to = i;
                        next.insert(pos, rest);
                    }
                    next.insert(
                        pos,
                        Link {
                            at: i,
                            to: i,
                            w: Wrap {
                                c: s.pivot,
                                r,
                                side: sgn,
                                id: s.pivot_id,
                            },
                            pts: Vec::new(),
                            stack: s.attached,
                        },
                    );
                }
            }
            match chain(&next) {
                Ok(b) => {
                    ws = next;
                    *elems = b;
                }
                Err(_) => {
                    banned.push((s.pivot_id, sgn));
                    info.refused += 1;
                }
            }
        }
        // the two tests of the whole band; where one fails, the nearest wrap gives way
        let path = flatten(elems);
        let wraps = (0..ws.len()).filter(|&k| ws[k].w.id != CORNER);
        let (k, why) = if let Some(i) = missed_portal(portals, &path, true) {
            fallback = 3;
            let far = |w: &Link| i.saturating_sub(w.to).max(w.at.saturating_sub(i));
            let k = wraps.min_by(|&a, &b| far(&ws[a]).cmp(&far(&ws[b])).then(ws[b].w.r.total_cmp(&ws[a].w.r)));
            (k, 5)
        } else if let Some(p) = leaves(&path) {
            fallback = 2;
            let far = |w: &Link| (w.w.c.dist(p) - w.w.r).abs();
            (wraps.min_by(|&a, &b| far(&ws[a]).total_cmp(&far(&ws[b]))), 6)
        } else if let Some(k) = wraps.clone().find(|&k| !ws[k].stack && !in_lane(portals, &ws[k])) {
            fallback = 3;
            (Some(k), 7)
        } else {
            fallback = 0;
            break;
        };
        // no wrap is left to give way: the chain is the funnel polyline
        let Some(k) = k else { break };
        gave[if ws[k].pts.is_empty() { 9 } else { why }] += 1;
        demote(&mut ws, k, &mut banned);
        best = repair(&mut ws, &mut banned, &mut gave);
        if best.is_none() {
            fallback = 1;
        }
    }
    info.gave_way.copy_from_slice(&gave[..8]);
    info.idle = gave[8];
    info.refused += gave[9];
    info.fallback = fallback;
    if fallback != 0 {
        return None;
    }
    info.wraps = ws.iter().filter(|w| w.w.id != CORNER).count();
    info.stack_wraps = ws.iter().filter(|w| w.w.id != CORNER && w.stack).count();
    info.corners = ws.iter().filter(|w| w.w.id == CORNER).count();
    best
}

pub fn flatten(elems: &[Elem]) -> Vec<Point> {
    let mut pts = Vec::new();
    for e in elems {
        match *e {
            Elem::Line(a, b) => {
                if pts.last() != Some(&a) {
                    pts.push(a);
                }
                pts.push(b);
            }
            Elem::Arc(a, m, b) => {
                let p = arc_polyline(a, m, b, 0.001);
                for q in p {
                    if pts.last() != Some(&q) {
                        pts.push(q);
                    }
                }
            }
        }
    }
    pts
}

/// Tangent from circle a to circle b (signed sides), as (point on a, point on b).
pub(crate) fn tangent(a: &Wrap, b: &Wrap) -> Option<(Point, Point)> {
    let d = b.c - a.c;
    let l = d.len();
    if l < 1e-9 {
        return None;
    }
    let ra = a.side * a.r;
    let rb = b.side * b.r;
    let k = (rb - ra) / l;
    if k.abs() > 1.0 {
        return None;
    }
    let dn = d * (1.0 / l);
    // n: left normal of the travel direction; centre = touch point + n * signed radius
    let n = dn * k + dn.perp() * (1.0 - k * k).sqrt();
    Some((a.c - n * ra, b.c - n * rb))
}

/// Funnel over portals (left, right); returns the bends as (portal index, on left side).
pub fn funnel_bends(portals: &[(Point, Point)]) -> Vec<(usize, bool)> {
    let mut bends = Vec::new();
    let n = portals.len();
    let (mut apex, mut left, mut right) = (portals[0].0, portals[0].0, portals[0].1);
    let (mut left_i, mut right_i) = (0usize, 0usize);
    let mut i = 1;
    while i < n {
        let (l, r) = portals[i];
        // right side: must stay right of (or on) the left boundary
        if orient(apex, right, r) >= 0.0 {
            if apex == right || orient(apex, left, r) < 0.0 {
                right = r;
                right_i = i;
            } else {
                bends.push((left_i, true));
                apex = left;
                let ai = left_i;
                left = apex;
                right = apex;
                left_i = ai;
                right_i = ai;
                i = ai + 1;
                continue;
            }
        }
        if orient(apex, left, l) <= 0.0 {
            if apex == left || orient(apex, right, l) > 0.0 {
                left = l;
                left_i = i;
            } else {
                bends.push((right_i, false));
                apex = right;
                let ai = right_i;
                left = apex;
                right = apex;
                left_i = ai;
                right_i = ai;
                i = ai + 1;
                continue;
            }
        }
        i += 1;
    }
    bends.retain(|&(i, _)| i > 0 && i < n - 1);
    bends
}

/// Shortest path through portals as a polyline.
pub fn funnel(portals: &[(Point, Point)]) -> Vec<Point> {
    let mut path = vec![portals[0].0];
    for (i, left) in funnel_bends(portals) {
        path.push(if left { portals[i].0 } else { portals[i].1 });
    }
    path.push(portals[portals.len() - 1].0);
    path.dedup_by(|a, b| a.dist(*b) < 1e-9);
    path
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lines_on_one_line_with_one_width_become_one_and_corners_widths_and_junctions_stay() {
        let line = |p: &[Point]| -> Vec<Elem> { p.windows(2).map(|w| Elem::Line(w[0], w[1])).collect() };
        let p = vec![pt(0., 0.), pt(1., 0.), pt(2., 0.), pt(3., 0.), pt(3., 1.)];
        // three pieces of one line and a corner: two elements
        let (mut pts, mut elems) = (p.clone(), line(&p));
        assert_eq!(join_lines(&mut pts, &mut elems, &|_, _| 0.2, &|_| false), 2);
        assert_eq!(elems, line(&[p[0], p[3], p[4]]));
        assert_eq!(pts, [p[0], p[3], p[4]]);
        // the piece in the middle is narrower: nothing to join
        let (mut pts, mut elems) = (p.clone(), line(&p));
        let narrow = |a: Point, b: Point| if a.x.min(b.x) == 1.0 && a.y == b.y { 0.15 } else { 0.2 };
        assert_eq!(join_lines(&mut pts, &mut elems, &narrow, &|_| false), 0);
        assert_eq!((pts.len(), elems.len()), (5, 4));
        // something ends at (2, 0): that joint stays
        let (mut pts, mut elems) = (p.clone(), line(&p));
        assert_eq!(join_lines(&mut pts, &mut elems, &|_, _| 0.2, &|j| j == pt(2., 0.)), 1);
        assert_eq!(elems, line(&[p[0], p[2], p[3], p[4]]));
        // a line that turns back on itself is no line, and an arc between two lines parts them
        let back = vec![pt(0., 0.), pt(2., 0.), pt(1., 0.)];
        let (mut pts, mut elems) = (back.clone(), line(&back));
        assert_eq!(join_lines(&mut pts, &mut elems, &|_, _| 0.2, &|_| false), 0);
        let mut elems = vec![
            Elem::Line(pt(0., 0.), pt(1., 0.)),
            Elem::Arc(pt(1., 0.), pt(1.5, 0.5), pt(2., 0.)),
            Elem::Line(pt(2., 0.), pt(3., 0.)),
        ];
        let mut pts = flatten(&elems);
        let n = pts.len();
        assert_eq!(join_lines(&mut pts, &mut elems, &|_, _| 0.2, &|_| false), 0);
        assert_eq!((pts.len(), elems.len()), (n, 3));
        // a slow curve is not straightened piece by piece: every joint taken out stays within
        // a millionth of a mm of the line
        let curve: Vec<Point> = (0..8).map(|i| pt(i as f64, 4e-7 * (i * i) as f64)).collect();
        let (mut pts, mut elems) = (curve.clone(), line(&curve));
        join_lines(&mut pts, &mut elems, &|_, _| 0.2, &|_| false);
        assert!(elems.len() > 1, "{elems:?}");
        for e in &elems {
            let Elem::Line(a, b) = *e else { unreachable!() };
            for q in curve.iter().filter(|q| q.x > a.x && q.x < b.x) {
                assert!(crate::geom::point_seg_dist(*q, a, b) < 1e-6);
            }
        }
    }

    /// A line is cut where a track of its width would start to reach into the neck zone it
    /// enters or leaves, and nowhere else.
    #[test]
    fn lines_are_cut_where_they_leave_a_zone() {
        let mut zone = crate::geom::Rect::empty();
        zone.add(pt(4., 4.));
        zone.add(pt(6., 6.));
        let cuts = |a: (f64, f64), b: (f64, f64)| zone_cuts(pt(a.0, a.1), pt(b.0, b.1), &[zone], 0.75, 0.0);
        let out = cuts((5., 5.), (12., 5.));
        assert!(out.len() == 1 && out[0].dist(pt(6.376, 5.)) < 1e-9, "{out:?}");
        let through = cuts((0., 5.), (10., 5.));
        assert!(
            through.len() == 2 && through[0].dist(pt(3.624, 5.)) < 1e-9 && through[1].dist(pt(6.376, 5.)) < 1e-9,
            "{through:?}"
        );
        assert!(cuts((4.5, 5.), (5.5, 5.)).is_empty());
        assert!(cuts((0., 0.), (1., 1.)).is_empty());
        // a line that passes the zone without entering it keeps its width
        assert!(cuts((0., 6.2), (10., 6.2)).is_empty());
        // the piece into the zone touches it; the next one does not, and neither does its end
        let p = cut_at_zones(&[pt(5., 5.), pt(12., 5.)], &[zone], 0.75, 0.0);
        assert!(
            zone.touches_seg(p[0], p[1]) && zone.dist_seg(p[1], p[2]) > 0.75 / 2.0,
            "{p:?}"
        );
    }

    /// A route keeps its pieces off kept copper and drops those on it; a straight piece from
    /// one piece of kept copper to another is new copper, though both its ends lie on copper.
    #[test]
    fn a_straight_piece_between_two_kept_tracks_stays() {
        let close = |a: Point, b: Point| a.dist(b) < 1e-9;
        let kept = |p: Point| p.x <= 1.0 || p.x >= 9.0;
        let runs = off_kept(&[pt(0., 0.), pt(10., 0.)], kept).unwrap();
        assert_eq!(runs.len(), 1, "{runs:?}");
        assert!(
            close(runs[0][0], pt(0., 0.)) && close(*runs[0].last().unwrap(), pt(10., 0.)),
            "{runs:?}"
        );
        // a route along kept copper only needs nothing new
        assert!(off_kept(&[pt(0., 0.), pt(0.5, 0.)], kept).unwrap().is_empty());
        // off the copper as a whole: no pieces to cut
        assert!(off_kept(&[pt(2., 0.), pt(8., 0.)], kept).is_none());
        // from the copper out to a point: one piece
        let runs = off_kept(&[pt(0., 0.), pt(0.5, 0.), pt(5., 0.)], kept).unwrap();
        assert_eq!(runs, vec![vec![pt(0.5, 0.), pt(5., 0.)]]);
    }

    #[test]
    fn funnel_bends_around_a_corner() {
        // Walk along +x (y up): left is +y, right is -y. A portal forces the path up around x=5.
        let portals = vec![
            (pt(0., 0.), pt(0., 0.)),
            (pt(5., 3.), pt(5., 1.)),
            (pt(10., 0.), pt(10., 0.)),
        ];
        assert_eq!(funnel(&portals), vec![pt(0., 0.), pt(5., 1.), pt(10., 0.)]);
        let portals = vec![
            (pt(0., 0.), pt(0., 0.)),
            (pt(5., 3.), pt(5., -3.)),
            (pt(10., 0.), pt(10., 0.)),
        ];
        assert_eq!(funnel(&portals), vec![pt(0., 0.), pt(10., 0.)]);
        let portals = vec![
            (pt(0., 0.), pt(0., 0.)),
            (pt(5., -1.), pt(5., -3.)),
            (pt(10., 0.), pt(10., 0.)),
        ];
        assert_eq!(funnel(&portals), vec![pt(0., 0.), pt(5., -1.), pt(10., 0.)]);
    }

    #[test]
    fn tangents_wrap_a_circle() {
        // from origin, around a circle at (5,0) r=1 with the centre on the right, to (10,0)
        let a = Wrap {
            c: pt(0., 0.),
            r: 0.0,
            side: 1.0,
            id: 0,
        };
        let c = Wrap {
            c: pt(5., 0.),
            r: 1.0,
            side: -1.0,
            id: 1,
        };
        let (p, q) = tangent(&a, &c).unwrap();
        assert!(p.dist(pt(0., 0.)) < 1e-12);
        assert!((q.dist(pt(5., 0.)) - 1.0).abs() < 1e-12);
        assert!(q.y > 0.0, "band passes above the circle: {q:?}");
        assert!(
            (q - p).dot(q - pt(5., 0.)).abs() < 1e-9,
            "tangent is perpendicular to the radius"
        );
    }

    fn wrap(c: Point, r: f64, side: f64, id: u32) -> Wrap {
        Wrap { c, r, side, id }
    }

    fn end_portal(p: Point) -> (Side, Side) {
        let s = Side {
            p,
            pivot: p,
            pivot_id: u32::MAX,
            rad: 0.0,
            attached: false,
        };
        (s, s)
    }

    /// The portal of a track on an edge from vertex `id` at `pivot` through `through`: the
    /// track lies `lo` from the pivot along the edge and needs `rad` around it; the other end
    /// of the edge is far. `left`: the pivot is on the left of travel.
    fn portal_at(pivot: Point, id: u32, through: Point, lo: f64, rad: Option<f64>, left: bool) -> (Side, Side) {
        let dir = (through - pivot) * (1.0 / through.dist(pivot));
        let near = Side {
            p: pivot + dir * lo,
            pivot,
            pivot_id: id,
            rad: rad.unwrap_or(0.0),
            attached: rad.is_some(),
        };
        let far = Side {
            p: pivot + dir * (lo + 2.0),
            pivot: pivot + dir * (lo + 2.3),
            pivot_id: id + 100,
            rad: 0.0,
            attached: false,
        };
        if left { (near, far) } else { (far, near) }
    }

    /// The point of the line a-b at distance `d` from `c`: the first one on the way, or the
    /// second.
    fn at_dist(c: Point, a: Point, b: Point, d: f64, second: bool) -> Point {
        let v = (b - a) * (1.0 / a.dist(b));
        let foot = (c - a).dot(v);
        let h2 = (c - a).len2() - foot * foot;
        let half = (d * d - h2).sqrt();
        a + v * if second { foot + half } else { foot - half }
    }

    fn ends_of(e: &Elem) -> (Point, Point) {
        match *e {
            Elem::Line(a, b) | Elem::Arc(a, _, b) => (a, b),
        }
    }

    fn arcs(elems: &[Elem]) -> usize {
        elems.iter().filter(|e| matches!(e, Elem::Arc(..))).count()
    }

    fn stack(portals: &[(Side, Side)]) -> (Option<Vec<Elem>>, PathInfo, Vec<(usize, bool)>) {
        let pts: Vec<(Point, Point)> = portals.iter().map(|(l, r)| (l.p, r.p)).collect();
        let bends = funnel_bends(&pts);
        let mut info = PathInfo::default();
        let band = stack_band(portals, &bends, &|_| None, &mut info);
        (band, info, bends)
    }

    /// Two vertices on opposite sides of a track, 1 mm apart. The track needs 0.31 mm around
    /// each, but both of its edges run along a wall and are crossed at a slant, so that the
    /// track lies 0.62 mm from the vertex along the edge. Circles of 0.62 mm overlap and have
    /// no tangent; with the radius of the stack the track has its two arcs, and it still runs
    /// through its portals, because it crosses the slanted edges on its straight pieces.
    #[test]
    fn two_wraps_that_overlap_along_their_edges_keep_their_arcs() {
        let (s, e) = (pt(0., 0.), pt(7., 1.5));
        let (a, b) = (pt(3., 1.), pt(3.8, 0.4));
        let ideal = band(s, e, &[wrap(a, 0.31, -1.0, 1), wrap(b, 0.31, 1.0, 2)]).unwrap();
        let (first, last) = (ends_of(&ideal[0]), ends_of(ideal.last().unwrap()));
        let qa = at_dist(a, first.0, first.1, 0.63, false);
        let qb = at_dist(b, last.0, last.1, 0.63, true);
        let portals = |rad: Option<f64>| {
            vec![
                end_portal(s),
                portal_at(a, 1, qa, 0.62, rad, false),
                portal_at(b, 2, qb, 0.62, rad, true),
                end_portal(e),
            ]
        };
        // the artefact: the radius along the edge
        assert!(band(s, e, &[wrap(a, 0.62, -1.0, 1), wrap(b, 0.62, 1.0, 2)]).is_none());
        let (band, info, bends) = stack(&portals(Some(0.31)));
        assert_eq!(bends, vec![(1, false), (2, true)]);
        let band = band.unwrap();
        assert_eq!(
            (arcs(&band), info.wraps, info.stack_wraps, info.corners),
            (2, 2, 2, 0),
            "{info:?}"
        );
        assert_eq!(missed_portal(&portals(Some(0.31)), &flatten(&band), true), None);
        // with the radius along the edge one of the two gives way for the other, and that one
        // swings out of the lane of the track: both are corners
        let (band, info, _) = stack(&portals(None));
        let band = band.unwrap();
        assert_eq!((arcs(&band), info.wraps, info.corners), (0, 0, 2), "{info:?}");
        assert_eq!(info.gave_way, [0, 0, 1, 0, 0, 0, 0, 1]);
        assert_eq!(missed_portal(&portals(None), &flatten(&band), true), None);
    }

    /// Two vertices of one rounded pad corner, 0.05 mm apart, on the same side of the track.
    /// Their edges meet the pad side at different angles, so the track lies 0.3205 mm from one
    /// and 0.385 mm from the other along the edges: one circle inside the other, no tangent.
    /// Around both the track needs the same 0.31 mm, and with that it turns around the corner
    /// in arcs.
    #[test]
    fn a_rounded_corner_with_two_slants_is_turned_in_arcs() {
        let (s, e) = (pt(0., 0.), pt(5., -2.));
        let (a, b) = (pt(3., 1.), pt(3.04, 0.97));
        let ideal = band(s, e, &[wrap(a, 0.31, -1.0, 1), wrap(b, 0.31, -1.0, 2)]).unwrap();
        let (first, last) = (ends_of(&ideal[0]), ends_of(ideal.last().unwrap()));
        let qa = at_dist(a, first.0, first.1, 0.3215, false);
        let qb = at_dist(b, last.0, last.1, 0.386, true);
        let portals = vec![
            end_portal(s),
            portal_at(a, 1, qa, 0.3205, Some(0.31), false),
            portal_at(b, 2, qb, 0.385, Some(0.31), false),
            end_portal(e),
        ];
        assert!(band(s, e, &[wrap(a, 0.3205, -1.0, 1), wrap(b, 0.385, -1.0, 2)]).is_none());
        let (band, info, _) = stack(&portals);
        let band = band.unwrap();
        assert!(arcs(&band) >= 1, "{band:?}");
        assert_eq!((info.wraps, info.stack_wraps, info.corners), (2, 2, 0), "{info:?}");
        assert_eq!(info.gave_way, [0; 8]);
        assert_eq!(missed_portal(&portals, &flatten(&band), true), None);
        // the band keeps the need from both vertices
        let flat = flatten(&band);
        for c in [a, b] {
            let d = flat
                .windows(2)
                .map(|w| crate::geom::point_seg_dist(c, w[0], w[1]))
                .fold(f64::INFINITY, f64::min);
            assert!(d > 0.31 - 1e-3, "{d}");
        }
    }

    /// A track that turns around one vertex and then ends close to another, on an edge with
    /// slack: its place on that edge is 1.6 mm from the vertex, and the end lies inside that
    /// circle. Only this bend becomes a corner; the arc around the first vertex stays.
    #[test]
    fn an_end_inside_a_circle_costs_only_that_wrap() {
        let (s, e) = (pt(0., 0.), pt(6.5, 0.));
        let a = pt(3., 1.);
        let ideal = band(s, pt(6., 0.6), &[wrap(a, 0.31, -1.0, 1)]).unwrap();
        let first = ends_of(&ideal[0]);
        let qa = at_dist(a, first.0, first.1, 0.63, false);
        let portals = vec![
            end_portal(s),
            portal_at(a, 1, qa, 0.62, Some(0.31), false),
            portal_at(pt(6., -1.), 2, pt(6., 0.), 1.6, None, false),
            end_portal(e),
        ];
        let (band, info, bends) = stack(&portals);
        assert_eq!(bends, vec![(1, false), (2, false)]);
        // the artefact: no band at all for the track
        assert!(super::band(s, e, &[wrap(a, 0.62, -1.0, 1), wrap(pt(6., -1.), 1.6, -1.0, 2)]).is_none());
        let band = band.unwrap();
        assert_eq!((arcs(&band), info.wraps, info.corners), (1, 1, 1), "{info:?}");
        assert_eq!(info.gave_way, [1, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(missed_portal(&portals, &flatten(&band), true), None);
        assert!(
            flatten(&band).iter().any(|p| p.dist(pt(6., 0.6)) < 1e-9),
            "the corner is the point of the portal"
        );
    }

    /// A wrap that takes the band past its portal, and one that takes it out of its triangles,
    /// give way alone: the band that is left passes both tests, and the other wrap keeps its
    /// arc.
    #[test]
    fn a_wrap_that_fails_a_test_gives_way_alone() {
        let (s, e) = (pt(0., 0.), pt(7., 1.5));
        let (a, b) = (pt(3., 1.), pt(3.8, 0.4));
        let ideal = band(s, e, &[wrap(a, 0.31, -1.0, 1), wrap(b, 0.31, 1.0, 2)]).unwrap();
        let (first, last) = (ends_of(&ideal[0]), ends_of(ideal.last().unwrap()));
        let qa = at_dist(a, first.0, first.1, 0.63, false);
        let qb = at_dist(b, last.0, last.1, 0.63, true);
        // a third vertex with two edges: on the first the track lies 0.4 to 1.1 mm from it, on
        // the second, with slack, 1.2 mm. A circle of 1.2 mm misses the place on the first
        let c = pt(6., 0.);
        let on_edge = |deg: f64, lo: f64, hi: f64| {
            let dir = pt(1., 0.).rotate(deg.to_radians());
            let side = |d: f64, id: u32| Side {
                p: c + dir * d,
                pivot: if id == 3 { c } else { c + dir * 3.0 },
                pivot_id: id,
                rad: 0.0,
                attached: false,
            };
            (side(hi, 103), side(lo, 3))
        };
        let portals = vec![
            end_portal(s),
            portal_at(a, 1, qa, 0.62, Some(0.31), false),
            on_edge(120., 0.4, 1.1),
            on_edge(30., 1.2, 1.4),
            end_portal(pt(9., -3.)),
        ];
        let (band, info, bends) = stack(&portals);
        assert_eq!(bends, vec![(1, false), (3, false)]);
        let band = band.unwrap();
        assert_eq!((arcs(&band), info.wraps, info.corners), (1, 1, 1), "{info:?}");
        assert_eq!(info.gave_way, [0, 0, 0, 0, 0, 1, 0, 0]);
        assert_eq!(missed_portal(&portals, &flatten(&band), true), None);
        // the triangles end right above the first vertex: its arc is outside
        let portals = vec![
            end_portal(s),
            portal_at(a, 1, qa, 0.62, Some(0.31), false),
            portal_at(b, 2, qb, 0.62, Some(0.31), true),
            end_portal(e),
        ];
        let pts: Vec<(Point, Point)> = portals.iter().map(|(l, r)| (l.p, r.p)).collect();
        let bends = funnel_bends(&pts);
        let leaves = |path: &[Point]| path.iter().copied().find(|p| p.x < 3.5 && p.y > 1.3);
        let mut info = PathInfo::default();
        let band = stack_band(&portals, &bends, &leaves, &mut info).unwrap();
        assert_eq!((arcs(&band), info.wraps, info.corners), (1, 1, 1), "{info:?}");
        assert_eq!(info.gave_way, [0, 0, 0, 0, 0, 0, 1, 0]);
        assert_eq!(leaves(&flatten(&band)), None);
        assert_eq!(missed_portal(&portals, &flatten(&band), true), None);
    }

    /// A path that runs through its portal and then swings across the same edge twice more,
    /// farther out, has gone around whatever lies between: the strict test says so.
    #[test]
    fn a_band_may_cross_an_edge_only_in_its_portal() {
        let side = |y: f64, pivot: f64, id: u32| Side {
            p: pt(0., y),
            pivot: pt(0., pivot),
            pivot_id: id,
            rad: 0.0,
            attached: false,
        };
        let portals = vec![
            end_portal(pt(-1., 0.1)),
            (side(0.2, 3., 2), side(0., -1., 1)),
            end_portal(pt(2., 2.5)),
        ];
        let straight = [pt(-1., 0.1), pt(2., 2.5)];
        let looped = [
            pt(-1., 0.1),
            pt(1., 0.1),
            pt(1., 2.),
            pt(-1., 2.),
            pt(-1., 2.5),
            pt(2., 2.5),
        ];
        assert_eq!(missed_portal(&portals, &looped, false), None);
        assert_eq!(missed_portal(&portals, &looped, true), Some(1));
        assert_eq!(
            missed_portal(&portals, &[pt(-1., 0.1), pt(1., 0.1), pt(2., 2.5)], true),
            None
        );
        assert_eq!(missed_portal(&portals, &straight, true), Some(1));
    }

    /// Tracks that have to bend around pads on their way: with the radius of the stack every
    /// band runs through all of its portals and stays in its triangles, and no track has
    /// fewer arcs than with the radius along the edge.
    #[test]
    fn the_band_of_the_stack_keeps_to_its_portals_on_a_board() {
        let mut pins = Vec::new();
        for (k, net) in ["A", "B", "C", "D"].into_iter().enumerate() {
            pins.push(("U1", net, 3.0, 8.5 + k as f64));
            pins.push(("U2", net, 17.0, 11.5 - k as f64));
        }
        for (k, net) in ["P", "Q", "R", "S", "T"].into_iter().enumerate() {
            pins.push(("X", net, 7.0 + 1.5 * k as f64, 6.0 + 2.0 * (k % 3) as f64));
        }
        let b = crate::route::tests::pad_board(&pins);
        let sub = crate::topo::Substrate::build(&b, &crate::topo::SubstrateConfig::default());
        let mut r = Router::new(&b, &sub, crate::route::RouterConfig::default());
        r.plan();
        r.run();
        let ids: Vec<u32> = (0..r.segs.len() as u32)
            .filter(|&i| r.segs[i as usize].is_some())
            .collect();
        assert!(ids.len() >= 4, "{}", ids.len());
        let (mut edge, mut stack) = (0, 0);
        for &sid in &ids {
            r.stack_band = false;
            let before = arcs(&r.seg_path(sid));
            r.stack_band = true;
            let (band, info) = r.seg_path_info(sid);
            let flat = flatten(&band);
            if info.fallback == 0 {
                assert_eq!(missed_portal(&r.seg_portals(sid), &flat, true), None, "segment {sid}");
                assert!(r.in_corridor(sid, &flat), "segment {sid}");
            }
            assert!(
                arcs(&band) >= before,
                "segment {sid}: {} < {before}, {info:?}",
                arcs(&band)
            );
            edge += before;
            stack += arcs(&band);
        }
        assert!(stack > 0 && stack >= edge, "{stack} {edge}");
    }

    /// The same board with the band of the discs: every track gets its band, the band keeps
    /// the radius of each of its discs and passes it on its side, and no two bands of
    /// different nets cross.
    #[test]
    fn the_band_of_the_discs_keeps_every_disc_and_crosses_no_other() {
        let mut pins = Vec::new();
        for (k, net) in ["A", "B", "C", "D"].into_iter().enumerate() {
            pins.push(("U1", net, 3.0, 8.5 + k as f64));
            pins.push(("U2", net, 17.0, 11.5 - k as f64));
        }
        for (k, net) in ["P", "Q", "R", "S", "T"].into_iter().enumerate() {
            pins.push(("X", net, 7.0 + 1.5 * k as f64, 6.0 + 2.0 * (k % 3) as f64));
        }
        let b = crate::route::tests::pad_board(&pins);
        let sub = crate::topo::Substrate::build(&b, &crate::topo::SubstrateConfig::default());
        let mut r = Router::new(&b, &sub, crate::route::RouterConfig::default());
        r.plan();
        r.run();
        r.disc_band = true;
        let ids: Vec<u32> = (0..r.segs.len() as u32)
            .filter(|&i| r.segs[i as usize].is_some())
            .collect();
        let mut wraps = 0;
        for &sid in &ids {
            let (band, info) = r.seg_path_info(sid);
            assert_eq!(info.fallback, 0, "segment {sid}: {info:?}");
            assert_eq!(info.gave_way.iter().sum::<usize>(), 0, "segment {sid}: {info:?}");
            let flat = flatten(&band);
            for d in r.discs(sid) {
                let short = disc_deficit(
                    &flat,
                    &Disc {
                        r: d.r.max(POINT_DISC),
                        ..d
                    },
                );
                // an arc is flattened to chords, which lie a little inside it
                assert!(short < 2e-3, "segment {sid}: {short} short of {d:?}");
            }
            wraps += info.wraps;
        }
        assert!(wraps > 0);
        assert!(r.debug_intersections(10).is_empty(), "{:?}", r.debug_intersections(10));
    }
}

impl Router<'_> {
    /// Debug: pairs of segments of different nets on one layer whose bands intersect.
    pub fn debug_intersections(&self, limit: usize) -> Vec<String> {
        let ids: Vec<u32> = (0..self.segs.len() as u32)
            .filter(|&i| self.segs[i as usize].is_some())
            .collect();
        let paths: Vec<Vec<Point>> = ids.iter().map(|&i| flatten(&self.seg_path(i))).collect();
        let mut out = Vec::new();
        for a in 0..ids.len() {
            for b in a + 1..ids.len() {
                let (sa, sb) = (
                    self.segs[ids[a] as usize].as_ref().unwrap(),
                    self.segs[ids[b] as usize].as_ref().unwrap(),
                );
                if sa.net == sb.net || sa.li != sb.li {
                    continue;
                }
                let hit = paths[a].windows(2).find_map(|p| {
                    paths[b]
                        .windows(2)
                        .find(|q| crate::geom::segs_intersect(p[0], p[1], q[0], q[1]))
                        .map(|_| p[0])
                });
                let Some(at) = hit else { continue };
                let common: Vec<String> = sa
                    .crossings
                    .iter()
                    .enumerate()
                    .filter(|(_, c)| sb.crossings.iter().any(|d| d.e == c.e))
                    .map(|(k, c)| {
                        let m = &self.sub.m[sa.li as usize];
                        let l = &self.cross[sa.li as usize][c.e as usize];
                        let ia = l.iter().position(|&x| x == ids[a]).unwrap_or(0);
                        let ib = l.iter().position(|&x| x == ids[b]).unwrap_or(0);
                        let first = m.tris[c.from as usize][m.local_edge(c.from, c.e)];
                        let a_is_u = first == m.edges[c.e as usize][0];
                        let b_right = (ib < ia) == a_is_u;
                        format!("#{k} e{} B {}", c.e, if b_right { "R" } else { "L" })
                    })
                    .collect();
                let mm = &self.sub.m[sa.li as usize];
                let tri_at = (0..mm.tris.len() as u32).find(|&t| {
                    let v = mm.tris[t as usize];
                    let (p, q, r) = (
                        mm.verts[v[0] as usize],
                        mm.verts[v[1] as usize],
                        mm.verts[v[2] as usize],
                    );
                    orient(p, q, at) >= 0.0 && orient(q, r, at) >= 0.0 && orient(r, p, at) >= 0.0
                });
                if let Some(t) = tri_at {
                    let es = mm.tri_edges[t as usize];
                    let ca: Vec<usize> = sa
                        .crossings
                        .iter()
                        .enumerate()
                        .filter(|(_, c)| es.contains(&c.e))
                        .map(|(k, _)| k)
                        .collect();
                    let cb: Vec<usize> = sb
                        .crossings
                        .iter()
                        .enumerate()
                        .filter(|(_, c)| es.contains(&c.e))
                        .map(|(k, _)| k)
                        .collect();
                    out.push(format!(
                        "   tri {t} status {} chords {:?}; A crosses its edges at {:?}, B at {:?}; A ends {:?}/{:?} B ends {:?}/{:?}",
                        mm.status[t as usize], self.chords_debug(sa.li as usize, t), ca, cb, sa.start, sa.end, sb.start, sb.end
                    ));
                }
                out.push(format!(
                    "segs {} ({}) x {} ({}) at {:.3},{:.3} li {}: {} vs {} crossings, common {:?}",
                    ids[a],
                    self.board.nets[sa.net as usize].name,
                    ids[b],
                    self.board.nets[sb.net as usize].name,
                    at.x,
                    at.y,
                    sa.li,
                    sa.crossings.len(),
                    sb.crossings.len(),
                    common
                ));
                if out.len() >= limit {
                    return out;
                }
            }
        }
        out
    }
}

impl Router<'_> {
    /// Debug: tracks too close to vias, with the topological situation.
    pub fn debug_vias(&self, limit: usize) -> Vec<String> {
        let mut out = Vec::new();
        let paths: Vec<(u32, Vec<Point>)> = (0..self.segs.len() as u32)
            .filter(|&i| self.segs[i as usize].is_some())
            .map(|i| (i, flatten(&self.seg_path(i))))
            .collect();
        let mut stats = [0usize; 4];
        for (&site, via) in &self.vias {
            let vp = self.site_point(site);
            for (sid, path) in &paths {
                let s = self.segs[*sid as usize].as_ref().unwrap();
                if s.net == via.net {
                    continue;
                }
                let need = via.r + via.clearance.max(s.clearance) + s.width / 2.0 - 0.005;
                let d = path
                    .windows(2)
                    .map(|w| crate::geom::point_seg_dist(vp, w[0], w[1]))
                    .fold(f64::INFINITY, f64::min);
                if d >= need {
                    continue;
                }
                let m = &self.sub.m[s.li as usize];
                let Some(&v) = m.site_vert.get(&site) else { continue };
                let incident: Vec<usize> = s
                    .crossings
                    .iter()
                    .enumerate()
                    .filter(|(_, c)| m.edges[c.e as usize].contains(&v))
                    .map(|(k, _)| k)
                    .collect();
                let in_fan = s.crossings.iter().any(|c| {
                    m.tris[c.from as usize].contains(&v) || m.tris[m.other_tri(c.e, c.from) as usize].contains(&v)
                });
                let kind = if !incident.is_empty() {
                    0
                } else if in_fan {
                    1
                } else {
                    2
                };
                stats[kind] += 1;
                if out.len() < limit {
                    let portal_d: Vec<String> = incident
                        .iter()
                        .map(|&k| {
                            let (l, r) = self.portal(*sid, k);
                            format!("{:.3}/{:.3}", l.p.dist(vp), r.p.dist(vp))
                        })
                        .collect();
                    out.push(format!(
                        "via site {site} {} at {:.3},{:.3} vs seg {sid} {} li {}: dist {:.3} need {:.3} kind {kind} portals {:?}",
                        self.board.nets[via.net as usize].name, vp.x, vp.y, self.board.nets[s.net as usize].name, s.li, d, need, portal_d
                    ));
                }
            }
        }
        out.push(format!(
            "via violations: incident {} fan-only {} outside-fan {}",
            stats[0], stats[1], stats[2]
        ));
        out
    }
}

impl Router<'_> {
    /// Debug: pairs of vias of different nets closer than allowed.
    pub fn debug_via_pairs(&self) -> Vec<String> {
        let mut out = Vec::new();
        let v: Vec<(&u32, &crate::route::ViaInfo)> = self.vias.iter().collect();
        for i in 0..v.len() {
            for j in i + 1..v.len() {
                let (a, b) = (v[i], v[j]);
                if a.1.net == b.1.net {
                    continue;
                }
                let d = self.site_point(*a.0).dist(self.site_point(*b.0));
                let need = a.1.r + b.1.r + a.1.clearance.max(b.1.clearance);
                if d < need {
                    let conns_a: Vec<usize> = (0..self.conns.len())
                        .filter(|&c| self.conn_vias[c].contains(a.0))
                        .collect();
                    let conns_b: Vec<usize> = (0..self.conns.len())
                        .filter(|&c| self.conn_vias[c].contains(b.0))
                        .collect();
                    out.push(format!(
                        "vias {} ({}) {:?} and {} ({}) {:?}: {:.3} < {:.3}",
                        a.0,
                        self.board.nets[a.1.net as usize].name,
                        conns_a,
                        b.0,
                        self.board.nets[b.1.net as usize].name,
                        conns_b,
                        d,
                        need
                    ));
                }
            }
        }
        out
    }
}

impl Router<'_> {
    /// Debug: segments whose funnel polyline leaves the corridor of triangles they cross.
    pub fn debug_corridors(&self, limit: usize) -> Vec<String> {
        let mut out = Vec::new();
        for sid in 0..self.segs.len() as u32 {
            let Some(s) = self.segs[sid as usize].as_ref() else {
                continue;
            };
            let m = &self.sub.m[s.li as usize];
            let mut tris: Vec<u32> = Vec::new();
            for c in &s.crossings {
                tris.push(c.from);
                tris.push(m.other_tri(c.e, c.from));
            }
            if tris.is_empty() {
                continue;
            }
            let own = |p: Point| {
                self.sub.shared.obs_index.query_point(p, 0.0).iter().any(|&o| {
                    let ob = &self.board.obstacles[o as usize];
                    ob.net == Some(s.net)
                        && !matches!(ob.kind, ObstacleKind::Keepout { .. })
                        && self.sub.shared.obs_polys[o as usize].contains(p)
                })
            };
            let inside = |p: Point| {
                own(p)
                    || tris.iter().any(|&t| {
                        let v = m.tris[t as usize];
                        let (a, b, c) = (m.verts[v[0] as usize], m.verts[v[1] as usize], m.verts[v[2] as usize]);
                        orient(a, b, p) >= -1e-9 && orient(b, c, p) >= -1e-9 && orient(c, a, p) >= -1e-9
                    })
            };
            let path = flatten(&self.seg_path(sid));
            let mut bad = None;
            for w in path.windows(2) {
                for k in 1..20 {
                    let p = w[0].lerp(w[1], k as f64 / 20.0);
                    if !inside(p) {
                        bad = Some(p);
                        break;
                    }
                }
                if bad.is_some() {
                    break;
                }
            }
            if let Some(p) = bad {
                out.push(format!(
                    "seg {sid} {} li {} leaves its corridor at {:.3},{:.3} ({} crossings, start {:?} end {:?})",
                    self.board.nets[s.net as usize].name,
                    s.li,
                    p.x,
                    p.y,
                    s.crossings.len(),
                    s.start,
                    s.end
                ));
                if out.len() >= limit {
                    break;
                }
            }
        }
        out
    }
}
