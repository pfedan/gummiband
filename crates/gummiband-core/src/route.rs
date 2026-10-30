//! Topological router: A* over triangle edges with slot positions. Every route segment on a
//! layer is a chain of edge crossings; inside a free triangle it is a chord between two boundary
//! positions, and no two chords of different nets may interleave. So each layer stays planar
//! and the realization can pull every segment taut without crossings.
//!
//! Each routing layer has its own mesh. Vias sit on via sites inside free triangles; placing
//! one splits the triangle that holds the site in every layer mesh, and the routes that already
//! cross those triangles are threaded through the new ones.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use crate::board::{Board, NetId, ObstacleKind};
use crate::geom::{Point, Rect, orient, point_seg_dist, pt, segs_intersect};
use crate::topo::{Mesh, NONE, OUTSIDE, PAD_SITE, STEINER, Substrate};

#[derive(Debug, Clone)]
pub struct RouterConfig {
    /// Cost of a via in mm of track.
    pub via_cost: f64,
    /// Route plane fan-outs before signal nets.
    pub fanout_first: bool,
    /// Expansion limit per connection search.
    pub max_expansions: usize,
    /// Seed for order perturbation (0 = plain priority order).
    pub seed: u64,
    /// Rip-up and reroute passes for failed connections.
    pub ripup_passes: usize,
    /// Via sites tried per triangle and expansion.
    pub sites_per_tri: usize,
    /// Extra clearance in mm kept in the capacity model (room for the geometry step).
    pub margin: f64,
    /// Allow vias inside SMD pads for plane fan-outs (otherwise only as a last resort).
    pub via_in_pad: bool,
    /// Extra cost (mm) for passing right next to a pad of another net that still waits for its
    /// route: keeps the escape of dense pin rows free.
    pub escape_cost: f64,
    /// Hot spots (centre, extra clearance in mm) where an earlier geometry had violations: the
    /// capacity model asks for more room there, so the topology spreads out.
    pub hotspots: std::sync::Arc<Vec<(Point, f64)>>,
}

impl Default for RouterConfig {
    fn default() -> Self {
        let env = |k: &str, d: f64| std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d);
        RouterConfig {
            via_cost: 3.0,
            fanout_first: true,
            max_expansions: env("GB_MAXEXP", 400_000.0) as usize,
            seed: 0,
            ripup_passes: env("GB_RIPUP", 3.0) as usize,
            sites_per_tri: 3,
            margin: 0.01,
            via_in_pad: false,
            escape_cost: env("GB_ESCAPE", 0.0),
            hotspots: Default::default(),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Goal {
    /// Any of these pins.
    Pins(Vec<usize>),
    /// A via into a plane of the net (or a through hole pin of the net).
    Plane,
}

#[derive(Debug, Clone)]
pub struct Conn {
    pub net: NetId,
    pub from: usize,
    pub goal: Goal,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum End {
    Pin(usize),
    /// A via, by site id.
    Via(u32),
}

/// One crossing of a route segment: edge e, left from triangle `from`.
#[derive(Debug, Clone, Copy)]
pub struct Crossing {
    pub e: u32,
    pub from: u32,
}

#[derive(Debug, Clone)]
pub struct Seg {
    pub net: NetId,
    pub conn: u32,
    pub li: u8,
    pub width: f64,
    pub clearance: f64,
    pub start: End,
    pub end: End,
    pub crossings: Vec<Crossing>,
    /// Track width at each crossing (narrower inside the neck zones).
    pub widths: Vec<f64>,
    /// Free triangles with a chord of this segment.
    pub chord_tris: Vec<u32>,
    /// Zones around the connection's pins where the track may neck down, and the neck width.
    pub necks: Vec<Rect>,
    pub neck_w: f64,
}

/// Position on a triangle boundary: local edge or local vertex.
#[derive(Debug, Clone, Copy, PartialEq)]
enum CPos {
    Edge(u8),
    Vertex(u8),
}

#[derive(Debug, Clone, Copy)]
struct Chord {
    seg: u32,
    a: CPos,
    b: CPos,
}

#[derive(Debug, Clone, Copy)]
pub struct ViaInfo {
    pub net: NetId,
    pub r: f64,
    pub drill: f64,
    pub clearance: f64,
    pub users: u32,
}

/// Mutable routing state. Owns its copy of the meshes, because vias split triangles.
#[derive(Clone)]
pub struct Router<'a> {
    pub board: &'a Board,
    pub sub: Substrate,
    pub cfg: RouterConfig,
    /// [layer][edge] -> segment ids ordered from edges[e][0] to edges[e][1]
    pub cross: Vec<Vec<Vec<u32>>>,
    chords: Vec<HashMap<u32, Vec<Chord>>>,
    pub segs: Vec<Option<Seg>>,
    /// Placed vias by site id.
    pub vias: HashMap<u32, ViaInfo>,
    via_cells: HashMap<(i32, i32), Vec<u32>>,
    pub conns: Vec<Conn>,
    /// Segment ids per connection (empty = unrouted).
    pub conn_segs: Vec<Vec<u32>>,
    pub conn_vias: Vec<Vec<u32>>,
    /// Pins whose pads overlap (same net) share a group: they are connected already.
    pub pin_group: Vec<u32>,
    /// Per pin: number of unrouted connections that start or end there.
    pending: Vec<u32>,
    /// Debugging: positions expanded by the last search of a traced connection.
    pub trace: std::sync::Arc<std::sync::Mutex<Option<Vec<(Point, u8)>>>>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum NK {
    /// In triangle t on layer li, entered through local edge `entry` (3 = none) at `slot`.
    Tri {
        t: u32,
        li: u8,
        entry: u8,
        slot: u16,
    },
    /// At an existing via vertex (by site) on layer li.
    Vert {
        site: u32,
        li: u8,
    },
    /// At a new via on `site`, continuing on layer li.
    Site {
        site: u32,
        li: u8,
    },
    GoalPin {
        pin: usize,
    },
    GoalVia {
        site: u32,
    },
    GoalSite {
        site: u32,
    },
}

#[derive(Debug, Clone, Copy)]
struct Node {
    kind: NK,
    pos: Point,
    g: f64,
    parent: u32,
}

#[derive(PartialEq)]
struct Open(f64, u32);
impl Eq for Open {}
impl PartialOrd for Open {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Open {
    fn cmp(&self, o: &Self) -> Ordering {
        o.0.total_cmp(&self.0).then(o.1.cmp(&self.1))
    }
}

/// Space a crossing needs on an edge.
#[derive(Debug, Clone, Copy)]
pub struct Item {
    pub net: NetId,
    pub w: f64,
    pub c: f64,
    /// Side of the apex of each adjacent triangle (edge_tris order) the track passes:
    /// 0 = towards edges[e][0], 1 = towards edges[e][1], 2 = unknown or not relevant.
    pub sides: [u8; 2],
}

/// An extra obstacle at a triangle apex for capacity checks (a via about to be placed).
#[derive(Debug, Clone, Copy)]
pub struct ExtraApex {
    /// Index into edge_tris[e] of the triangle whose apex it is.
    pub side: usize,
    pub p: Point,
    pub via: ViaInfo,
}

fn key(k: &NK) -> (u8, u32, u32) {
    match *k {
        NK::Tri { t, li, entry, slot } => (0, t, ((li as u32) << 24) | ((entry as u32) << 16) | slot as u32),
        NK::Vert { site, li } => (1, site, li as u32),
        NK::GoalPin { pin } => (2, pin as u32, 0),
        NK::GoalVia { site } => (3, site, 0),
        NK::Site { site, li } => (4, site, li as u32),
        NK::GoalSite { site } => (5, site, 0),
    }
}

impl<'a> Router<'a> {
    pub fn new(board: &'a Board, sub: &Substrate, cfg: RouterConfig) -> Router<'a> {
        Router {
            board,
            sub: sub.clone(),
            cfg,
            cross: sub.m.iter().map(|m| vec![Vec::new(); m.edges.len()]).collect(),
            chords: vec![HashMap::new(); sub.m.len()],
            segs: Vec::new(),
            vias: HashMap::new(),
            via_cells: HashMap::new(),
            conns: Vec::new(),
            conn_segs: Vec::new(),
            conn_vias: Vec::new(),
            trace: Default::default(),
            pin_group: Self::pin_groups(board, sub),
            pending: vec![0; board.pins.len()],
        }
    }

    fn pin_groups(board: &Board, sub: &Substrate) -> Vec<u32> {
        let n = board.pins.len();
        let mut parent: Vec<u32> = (0..n as u32).collect();
        fn find(p: &mut [u32], x: u32) -> u32 {
            let mut r = x;
            while p[r as usize] != r {
                r = p[r as usize];
            }
            let mut y = x;
            while p[y as usize] != r {
                let next = p[y as usize];
                p[y as usize] = r;
                y = next;
            }
            r
        }
        for (pi, pin) in board.pins.iter().enumerate() {
            for &o in &pin.obstacles {
                let poly = &sub.shared.obs_polys[o];
                for q in sub.shared.obs_index.query(&poly.bbox()) {
                    let other = &board.obstacles[q as usize];
                    let Some(pj) = other.pin else { continue };
                    if pj == pi || board.pins[pj].net != pin.net || !other.layers.intersects(board.obstacles[o].layers)
                    {
                        continue;
                    }
                    if poly.dist_to_poly(&sub.shared.obs_polys[q as usize]) <= 0.0 {
                        let (a, b) = (find(&mut parent, pi as u32), find(&mut parent, pj as u32));
                        if a != b {
                            parent[a as usize] = b;
                        }
                    }
                }
            }
        }
        (0..n as u32).map(|x| find(&mut parent, x)).collect()
    }

    fn m(&self, li: usize) -> &Mesh {
        &self.sub.m[li]
    }

    fn nl(&self) -> usize {
        self.sub.m.len()
    }

    pub fn site_point(&self, site: u32) -> Point {
        self.sub.shared.sites[site as usize].p
    }

    // ------------------------------------------------------------------ clearances

    /// Clearance a route of clearance `c` keeps from obstacle `obs`.
    pub fn eff_clearance(&self, obs: u32, c: f64) -> f64 {
        if obs == OUTSIDE {
            return self.board.rules.edge_clearance;
        }
        let o = &self.board.obstacles[obs as usize];
        match o.kind {
            ObstacleKind::Keepout { .. } => crate::board::KEEPOUT_GAP,
            ObstacleKind::Hole => self.board.rules.hole_clearance.max(o.clearance),
            _ => o.clearance.max(c),
        }
    }

    fn obs_net(&self, obs: u32) -> Option<NetId> {
        if obs == OUTSIDE {
            None
        } else {
            self.board.obstacles[obs as usize].net
        }
    }

    /// Via at mesh vertex v of layer li, if any. A via vertex without via (a via that was
    /// ripped up, or a split that was not used) is a phantom: tracks keep their clearance from
    /// the point, so that they never bunch up at a free vertex.
    fn via_at(&self, li: usize, v: u32) -> Option<ViaInfo> {
        let site = self.m(li).vert_site[v as usize];
        if site == NONE {
            return None;
        }
        Some(self.vias.get(&site).copied().unwrap_or(ViaInfo {
            net: NetId::MAX,
            r: 0.0,
            drill: 0.0,
            clearance: 0.0,
            users: 0,
        }))
    }

    fn via_need(via: &ViaInfo, it: &Item) -> f64 {
        if via.net == NetId::MAX {
            // phantom point: tracks on both sides together keep one clearance
            return it.c / 2.0 + it.w / 2.0;
        }
        if via.net == it.net {
            0.0
        } else {
            via.r + via.clearance.max(it.c) + it.w / 2.0
        }
    }

    /// Distance from the edge end vertex to the centre line of a track of `it`.
    fn end_need(&self, li: usize, e: u32, end: usize, it: &Item) -> f64 {
        let m = self.m(li);
        let mut need: f64 = 0.0;
        for d in m.demands(e, end) {
            if self.obs_net(d.obs) == Some(it.net) {
                continue;
            }
            need = need.max((self.eff_clearance(d.obs, it.c) + it.w / 2.0) * d.factor as f64);
        }
        if let Some(via) = self.via_at(li, m.edges[e as usize][end]) {
            need = need.max(Self::via_need(&via, it));
        }
        need
    }

    /// Need of a track at vertex v (vertex demands only), for a new edge.
    fn vertex_need(&self, li: usize, v: u32, it: &Item) -> f64 {
        let m = self.m(li);
        let mut need: f64 = 0.0;
        for &t in &m.vert_tris[v as usize] {
            let o = m.status[t as usize];
            if o == NONE || self.obs_net(o) == Some(it.net) {
                continue;
            }
            need = need.max(self.eff_clearance(o, it.c) + it.w / 2.0);
        }
        if let Some(via) = self.via_at(li, v) {
            need = need.max(Self::via_need(&via, it));
        }
        need
    }

    fn item_of(&self, seg: u32) -> Item {
        let s = self.segs[seg as usize].as_ref().unwrap();
        Item {
            net: s.net,
            w: s.width,
            c: s.clearance,
            sides: [2, 2],
        }
    }

    /// Neck zones and width of connection ci: around its pins a wide track may narrow down.
    /// With a minimum width rule for the class, only inside exempt courtyards.
    pub fn necks(&self, ci: usize) -> (Vec<Rect>, f64) {
        let c = &self.conns[ci];
        let cls = self.board.class_of(c.net);
        let mut pins = vec![c.from];
        if let Goal::Pins(p) = &c.goal {
            pins.extend(p.iter().copied());
        }
        // near pins the track may narrow to the pad width (fine pitch parts), never below the
        // board minimum
        let pad_w = pins
            .iter()
            .map(|&p| self.board.pins[p].size.0.min(self.board.pins[p].size.1))
            .fold(f64::INFINITY, f64::min);
        let neck_w = self
            .board
            .rules
            .min_track_width
            .max(pad_w)
            .min(cls.track_width);
        if neck_w >= cls.track_width - 1e-9 {
            return (Vec::new(), cls.track_width);
        }
        let mut zones = Vec::new();
        for &p in &pins {
            let pin = &self.board.pins[p];
            if cls.min_width.is_some() {
                // custom rules allow the neck where the track touches the part's courtyard
                if !cls.neck_refs.contains(&pin.footprint) {
                    continue;
                }
                if let Some(r) = self.board.courtyards.get(&pin.footprint)
                    && r.contains(pin.pos)
                {
                    zones.push(*r);
                }
            } else {
                // the whole fan-out of the part narrows alike (a zone per pin would leave
                // a narrow track next to a wide one at the pin pitch)
                let mut r = Rect::empty();
                r.add(pin.pos);
                let r = r.grow(pin.size.0.max(pin.size.1) / 2.0 + 0.8);
                zones.push(match self.board.courtyards.get(&pin.footprint) {
                    Some(c) if c.contains(pin.pos) => {
                        let c = c.grow(0.3);
                        Rect {
                            min: pt(c.min.x.min(r.min.x), c.min.y.min(r.min.y)),
                            max: pt(c.max.x.max(r.max.x), c.max.y.max(r.max.y)),
                        }
                    }
                    _ => r,
                });
            }
        }
        (zones, neck_w)
    }

    /// Escape penalty for crossing edge e: an end vertex on a pad of another net that still
    /// has unrouted connections.
    fn escape_penalty(&self, li: usize, e: u32, net: NetId) -> f64 {
        if self.cfg.escape_cost <= 0.0 {
            return 0.0;
        }
        let m = self.m(li);
        for end in 0..2 {
            for d in m.demands(e, end) {
                if d.obs == OUTSIDE {
                    continue;
                }
                let o = &self.board.obstacles[d.obs as usize];
                if let Some(p) = o.pin
                    && o.net != Some(net)
                    && self.pending[p] > 0
                {
                    return self.cfg.escape_cost;
                }
            }
        }
        0.0
    }

    /// Extra room asked for near hot spots, for an edge a-b.
    fn hot_extra(&self, a: Point, b: Point) -> f64 {
        let mut x: f64 = 0.0;
        for &(c, extra) in self.cfg.hotspots.iter() {
            if point_seg_dist(c, a, b) < 1.0 {
                x = x.max(extra);
            }
        }
        x
    }

    /// Width of a track of `full` width with neck zones at edge e.
    fn width_at_edge(&self, li: usize, e: u32, full: f64, necks: &[Rect], neck_w: f64) -> f64 {
        if necks.is_empty() {
            return full;
        }
        let m = self.m(li);
        let [u, v] = m.edges[e as usize];
        let (a, b) = (m.verts[u as usize], m.verts[v as usize]);
        // narrow where the edge touches a zone: the realized track narrows on every piece that
        // touches a zone (that is what intersectsCourtyard() rules allow), and an exit from a
        // fine pitch pad crosses its first edges right next to the pad
        let hit = necks.iter().any(|r| r.touches_seg(a, b));
        if hit { neck_w } else { full }
    }

    /// Item of segment `seg` crossing edge e, with its sides relative to the two apexes.
    fn item_on(&self, li: usize, e: u32, seg: u32) -> Item {
        let mut it = self.item_of(seg);
        {
            let s = self.segs[seg as usize].as_ref().unwrap();
            if let Some(k) = s.crossings.iter().position(|c| c.e == e) {
                it.w = s.widths.get(k).copied().unwrap_or(s.width);
            }
        }
        for (k, &t) in self.m(li).edge_tris[e as usize].iter().enumerate() {
            if t != NONE {
                it.sides[k] = self.chord_side(li, e, t, seg);
            }
        }
        it
    }

    pub fn items_on(&self, li: usize, e: u32) -> Vec<Item> {
        self.cross[li][e as usize]
            .iter()
            .map(|&s| self.item_on(li, e, s))
            .collect()
    }

    /// Side of the apex of t the chord of `seg` in t passes, seen on edge e (see [`Item`]).
    fn chord_side(&self, li: usize, e: u32, t: u32, seg: u32) -> u8 {
        let Some(list) = self.chords[li].get(&t) else { return 2 };
        let Some(ch) = list.iter().find(|c| c.seg == seg) else {
            return 2;
        };
        let ke = self.m(li).local_edge(t, e) as u8;
        let other = if ch.a == CPos::Edge(ke) { ch.b } else { ch.a };
        self.side_of_pos(li, t, e, other)
    }

    /// Side (0/1 = towards edges[e][0]/[1]) for a chord from edge e to `other` in triangle t.
    fn side_of_pos(&self, li: usize, t: u32, e: u32, other: CPos) -> u8 {
        let m = self.m(li);
        let [u, v] = m.edges[e as usize];
        let tv = m.tris[t as usize];
        let x = match other {
            CPos::Edge(k) => {
                let (p, q) = (tv[k as usize], tv[(k as usize + 1) % 3]);
                if p == u || q == u {
                    u
                } else if p == v || q == v {
                    v
                } else {
                    return 2;
                }
            }
            CPos::Vertex(i) => tv[i as usize],
        };
        if x == u {
            0
        } else if x == v {
            1
        } else {
            2
        }
    }

    /// Lower and upper bounds of each item's centre line from the apex obstacles of the two
    /// triangles next to e (an obstacle or via at the apex blocks the middle of the edge).
    fn apex_bounds(&self, li: usize, e: u32, items: &[Item], extra: Option<ExtraApex>) -> Vec<(f64, f64)> {
        let m = self.m(li);
        let mut b = vec![(f64::NEG_INFINITY, f64::INFINITY); items.len()];
        let [u, v] = m.edges[e as usize];
        let pu = m.verts[u as usize];
        let pv = m.verts[v as usize];
        let len = pu.dist(pv);
        if len <= 0.0 {
            return b;
        }
        let dir = (pv - pu) * (1.0 / len);
        let margin = self.cfg.margin;
        for (k, &t) in m.edge_tris[e as usize].iter().enumerate() {
            if t == NONE || m.status[t as usize] != NONE {
                continue;
            }
            let (pw, obs, via_here) = match extra {
                Some(x) if x.side == k => (x.p, Vec::new(), Some(x.via)),
                _ => {
                    let w = *m.tris[t as usize].iter().find(|&&x| x != u && x != v).unwrap();
                    let mut obs: Vec<u32> = Vec::new();
                    for &tt in &m.vert_tris[w as usize] {
                        let o = m.status[tt as usize];
                        if o != NONE && !obs.contains(&o) {
                            obs.push(o);
                        }
                    }
                    (m.verts[w as usize], obs, self.via_at(li, w))
                }
            };
            if obs.is_empty() && via_here.is_none() {
                continue;
            }
            let h = (pw - pu).cross(dir).abs();
            let proj = (pw - pu).dot(dir);
            for (i, it) in items.iter().enumerate() {
                let side = it.sides[k];
                if side > 1 {
                    continue;
                }
                let mut need: f64 = 0.0;
                for &o in &obs {
                    if self.obs_net(o) != Some(it.net) {
                        need = need.max(self.eff_clearance(o, it.c) + it.w / 2.0);
                    }
                }
                if let Some(via) = via_here {
                    need = need.max(Self::via_need(&via, it));
                }
                if need <= 0.0 {
                    continue;
                }
                let need = need + margin;
                if need <= h {
                    continue;
                }
                let half = (need * need - h * h).sqrt();
                if side == 0 {
                    b[i].1 = b[i].1.min(proj - half);
                } else {
                    b[i].0 = b[i].0.max(proj + half);
                }
            }
        }
        b
    }

    fn gap(board: &Board, a: &Item, b: &Item) -> f64 {
        if a.net == b.net {
            return 0.0;
        }
        let mut c = a.c.max(b.c);
        if a.net != NetId::MAX && b.net != NetId::MAX && !board.pair_clearance.is_empty() {
            c = c.max(board.track_clearance(a.net, b.net));
        }
        a.w / 2.0 + c + b.w / 2.0
    }

    /// Pack items along a segment of length `len` with end needs; centre line intervals.
    #[allow(clippy::too_many_arguments)]
    fn pack_with(
        board: &Board,
        items: &[Item],
        len: f64,
        need_u: impl Fn(&Item) -> f64,
        need_v: impl Fn(&Item) -> f64,
        bounds: &[(f64, f64)],
        extra_gap: f64,
    ) -> Option<Vec<(f64, f64)>> {
        let n = items.len();
        let mut lo = vec![0.0; n];
        let mut hi = vec![0.0; n];
        for i in 0..n {
            let mut x = need_u(&items[i]).max(bounds.get(i).map_or(f64::NEG_INFINITY, |b| b.0));
            if i > 0 {
                x = x.max(lo[i - 1] + Self::gap(board, &items[i - 1], &items[i]) + extra_gap);
            }
            lo[i] = x;
        }
        for i in (0..n).rev() {
            let mut y = (len - need_v(&items[i])).min(bounds.get(i).map_or(f64::INFINITY, |b| b.1));
            if i + 1 < n {
                y = y.min(hi[i + 1] - Self::gap(board, &items[i], &items[i + 1]) - extra_gap);
            }
            hi[i] = y;
        }
        if (0..n).any(|i| lo[i] > hi[i] + 1e-9) {
            return None;
        }
        Some(
            lo.into_iter()
                .zip(hi)
                .map(|(a, b)| if a > b { ((a + b) / 2.0, (a + b) / 2.0) } else { (a, b) })
                .collect(),
        )
    }

    /// Centre line intervals of all items on edge e (u to v), or None if they do not fit.
    /// `extra_via` is a via about to be placed at site (vertex of this mesh).
    pub fn pack(
        &self,
        li: usize,
        e: u32,
        items: &[Item],
        extra_via: Option<(u32, &ViaInfo)>,
    ) -> Option<Vec<(f64, f64)>> {
        self.pack_ext(li, e, items, extra_via, None)
    }

    pub fn pack_ext(
        &self,
        li: usize,
        e: u32,
        items: &[Item],
        extra_via: Option<(u32, &ViaInfo)>,
        extra_apex: Option<ExtraApex>,
    ) -> Option<Vec<(f64, f64)>> {
        let m = self.m(li);
        let len = m.edge_len(e);
        let [u, v] = m.edges[e as usize];
        let margin = self.cfg.margin;
        let mut extra_apex = extra_apex;
        let mut via_vertex_here = NONE;
        if let Some((site, via)) = extra_via
            && let Some(&vv) = m.site_vert.get(&site)
        {
            via_vertex_here = vv;
            for (k, &t) in m.edge_tris[e as usize].iter().enumerate() {
                if t != NONE && m.tris[t as usize].contains(&vv) && vv != u && vv != v {
                    extra_apex = Some(ExtraApex {
                        side: k,
                        p: m.verts[vv as usize],
                        via: *via,
                    });
                }
            }
        }
        let extra_end = |vert: u32, it: &Item| -> f64 {
            match extra_via {
                Some((_, via)) if vert == via_vertex_here => Self::via_need(via, it),
                _ => 0.0,
            }
        };
        let bounds = self.apex_bounds(li, e, items, extra_apex);
        let hot = self.hot_extra(m.verts[u as usize], m.verts[v as usize]);
        Self::pack_with(
            self.board,
            items,
            len,
            |it| self.end_need(li, e, 0, it).max(extra_end(u, it)) + margin + hot,
            |it| self.end_need(li, e, 1, it).max(extra_end(v, it)) + margin + hot,
            &bounds,
            hot,
        )
    }

    /// Interval for a new item at slot j of edge e.
    fn slot_interval(&self, li: usize, e: u32, j: usize, it: Item) -> Option<(f64, f64)> {
        let list = &self.cross[li][e as usize];
        let mut items: Vec<Item> = Vec::with_capacity(list.len() + 1);
        for (i, &s) in list.iter().enumerate() {
            if i == j {
                items.push(it);
            }
            items.push(self.item_on(li, e, s));
        }
        if j == list.len() {
            items.push(it);
        }
        self.pack(li, e, &items, None).map(|v| v[j])
    }

    // ------------------------------------------------------------------ chords

    /// Fraction along edge e (from edges[e][0]) of a crossing: the slot of a new crossing, or
    /// the position of `seg` in the list.
    fn edge_frac(&self, li: usize, e: u32, seg: Option<u32>, slot: Option<usize>) -> f64 {
        let list = &self.cross[li][e as usize];
        let n = list.len() as f64;
        match (slot, seg) {
            (Some(j), _) => (j as f64 + 0.5) / (n + 1.0),
            (None, Some(s)) => (list.iter().position(|&x| x == s).unwrap_or(0) as f64 + 1.0) / (n + 1.0),
            _ => 0.5,
        }
    }

    /// Cyclic boundary coordinate of a position in triangle t.
    fn coord(&self, t: u32, li: usize, p: CPos, seg: Option<u32>, slot: Option<usize>) -> f64 {
        match p {
            CPos::Vertex(i) => i as f64,
            CPos::Edge(k) => {
                let m = self.m(li);
                let e = m.tri_edges[t as usize][k as usize];
                let f = self.edge_frac(li, e, seg, slot);
                let forward = m.tris[t as usize][k as usize] == m.edges[e as usize][0];
                k as f64 + if forward { f } else { 1.0 - f }
            }
        }
    }

    /// Approximate point of a boundary position (for geometric chord tests).
    fn pos_point(&self, t: u32, li: usize, p: CPos, seg: Option<u32>, slot: Option<usize>) -> Point {
        let m = self.m(li);
        match p {
            CPos::Vertex(i) => m.verts[m.tris[t as usize][i as usize] as usize],
            CPos::Edge(k) => {
                let e = m.tri_edges[t as usize][k as usize];
                let f = self.edge_frac(li, e, seg, slot);
                let [u, v] = m.edges[e as usize];
                m.verts[u as usize].lerp(m.verts[v as usize], f)
            }
        }
    }

    /// Can a chord of `net` from `a` (slot sa) to `b` (slot sb) be added to triangle t?
    #[allow(clippy::too_many_arguments)]
    fn chord_ok(&self, t: u32, li: usize, net: NetId, a: CPos, sa: Option<usize>, b: CPos, sb: Option<usize>) -> bool {
        let Some(list) = self.chords[li].get(&t) else {
            return true;
        };
        let x = self.coord(t, li, a, None, sa);
        let y = self.coord(t, li, b, None, sb);
        let (lo, hi) = if x < y { (x, y) } else { (y, x) };
        for ch in list {
            if self.segs[ch.seg as usize].as_ref().unwrap().net == net {
                continue;
            }
            let c = self.coord(t, li, ch.a, Some(ch.seg), None);
            let d = self.coord(t, li, ch.b, Some(ch.seg), None);
            let inside = |z: f64| z > lo + 1e-12 && z < hi - 1e-12;
            let on = |z: f64| (z - lo).abs() < 1e-12 || (z - hi).abs() < 1e-12;
            if on(c) || on(d) {
                continue;
            }
            if inside(c) != inside(d) {
                return false;
            }
        }
        true
    }

    /// Can a half chord from interior point `p` to boundary position `b` be added to t?
    fn half_chord_ok(&self, t: u32, li: usize, net: NetId, p: Point, b: CPos, sb: Option<usize>) -> bool {
        let Some(list) = self.chords[li].get(&t) else {
            return true;
        };
        let q = self.pos_point(t, li, b, None, sb);
        list.iter().all(|ch| {
            self.segs[ch.seg as usize].as_ref().unwrap().net == net || {
                let c = self.pos_point(t, li, ch.a, Some(ch.seg), None);
                let d = self.pos_point(t, li, ch.b, Some(ch.seg), None);
                !segs_intersect(p, q, c, d)
            }
        })
    }

    // ------------------------------------------------------------------ vias

    fn via_cell(p: Point) -> (i32, i32) {
        (p.x.floor() as i32, p.y.floor() as i32)
    }

    fn via_size(&self, net: NetId) -> (f64, f64, f64) {
        let c = self.board.class_of(net);
        (
            c.via_diameter / 2.0,
            c.via_drill,
            c.clearance.max(self.board.rules.min_clearance),
        )
    }

    fn via_info(&self, net: NetId) -> ViaInfo {
        let (r, drill, clearance) = self.via_size(net);
        ViaInfo {
            net,
            r,
            drill,
            clearance,
            users: 0,
        }
    }

    /// Static and via spacing checks of a via of `net` at site `site`.
    fn site_clear(&self, site: u32, net: NetId) -> bool {
        let (r, drill, clr) = self.via_size(net);
        let s = &self.sub.shared.sites[site as usize];
        let p = s.p;
        for n in &s.nbrs {
            let o = &self.board.obstacles[n.obs as usize];
            let need = if o.net == Some(net) && o.kind != ObstacleKind::Hole {
                match o.hole {
                    Some(_) => drill / 2.0 + self.board.rules.hole_to_hole,
                    None => 0.0,
                }
            } else {
                r + self.eff_clearance(n.obs, clr) + self.cfg.margin
            };
            if (n.dist as f64) < need {
                return false;
            }
        }
        if self.board.edge_dist(p) < r + self.board.rules.edge_clearance {
            return false;
        }
        let (cx, cy) = Self::via_cell(p);
        for dx in -2..=2 {
            for dy in -2..=2 {
                if let Some(list) = self.via_cells.get(&(cx + dx, cy + dy)) {
                    for &w in list {
                        if w == site {
                            continue;
                        }
                        let o = &self.vias[&w];
                        let d = p.dist(self.site_point(w));
                        let holes = drill / 2.0 + o.drill / 2.0 + self.board.rules.hole_to_hole;
                        let need = if o.net == net {
                            holes
                        } else {
                            (r + o.r + clr.max(o.clearance)).max(holes)
                        };
                        if d < need {
                            return false;
                        }
                    }
                }
            }
        }
        true
    }

    /// Can a via of `net` be used at the existing via vertex of `site`? Some(true) if a via of
    /// the net is already there.
    fn via_ok_vertex(&self, site: u32, net: NetId) -> Option<bool> {
        if let Some(via) = self.vias.get(&site) {
            return if via.net == net { Some(true) } else { None };
        }
        if !self.site_clear(site, net) {
            return None;
        }
        let info = self.via_info(net);
        for li in 0..self.nl() {
            let m = self.m(li);
            // a via in a pad has no vertex on the pad's layer
            let Some(&v) = m.site_vert.get(&site) else { continue };
            for &t in &m.vert_tris[v as usize] {
                for &e in &m.tri_edges[t as usize] {
                    if self.cross[li][e as usize].is_empty() {
                        continue;
                    }
                    let items = self.items_on(li, e);
                    self.pack(li, e, &items, Some((site, &info)))?;
                }
            }
        }
        Some(false)
    }

    /// Can a via of `net` be placed at the unused `site` (splitting its triangles)?
    fn via_ok_site(&self, site: u32, net: NetId) -> bool {
        if !self.sub.site_usable(site) || !self.site_clear(site, net) {
            return false;
        }
        let p = self.site_point(site);
        let via = self.via_info(net);
        for li in 0..self.nl() {
            let m = self.m(li);
            let t = m.site_tri[site as usize];
            if t == PAD_SITE {
                continue;
            }
            let tv = m.tris[t as usize];
            if let Some(list) = self.chords[li].get(&t) {
                // which new edge (s, v_i) each chord crosses and where
                let mut per_edge: [Vec<(f64, Item)>; 3] = [Vec::new(), Vec::new(), Vec::new()];
                for ch in list {
                    let it = self.item_of(ch.seg);
                    for (i, u) in self.rethread_hits(t, li, ch, p) {
                        per_edge[i].push((u, it));
                    }
                }
                for (i, hits) in per_edge.iter_mut().enumerate() {
                    if hits.is_empty() {
                        continue;
                    }
                    // order from v_i (u = 1) towards s (u = 0)
                    hits.sort_by(|a, b| b.0.total_cmp(&a.0));
                    let items: Vec<Item> = hits.iter().map(|h| h.1).collect();
                    let vi = tv[i];
                    let len = p.dist(m.verts[vi as usize]);
                    let margin = self.cfg.margin;
                    let ok = Self::pack_with(
                        self.board,
                        &items,
                        len,
                        |it| self.vertex_need(li, vi, it) + margin,
                        |it| Self::via_need(&via, it) + margin,
                        &[],
                        0.0,
                    );
                    if ok.is_none() {
                        return false;
                    }
                }
            }
            // the old edges get the via as apex of their new triangle
            for k in 0..3 {
                let e = m.tri_edges[t as usize][k];
                if self.cross[li][e as usize].is_empty() {
                    continue;
                }
                let side_idx = m.edge_tris[e as usize].iter().position(|&x| x == t).unwrap();
                let mut items = self.items_on(li, e);
                let chords = self.chords[li].get(&t);
                for (it, &sg) in items.iter_mut().zip(&self.cross[li][e as usize]) {
                    it.sides[side_idx] = 2;
                    let Some(ch) = chords.and_then(|l| l.iter().find(|c| c.seg == sg)) else {
                        continue;
                    };
                    let hits = self.rethread_hits(t, li, ch, p);
                    let at_start = ch.a == CPos::Edge(k as u8);
                    let hit = if at_start { hits.first() } else { hits.last() };
                    let x = match hit {
                        Some(&(i, _)) => tv[i],
                        None => match if at_start { ch.b } else { ch.a } {
                            CPos::Vertex(i) => tv[i as usize],
                            CPos::Edge(_) => continue,
                        },
                    };
                    it.sides[side_idx] = if x == m.edges[e as usize][0] { 0 } else { 1 };
                }
                if self
                    .pack_ext(li, e, &items, None, Some(ExtraApex { side: side_idx, p, via }))
                    .is_none()
                {
                    return false;
                }
            }
        }
        true
    }

    /// Where an existing chord of triangle t crosses the new edges (s, v_i) if t is split at p:
    /// (i, u) with u the position along s -> v_i (0 at s, 1 at v_i), in chord order.
    fn rethread_hits(&self, t: u32, li: usize, ch: &Chord, p: Point) -> Vec<(usize, f64)> {
        let m = self.m(li);
        let a = self.pos_point(t, li, ch.a, Some(ch.seg), None);
        let b = self.pos_point(t, li, ch.b, Some(ch.seg), None);
        let tv = m.tris[t as usize];
        // the ray s -> v_i crosses the chord iff v_i and s lie on opposite sides of it
        let mut os = orient(a, b, p);
        if os == 0.0 {
            os = 1e-300;
        }
        let mut hits = Vec::new();
        for i in 0..3 {
            if ch.a == CPos::Vertex(i as u8) || ch.b == CPos::Vertex(i as u8) {
                continue;
            }
            let vi = m.verts[tv[i] as usize];
            let ov = orient(a, b, vi);
            if os * ov >= 0.0 {
                continue;
            }
            let (tc, u) = line_params(a, b, p, vi);
            hits.push((tc, i, u.clamp(1e-6, 1.0 - 1e-6)));
        }
        hits.sort_by(|x, y| x.0.total_cmp(&y.0));
        hits.into_iter().map(|(_, i, u)| (i, u)).collect()
    }

    /// Place a via vertex at `site` in every layer mesh: split the triangle that holds it and
    /// thread the existing chords through the new triangles. Returns per layer the old triangle
    /// and the new ones X_k (X_k holds old edge k).
    fn split_at(&mut self, site: u32) -> Vec<(u32, [u32; 3])> {
        let p = self.site_point(site);
        let shared = self.sub.shared.clone();
        let mut res = Vec::new();
        for li in 0..self.nl() {
            let t = self.m(li).site_tri[site as usize];
            if t == PAD_SITE {
                // via in pad: no vertex on the pad's own layer
                self.sub.m[li].site_tri[site as usize] = NONE;
                res.push((NONE, [NONE; 3]));
                continue;
            }
            // geometry of the old chords, before the mesh changes
            let mut old: Vec<(Chord, Vec<(usize, f64)>, Point)> = Vec::new();
            if let Some(list) = self.chords[li].remove(&t) {
                for ch in list {
                    let hits = self.rethread_hits(t, li, &ch, p);
                    let a = self.pos_point(t, li, ch.a, Some(ch.seg), None);
                    let b = self.pos_point(t, li, ch.b, Some(ch.seg), None);
                    old.push((ch, hits, a.lerp(b, 1e-6)));
                }
            }
            let old_tv = self.m(li).tris[t as usize];
            let old_edges = self.m(li).tri_edges[t as usize];
            let (_, x) = self.sub.m[li].split(t, site, p, &shared.sites);
            let ne = self.m(li).edges.len();
            self.cross[li].resize(ne, Vec::new());
            let mut on_edge: HashMap<u32, Vec<(f64, u32)>> = HashMap::new();
            for (ch, hits, start_pt) in old {
                let m = &self.sub.m[li];
                let new_edge = |k: usize| -> u32 { m.tri_edges[x[k] as usize][2] }; // E_k = (s, v_k)
                let mut cur = match ch.a {
                    CPos::Edge(k) => x[k as usize],
                    // the sub-triangle that has the chord's vertex and contains its start
                    CPos::Vertex(i) => *x
                        .iter()
                        .filter(|&&xt| m.tris[xt as usize].contains(&old_tv[i as usize]))
                        .find(|&&xt| {
                            let tv = m.tris[xt as usize];
                            let (a, b, c) = (
                                m.verts[tv[0] as usize],
                                m.verts[tv[1] as usize],
                                m.verts[tv[2] as usize],
                            );
                            orient(a, b, start_pt) >= -1e-12
                                && orient(b, c, start_pt) >= -1e-12
                                && orient(c, a, start_pt) >= -1e-12
                        })
                        .or_else(|| x.iter().find(|&&xt| m.tris[xt as usize].contains(&old_tv[i as usize])))
                        .unwrap_or(&x[0]),
                };
                // (0 if the vertex is missing: only after a degenerate rethreading, see below)
                let local_of_vertex =
                    |xt: u32, v: u32| m.tris[xt as usize].iter().position(|&q| q == v).unwrap_or(0) as u8;
                // a chord from vertex to vertex runs along an old edge: it stays in the new
                // triangle that holds both vertices, without crossings
                if let (CPos::Vertex(i), CPos::Vertex(j)) = (ch.a, ch.b)
                    && let Some(&xt) = x.iter().find(|&&xt| {
                        let tv = m.tris[xt as usize];
                        tv.contains(&old_tv[i as usize]) && tv.contains(&old_tv[j as usize])
                    })
                {
                    cur = xt;
                }
                let mut pos = match ch.a {
                    CPos::Edge(_) => CPos::Edge(0),
                    CPos::Vertex(i) => CPos::Vertex(local_of_vertex(cur, old_tv[i as usize])),
                };
                let mut hits = hits;
                if let (CPos::Vertex(_), CPos::Vertex(_)) = (ch.a, ch.b) {
                    hits.clear();
                }
                if hits.len() == 2 && !m.tri_edges[cur as usize].contains(&new_edge(hits[0].0)) {
                    hits.swap(0, 1);
                }
                if hits
                    .iter()
                    .any(|h| !(0..3).any(|k| m.tri_edges[x[k] as usize].contains(&new_edge(h.0))))
                    || hits
                        .first()
                        .is_some_and(|h| !m.tri_edges[cur as usize].contains(&new_edge(h.0)))
                {
                    hits.clear();
                }
                let mut new_cross: Vec<Crossing> = Vec::new();
                let mut new_chords: Vec<(u32, Chord)> = Vec::new();
                for (i, u) in hits {
                    let e = new_edge(i);
                    let k_out = m.local_edge(cur, e) as u8;
                    new_chords.push((
                        cur,
                        Chord {
                            seg: ch.seg,
                            a: pos,
                            b: CPos::Edge(k_out),
                        },
                    ));
                    new_cross.push(Crossing { e, from: cur });
                    on_edge.entry(e).or_default().push((u, ch.seg));
                    let next = m.other_tri(e, cur);
                    pos = CPos::Edge(m.local_edge(next, e) as u8);
                    cur = next;
                }
                let end = match ch.b {
                    CPos::Edge(_) => CPos::Edge(0),
                    CPos::Vertex(i) => CPos::Vertex(local_of_vertex(cur, old_tv[i as usize])),
                };
                new_chords.push((
                    cur,
                    Chord {
                        seg: ch.seg,
                        a: pos,
                        b: end,
                    },
                ));
                // splice the crossings into the segment
                let seg = self.segs[ch.seg as usize].as_mut().unwrap();
                let exit = match ch.b {
                    CPos::Edge(k) => seg
                        .crossings
                        .iter()
                        .position(|c| c.e == old_edges[k as usize] && c.from == t),
                    CPos::Vertex(_) => None,
                };
                match exit {
                    Some(ix) => {
                        seg.crossings[ix].from = cur;
                        let w = seg.widths.get(ix).copied().unwrap_or(seg.width);
                        for (j, c) in new_cross.into_iter().enumerate() {
                            seg.crossings.insert(ix + j, c);
                            seg.widths.insert(ix + j, w);
                        }
                    }
                    None => {
                        let w = seg.widths.last().copied().unwrap_or(seg.width);
                        for c in new_cross {
                            seg.crossings.push(c);
                            seg.widths.push(w);
                        }
                    }
                }
                seg.chord_tris.retain(|&q| q != t);
                for (xt, c) in new_chords {
                    if !seg.chord_tris.contains(&xt) {
                        seg.chord_tris.push(xt);
                    }
                    self.chords[li].entry(xt).or_default().push(c);
                }
            }
            // order on the new edges: from v_i (edges[e][0]) towards s
            for (e, mut list) in on_edge {
                list.sort_by(|a, b| b.0.total_cmp(&a.0));
                self.cross[li][e as usize] = list.into_iter().map(|(_, sg)| sg).collect();
            }
            res.push((t, x));
        }
        res
    }

    fn in_plane(&self, p: Point, net: NetId) -> bool {
        let (r, _, _) = self.via_size(net);
        self.board
            .planes
            .iter()
            .any(|pl| pl.net == net && pl.outline.contains(p) && pl.outline.boundary_dist(p) >= r + 0.3)
    }

    fn add_via(&mut self, site: u32, net: NetId) {
        if let Some(via) = self.vias.get_mut(&site) {
            via.users += 1;
            return;
        }
        let mut info = self.via_info(net);
        info.users = 1;
        self.vias.insert(site, info);
        self.via_cells
            .entry(Self::via_cell(self.site_point(site)))
            .or_default()
            .push(site);
    }

    fn remove_via(&mut self, site: u32) {
        let Some(via) = self.vias.get_mut(&site) else { return };
        via.users -= 1;
        if via.users == 0 {
            self.vias.remove(&site);
            let c = Self::via_cell(self.site_point(site));
            if let Some(list) = self.via_cells.get_mut(&c) {
                list.retain(|&x| x != site);
            }
        }
    }

    // ------------------------------------------------------------------ search

    fn tri_pin(&self, li: usize, t: u32) -> Option<usize> {
        let s = self.m(li).status[t as usize];
        if s == NONE || s == OUTSIDE {
            return None;
        }
        self.board.obstacles[s as usize].pin
    }

    /// A* for one connection. Returns the node chain from start to goal.
    fn search(&self, ci: usize) -> Option<Vec<Node>> {
        let conn = &self.conns[ci];
        let net = conn.net;
        let cls = self.board.class_of(net);
        let item = Item {
            net,
            w: cls.track_width,
            c: cls.clearance.max(self.board.rules.min_clearance),
            sides: [2, 2],
        };
        let (necks, neck_w) = self.necks(ci);
        let src = &self.board.pins[conn.from];
        let target_pts: Vec<Point> = match &conn.goal {
            Goal::Pins(p) => p.iter().map(|&p| self.board.pins[p].pos).collect(),
            Goal::Plane => Vec::new(),
        };
        let h = |p: Point| -> f64 {
            if target_pts.is_empty() {
                0.0
            } else {
                target_pts.iter().map(|&q| q.dist(p)).fold(f64::INFINITY, f64::min)
            }
        };
        let is_target = |pin: usize| -> bool {
            match &conn.goal {
                Goal::Pins(p) => p.iter().any(|&q| self.pin_group[q] == self.pin_group[pin]),
                Goal::Plane => {
                    let pn = &self.board.pins[pin];
                    // a nearby through hole pin of the net reaches the plane as well
                    self.pin_group[pin] != self.pin_group[conn.from]
                        && pn.net == net
                        && pn.pos.dist(src.pos) < 3.0
                        && self
                            .board
                            .planes
                            .iter()
                            .any(|pl| pl.net == net && pn.layers.has(pl.layer) && pl.outline.contains(pn.pos))
                }
            }
        };
        let nl = self.nl();

        let mut nodes: Vec<Node> = Vec::new();
        let mut best: HashMap<(u8, u32, u32), f64> = HashMap::new();
        let mut open = BinaryHeap::new();
        type Best = HashMap<(u8, u32, u32), f64>;
        let push = |best: &mut Best,
                    nodes: &mut Vec<Node>,
                    open: &mut BinaryHeap<Open>,
                    kind: NK,
                    pos: Point,
                    g: f64,
                    parent: u32| {
            let k = key(&kind);
            if let Some(&b) = best.get(&k)
                && b <= g + 1e-9
            {
                return;
            }
            best.insert(k, g);
            nodes.push(Node { kind, pos, g, parent });
            open.push(Open(g + h(pos), (nodes.len() - 1) as u32));
        };

        for (li, m) in self.sub.m.iter().enumerate() {
            if !src.layers.has(m.layer) {
                continue;
            }
            let group = self.pin_group[conn.from];
            let starts: Vec<usize> = (0..self.board.pins.len())
                .filter(|&q| self.pin_group[q] == group && self.board.pins[q].layers.has(m.layer))
                .flat_map(|q| self.board.pins[q].obstacles.iter().copied())
                .collect();
            for &o in &starts {
                for &t in m.obs_tris.get(&(o as u32)).map(|v| v.as_slice()).unwrap_or(&[]) {
                    push(
                        &mut best,
                        &mut nodes,
                        &mut open,
                        NK::Tri {
                            t,
                            li: li as u8,
                            entry: 3,
                            slot: 0,
                        },
                        src.pos,
                        0.0,
                        u32::MAX,
                    );
                }
            }
        }

        // crossings out of triangle t (layer lu) through local edge k
        #[allow(clippy::too_many_arguments)]
        let cross_out = |best: &mut Best,
                         nodes: &mut Vec<Node>,
                         open: &mut BinaryHeap<Open>,
                         t: u32,
                         lu: usize,
                         k: u8,
                         chord_from: Option<(CPos, Option<usize>)>,
                         interior: Option<Point>,
                         node: &Node,
                         idx: u32| {
            let m = self.m(lu);
            let t2 = m.tri_nbrs[t as usize][k as usize];
            if t2 == NONE {
                return;
            }
            let s2 = m.status[t2 as usize];
            if !Substrate::passable(self.board, s2, net) {
                return;
            }
            let free = m.status[t as usize] == NONE;
            let e = m.tri_edges[t as usize][k as usize];
            let k2 = m.local_edge(t2, e) as u8;
            if !free && s2 != NONE {
                push(
                    best,
                    nodes,
                    open,
                    NK::Tri {
                        t: t2,
                        li: lu as u8,
                        entry: 3,
                        slot: 0,
                    },
                    node.pos,
                    node.g,
                    idx,
                );
                return;
            }
            // never enter a triangle this path already passed on this layer (no loops)
            {
                let mut p = idx;
                while p != u32::MAX {
                    match nodes[p as usize].kind {
                        NK::Tri { t: tp, li: lp, .. } if lp as usize == lu => {
                            if tp == t2 {
                                return;
                            }
                        }
                        NK::Tri { .. } => {}
                        _ => break,
                    }
                    p = nodes[p as usize].parent;
                }
            }
            let mut item = item;
            item.w = self.width_at_edge(lu, e, item.w, &necks, neck_w);
            if free && let Some((cp, cs)) = chord_from {
                // the chord in t fixes the side of t's apex on the exit edge ...
                let side_idx = m.edge_tris[e as usize].iter().position(|&x| x == t).unwrap();
                item.sides[side_idx] = self.side_of_pos(lu, t, e, cp);
                // ... and on the entry edge, which must still fit
                if let (CPos::Edge(ke), Some(js)) = (cp, cs) {
                    let e_in = m.tri_edges[t as usize][ke as usize];
                    let mut it_in = item;
                    it_in.sides = [2, 2];
                    it_in.w = self.width_at_edge(lu, e_in, cls.track_width, &necks, neck_w);
                    let si = m.edge_tris[e_in as usize].iter().position(|&x| x == t).unwrap();
                    it_in.sides[si] = self.side_of_pos(lu, t, e_in, CPos::Edge(k));
                    if self.slot_interval(lu, e_in, js, it_in).is_none() {
                        return;
                    }
                }
            }
            let [u, v] = m.edges[e as usize];
            let (pu, pv) = (m.verts[u as usize], m.verts[v as usize]);
            let len = pu.dist(pv);
            let dir = (pv - pu) * (1.0 / len);
            let n = self.cross[lu][e as usize].len();
            let escape = self.escape_penalty(lu, e, net);
            for j in 0..=n {
                if free {
                    if let Some((cp, cs)) = chord_from
                        && !self.chord_ok(t, lu, net, cp, cs, CPos::Edge(k), Some(j))
                    {
                        continue;
                    }
                    if let Some(p) = interior
                        && !self.half_chord_ok(t, lu, net, p, CPos::Edge(k), Some(j))
                    {
                        continue;
                    }
                }
                let Some((lo, hi)) = self.slot_interval(lu, e, j, item) else {
                    continue;
                };
                let q = pu + dir * (node.pos - pu).dot(dir).clamp(lo, hi);
                push(
                    best,
                    nodes,
                    open,
                    NK::Tri {
                        t: t2,
                        li: lu as u8,
                        entry: k2,
                        slot: j as u16,
                    },
                    q,
                    node.g + node.pos.dist(q) + escape,
                    idx,
                );
            }
        };

        let mut expansions = 0;
        while let Some(Open(_, idx)) = open.pop() {
            let node = nodes[idx as usize];
            if best.get(&key(&node.kind)).is_some_and(|&b| b < node.g - 1e-9) {
                continue;
            }
            expansions += 1;
            if expansions > self.cfg.max_expansions {
                return None;
            }
            if let Some(t) = self.trace.lock().unwrap().as_mut() {
                let li = match node.kind {
                    NK::Tri { li, .. } | NK::Vert { li, .. } | NK::Site { li, .. } => li,
                    _ => 0,
                };
                t.push((node.pos, li));
            }
            match node.kind {
                NK::GoalPin { .. } | NK::GoalVia { .. } | NK::GoalSite { .. } => {
                    let mut chain = vec![node];
                    let mut p = node.parent;
                    while p != u32::MAX {
                        chain.push(nodes[p as usize]);
                        p = nodes[p as usize].parent;
                    }
                    chain.reverse();
                    return Some(chain);
                }
                NK::Tri { t, li, entry, slot } => {
                    let lu = li as usize;
                    let m = self.m(lu);
                    let status = m.status[t as usize];
                    let free = status == NONE;
                    let entry_pos = if entry < 3 {
                        Some((CPos::Edge(entry), Some(slot as usize)))
                    } else {
                        None
                    };
                    if let Some(pin) = self.tri_pin(lu, t)
                        && is_target(pin)
                    {
                        let q = self.board.pins[pin].pos;
                        push(
                            &mut best,
                            &mut nodes,
                            &mut open,
                            NK::GoalPin { pin },
                            q,
                            node.g + node.pos.dist(q),
                            idx,
                        );
                        continue;
                    }
                    // via in pad: a plane fan-out may end on a via inside the source pad
                    if !free
                        && self.cfg.via_in_pad
                        && let Some(pin) = self.tri_pin(lu, t)
                        && self.pin_group[pin] == self.pin_group[conn.from]
                        && let Some(list) = self.sub.shared.pad_sites.get(&(pin as u32))
                    {
                        for &site in list {
                            let q = self.site_point(site);
                            if !self.sub.site_usable(site) || !self.via_ok_site(site, net) {
                                continue;
                            }
                            if conn.goal == Goal::Plane {
                                if self.in_plane(q, net) {
                                    push(
                                        &mut best,
                                        &mut nodes,
                                        &mut open,
                                        NK::GoalSite { site },
                                        q,
                                        node.g + node.pos.dist(q) + self.cfg.via_cost,
                                        idx,
                                    );
                                }
                            } else {
                                // leave the pad through a via to the other layers
                                for l2 in 0..nl {
                                    if l2 != lu && self.m(l2).site_tri[site as usize] != PAD_SITE {
                                        push(
                                            &mut best,
                                            &mut nodes,
                                            &mut open,
                                            NK::Site { site, li: l2 as u8 },
                                            q,
                                            node.g + node.pos.dist(q) + self.cfg.via_cost * 2.0,
                                            idx,
                                        );
                                    }
                                }
                            }
                        }
                    }
                    // through hole copper of the net: switch layers for free
                    if !free && status != OUTSIDE && self.board.obstacles[status as usize].hole.is_some() {
                        for l2 in 0..nl {
                            if l2 == lu {
                                continue;
                            }
                            for &t2 in self.m(l2).obs_tris.get(&status).map(|v| v.as_slice()).unwrap_or(&[]) {
                                push(
                                    &mut best,
                                    &mut nodes,
                                    &mut open,
                                    NK::Tri {
                                        t: t2,
                                        li: l2 as u8,
                                        entry: 3,
                                        slot: 0,
                                    },
                                    node.pos,
                                    node.g,
                                    idx,
                                );
                            }
                        }
                    }
                    for k in 0..3u8 {
                        if k != entry {
                            cross_out(&mut best, &mut nodes, &mut open, t, lu, k, entry_pos, None, &node, idx);
                        }
                    }
                    if !free {
                        continue;
                    }
                    // existing via vertices at the corners
                    for i in 0..3u8 {
                        let v = m.tris[t as usize][i as usize];
                        let site = m.vert_site[v as usize];
                        if site == NONE || site == STEINER {
                            continue;
                        }
                        let Some(existing) = self.via_ok_vertex(site, net) else {
                            continue;
                        };
                        if let Some((ep, es)) = entry_pos
                            && !self.chord_ok(t, lu, net, ep, es, CPos::Vertex(i), None)
                        {
                            continue;
                        }
                        let q = m.verts[v as usize];
                        let d = node.pos.dist(q);
                        let vc = if existing { 0.1 } else { self.cfg.via_cost };
                        if conn.goal == Goal::Plane && self.in_plane(q, net) {
                            push(
                                &mut best,
                                &mut nodes,
                                &mut open,
                                NK::GoalVia { site },
                                q,
                                node.g + d + vc * 0.5,
                                idx,
                            );
                            continue;
                        }
                        for l2 in 0..nl {
                            if l2 != lu && self.m(l2).site_vert.contains_key(&site) {
                                push(
                                    &mut best,
                                    &mut nodes,
                                    &mut open,
                                    NK::Vert { site, li: l2 as u8 },
                                    q,
                                    node.g + d + vc,
                                    idx,
                                );
                            }
                        }
                    }
                    // new vias on sites inside the triangle (nearest few)
                    let sites = &m.tri_sites[t as usize];
                    if sites.is_empty() {
                        continue;
                    }
                    let mut cand: Vec<(f64, u32)> = sites
                        .iter()
                        .filter(|&&s| match self.sub.shared.sites[s as usize].pad {
                            None => true,
                            // a via into a target pad ends the connection (last resort)
                            Some(p) => self.cfg.via_in_pad && is_target(p as usize),
                        })
                        .map(|&s| (self.site_point(s).dist(node.pos), s))
                        .collect();
                    cand.sort_by(|a, b| a.0.total_cmp(&b.0));
                    let mut tried = 0;
                    // vias of this path so far (hole to hole spacing within one connection)
                    let mut own_vias: Vec<Point> = Vec::new();
                    {
                        let mut p = idx;
                        while p != u32::MAX {
                            if let NK::Site { site, .. } | NK::Vert { site, .. } = nodes[p as usize].kind {
                                own_vias.push(self.site_point(site));
                            }
                            p = nodes[p as usize].parent;
                        }
                    }
                    let (_, own_drill, _) = self.via_size(net);
                    let hole_gap = own_drill + self.board.rules.hole_to_hole;
                    for (d, site) in cand {
                        if tried >= self.cfg.sites_per_tri {
                            break;
                        }
                        let q = self.site_point(site);
                        if own_vias.iter().any(|v| v.dist(q) < hole_gap) {
                            continue;
                        }
                        if let Some((ep, es)) = entry_pos
                            && !self.half_chord_ok(t, lu, net, q, ep, es)
                        {
                            continue;
                        }
                        if !self.via_ok_site(site, net) {
                            continue;
                        }
                        tried += 1;
                        if self.sub.shared.sites[site as usize].pad.is_some()
                            || (conn.goal == Goal::Plane && self.in_plane(q, net))
                        {
                            push(
                                &mut best,
                                &mut nodes,
                                &mut open,
                                NK::GoalSite { site },
                                q,
                                node.g + d + self.cfg.via_cost * 0.5,
                                idx,
                            );
                            continue;
                        }
                        for l2 in 0..nl {
                            if l2 != lu {
                                push(
                                    &mut best,
                                    &mut nodes,
                                    &mut open,
                                    NK::Site { site, li: l2 as u8 },
                                    q,
                                    node.g + d + self.cfg.via_cost,
                                    idx,
                                );
                            }
                        }
                    }
                }
                NK::Vert { site, li } => {
                    let lu = li as usize;
                    let m = self.m(lu);
                    let Some(&v) = m.site_vert.get(&site) else { continue };
                    for &t in &m.vert_tris[v as usize] {
                        let status = m.status[t as usize];
                        if !Substrate::passable(self.board, status, net) {
                            continue;
                        }
                        let i = m.tris[t as usize].iter().position(|&x| x == v).unwrap() as u8;
                        let k = (i + 1) % 3;
                        cross_out(
                            &mut best,
                            &mut nodes,
                            &mut open,
                            t,
                            lu,
                            k,
                            Some((CPos::Vertex(i), None)),
                            None,
                            &node,
                            idx,
                        );
                    }
                }
                NK::Site { site, li } => {
                    let lu = li as usize;
                    let t = self.m(lu).site_tri[site as usize];
                    for k in 0..3u8 {
                        cross_out(
                            &mut best,
                            &mut nodes,
                            &mut open,
                            t,
                            lu,
                            k,
                            None,
                            Some(node.pos),
                            &node,
                            idx,
                        );
                    }
                }
            }
        }
        None
    }

    // ------------------------------------------------------------------ commit / rip-up

    /// Commit a found chain. Returns false (after placing nothing but mesh splits) if the chain
    /// no longer fits the mesh after its own vias split triangles; the caller searches again.
    fn commit(&mut self, ci: usize, chain: &[Node]) -> bool {
        let mut chain = chain.to_vec();
        let net = self.conns[ci].net;
        // Place new vias first: split their triangles and rewrite the chain to the new mesh.
        let mut split_tris: Vec<(usize, u32)> = Vec::new(); // (layer, old triangle)
        for i in 0..chain.len() {
            let (site, li) = match chain[i].kind {
                NK::Site { site, li } => (site, Some(li)),
                NK::GoalSite { site } => (site, None),
                _ => continue,
            };
            if !self.sub.site_usable(site) {
                return false;
            }
            let per_layer = self.split_at(site);
            for (l, &(t, _)) in per_layer.iter().enumerate() {
                split_tris.push((l, t));
            }
            chain[i].kind = match li {
                Some(li) => NK::Vert { site, li },
                None => NK::GoalVia { site },
            };
            // the arriving triangle node: old t entered through edge k -> X_k entered through 0
            if i > 0
                && let NK::Tri {
                    t: tp,
                    li: lp,
                    entry,
                    slot,
                } = chain[i - 1].kind
                && entry < 3
                && tp == per_layer[lp as usize].0
            {
                chain[i - 1].kind = NK::Tri {
                    t: per_layer[lp as usize].1[entry as usize],
                    li: lp,
                    entry: 0,
                    slot,
                };
            }
        }
        // every consecutive pair of triangle nodes must still be adjacent
        for w in chain.windows(2) {
            if let (
                NK::Tri { t, li, .. },
                NK::Tri {
                    t: t2, li: l2, entry, ..
                },
            ) = (w[0].kind, w[1].kind)
                && li == l2
                && entry < 3
            {
                let m = self.m(li as usize);
                if !m.tri_edges[t as usize].contains(&m.tri_edges[t2 as usize][entry as usize]) {
                    return false;
                }
            }
            if let NK::Tri { t, li, .. } = w[0].kind
                && split_tris.contains(&(li as usize, t))
                && !matches!(w[1].kind, NK::Vert { .. } | NK::GoalVia { .. })
            {
                return false;
            }
        }
        let conn = self.conns[ci].clone();
        let cls = self.board.class_of(net);
        let (width, clearance) = (cls.track_width, cls.clearance.max(self.board.rules.min_clearance));
        let mut seg_ids = Vec::new();
        let mut via_ids = Vec::new();
        let (necks, neck_w) = self.necks(ci);

        // Split the chain into runs of triangle nodes on one layer.
        let mut i = 0;
        let mut start = End::Pin(conn.from);
        while i < chain.len() {
            let NK::Tri { li, .. } = chain[i].kind else {
                if let NK::Vert { site, .. } = chain[i].kind {
                    start = End::Via(site);
                }
                i += 1;
                continue;
            };
            let mut j = i;
            let mut run: Vec<(u32, u8, u16)> = Vec::new();
            while j < chain.len() {
                match chain[j].kind {
                    NK::Tri { t, li: l, entry, slot } if l == li => run.push((t, entry, slot)),
                    _ => break,
                }
                j += 1;
            }
            let last_t = run.last().unwrap().0;
            let end = match chain.get(j).map(|n| n.kind) {
                Some(NK::Vert { site, .. }) | Some(NK::GoalVia { site }) => {
                    self.add_via(site, net);
                    via_ids.push(site);
                    End::Via(site)
                }
                Some(NK::GoalPin { pin }) => End::Pin(pin),
                // through hole layer change: the run ends at that pin
                _ => End::Pin(self.tri_pin(li as usize, last_t).unwrap_or(conn.from)),
            };
            let seg = Seg {
                net,
                conn: ci as u32,
                li,
                width,
                clearance,
                start,
                end,
                crossings: Vec::new(),
                widths: Vec::new(),
                chord_tris: Vec::new(),
                necks: necks.clone(),
                neck_w,
            };
            seg_ids.push(self.insert_seg(seg, &run));
            start = end;
            i = j;
        }
        self.conn_segs[ci] = seg_ids;
        self.conn_vias[ci] = via_ids;
        true
    }

    /// Insert a segment: its chords (against the lists as the search saw them), then its
    /// crossings at their slots.
    fn insert_seg(&mut self, mut s: Seg, run: &[(u32, u8, u16)]) -> u32 {
        let id = self.segs.len() as u32;
        let li = s.li as usize;
        let m = &self.sub.m[li];
        let vert = |e: End| match e {
            End::Via(site) => m.site_vert.get(&site).copied(),
            End::Pin(_) => None,
        };
        let vpos = |t: u32, v: u32| CPos::Vertex(m.tris[t as usize].iter().position(|&x| x == v).unwrap_or(0) as u8);
        let mut chords: Vec<(u32, Chord)> = Vec::new();
        // leaving a via vertex: the chord in the triangle before the first crossing
        if let (Some(v), Some(&(t0, e0, _))) = (vert(s.start), run.first())
            && e0 < 3
        {
            let e = m.tri_edges[t0 as usize][e0 as usize];
            let from = m.other_tri(e, t0);
            if from != NONE && m.status[from as usize] == NONE && m.tris[from as usize].contains(&v) {
                chords.push((
                    from,
                    Chord {
                        seg: id,
                        a: vpos(from, v),
                        b: CPos::Edge(m.local_edge(from, e) as u8),
                    },
                ));
            }
        }
        for (idx, &(t, entry, _)) in run.iter().enumerate() {
            if m.status[t as usize] != NONE {
                continue;
            }
            let a = if entry < 3 {
                CPos::Edge(entry)
            } else if let Some(v) = vert(s.start) {
                vpos(t, v)
            } else {
                continue;
            };
            let b = if let Some(&(t2, e2, _)) = run.get(idx + 1) {
                if e2 >= 3 {
                    continue;
                }
                CPos::Edge(m.local_edge(t, m.tri_edges[t2 as usize][e2 as usize]) as u8)
            } else if let Some(v) = vert(s.end) {
                vpos(t, v)
            } else {
                continue;
            };
            chords.push((t, Chord { seg: id, a, b }));
        }
        let mut crossings = Vec::new();
        for &(t, entry, slot) in run {
            if entry >= 3 {
                continue;
            }
            let e = m.tri_edges[t as usize][entry as usize];
            let from = m.other_tri(e, t);
            crossings.push((e, from, slot));
        }
        for (e, from, slot) in crossings {
            let w = self.width_at_edge(li, e, s.width, &s.necks, s.neck_w);
            s.widths.push(w);
            let list = &mut self.cross[li][e as usize];
            let j = (slot as usize).min(list.len());
            list.insert(j, id);
            s.crossings.push(Crossing { e, from });
        }
        for (t, ch) in chords {
            s.chord_tris.push(t);
            self.chords[li].entry(t).or_default().push(ch);
        }
        self.segs.push(Some(s));
        id
    }

    pub fn rip_up(&mut self, ci: usize) {
        for sid in std::mem::take(&mut self.conn_segs[ci]) {
            let Some(s) = self.segs[sid as usize].take() else {
                continue;
            };
            let li = s.li as usize;
            for c in &s.crossings {
                self.cross[li][c.e as usize].retain(|&x| x != sid);
            }
            for t in &s.chord_tris {
                if let Some(list) = self.chords[li].get_mut(t) {
                    list.retain(|ch| ch.seg != sid);
                }
            }
        }
        for v in std::mem::take(&mut self.conn_vias[ci]) {
            self.remove_via(v);
        }
    }

    // ------------------------------------------------------------------ driver

    /// Build the connection list: plane fan-outs and MST connections, in routing order.
    pub fn plan(&mut self) {
        let b = self.board;
        let mut conns: Vec<(i64, f64, Conn)> = Vec::new();
        for net in b.routable_nets() {
            let n = &b.nets[net as usize];
            let has_plane = b.planes.iter().any(|p| p.net == net);
            let prio = n.priority as i64;
            // one representative per group of overlapping pads
            let mut reps: Vec<usize> = Vec::new();
            for &p in &n.pins {
                if !reps.iter().any(|&r| self.pin_group[r] == self.pin_group[p]) {
                    reps.push(p);
                }
            }
            let reaches_plane = |p: usize| {
                n.pins.iter().any(|&q| {
                    self.pin_group[q] == self.pin_group[p]
                        && b.planes.iter().any(|pl| {
                            pl.net == net && b.pins[q].layers.has(pl.layer) && pl.outline.contains(b.pins[q].pos)
                        })
                })
            };
            if has_plane {
                for &p in &reps {
                    if reaches_plane(p) {
                        continue; // a through hole pad of the group reaches the plane itself
                    }
                    let key = if self.cfg.fanout_first {
                        i64::MIN / 2 + prio
                    } else {
                        prio
                    };
                    conns.push((
                        key,
                        0.0,
                        Conn {
                            net,
                            from: p,
                            goal: Goal::Plane,
                        },
                    ));
                }
                continue;
            }
            // Prim order: each new pin connects to the pins already in the tree
            let pins = &reps;
            if pins.len() < 2 {
                continue;
            }
            let mut in_tree = vec![false; pins.len()];
            in_tree[0] = true;
            let mut dist: Vec<f64> = pins.iter().map(|&p| b.pins[p].pos.dist(b.pins[pins[0]].pos)).collect();
            for _ in 1..pins.len() {
                let (k, d) = (0..pins.len())
                    .filter(|&k| !in_tree[k])
                    .map(|k| (k, dist[k]))
                    .min_by(|a, b| a.1.total_cmp(&b.1))
                    .unwrap();
                let tree: Vec<usize> = (0..pins.len()).filter(|&j| in_tree[j]).map(|j| pins[j]).collect();
                conns.push((
                    prio,
                    d,
                    Conn {
                        net,
                        from: pins[k],
                        goal: Goal::Pins(tree),
                    },
                ));
                in_tree[k] = true;
                for j in 0..pins.len() {
                    dist[j] = dist[j].min(b.pins[pins[j]].pos.dist(b.pins[pins[k]].pos));
                }
            }
        }
        // stable order: priority, then short connections first; the seed perturbs lengths
        let seed = self.cfg.seed;
        conns.sort_by(|a, b| {
            let ja = jitter(seed, a.2.from as u64) * a.1;
            let jb = jitter(seed, b.2.from as u64) * b.1;
            a.0.cmp(&b.0).then(ja.total_cmp(&jb))
        });
        self.conns = conns.into_iter().map(|c| c.2).collect();
        self.conn_segs = vec![Vec::new(); self.conns.len()];
        self.conn_vias = vec![Vec::new(); self.conns.len()];
    }

    /// Reorder the planned connections by a key (lower first; ties keep the plan order).
    pub fn reorder(&mut self, key: impl Fn(&Conn) -> f64) {
        let mut idx: Vec<usize> = (0..self.conns.len()).collect();
        idx.sort_by(|&a, &b| key(&self.conns[a]).total_cmp(&key(&self.conns[b])).then(a.cmp(&b)));
        self.conns = idx.iter().map(|&i| self.conns[i].clone()).collect();
    }

    /// Route all planned connections. Returns the number routed.
    fn conn_pins(&self, ci: usize) -> Vec<usize> {
        let c = &self.conns[ci];
        let mut p = vec![c.from];
        if let Goal::Pins(t) = &c.goal {
            p.extend(t.iter().copied());
        }
        p
    }

    pub fn run(&mut self) -> usize {
        let mut failed = Vec::new();
        for ci in 0..self.conns.len() {
            for p in self.conn_pins(ci) {
                self.pending[p] += 1;
            }
        }
        for ci in 0..self.conns.len() {
            let ok = self.route_one(ci);
            for p in self.conn_pins(ci) {
                self.pending[p] -= 1;
            }
            if !ok {
                failed.push(ci);
            }
        }
        // Fan-outs that found no plane: try a via in the pad, then connect to the nearest pin of
        // the net instead.
        // the second chance runs without hot spots: they must never cost a connection
        self.cfg.hotspots = Default::default();
        let vip = self.cfg.via_in_pad;
        self.cfg.via_in_pad = true;
        failed.retain(|&ci| !self.route_one(ci));
        self.cfg.via_in_pad = vip;
        for &ci in &failed {
            if self.conns[ci].goal == Goal::Plane {
                let c = &self.conns[ci];
                let pins: Vec<usize> = self.board.nets[c.net as usize]
                    .pins
                    .iter()
                    .copied()
                    .filter(|&p| p != c.from)
                    .collect();
                self.conns[ci].goal = Goal::Pins(pins);
            }
        }
        // Rip-up and reroute: route the failed connection first, then the ripped ones.
        for _pass in 0..self.cfg.ripup_passes {
            failed.retain(|&ci| self.conn_segs[ci].is_empty() && !self.route_one(ci));
            if failed.is_empty() {
                break;
            }
            let mut still = Vec::new();
            for &ci in &failed {
                let victims = self.blockers(ci);
                for &v in &victims {
                    self.rip_up(v);
                }
                let ok = self.route_one(ci);
                for &v in &victims {
                    if !self.route_one(v) {
                        still.push(v);
                    }
                }
                if !ok {
                    still.push(ci);
                }
            }
            still.sort_unstable();
            still.dedup();
            failed = still;
        }
        self.conn_segs.iter().filter(|s| !s.is_empty()).count()
    }

    pub fn route_one(&mut self, ci: usize) -> bool {
        for _ in 0..3 {
            match self.search(ci) {
                Some(chain) => {
                    let before = if std::env::var("GB_VALIDATE_EACH").is_ok() {
                        self.validate().len()
                    } else {
                        0
                    };
                    if self.commit(ci, &chain) {
                        if std::env::var("GB_VALIDATE_EACH").is_ok() {
                            let v = self.validate();
                            if v.len() > before {
                                let kinds: Vec<String> = chain.iter().map(|n| format!("{:?}", n.kind)).collect();
                                eprintln!(
                                    "conn {ci} broke the state: {:?}\n  chain {}",
                                    &v[v.len().min(before)..],
                                    kinds.join(" ")
                                );
                            }
                        }
                        return true;
                    }
                }
                None => return false,
            }
        }
        false
    }

    /// Connections whose routes pass close to the straight line of a failed connection.
    fn blockers(&self, ci: usize) -> Vec<usize> {
        let c = &self.conns[ci];
        let a = self.board.pins[c.from].pos;
        let b = match &c.goal {
            Goal::Pins(p) => p
                .iter()
                .map(|&q| self.board.pins[q].pos)
                .min_by(|x, y| x.dist(a).total_cmp(&y.dist(a)))
                .unwrap_or(a),
            Goal::Plane => a,
        };
        let radius = 1.0 + a.dist(b) * 0.15;
        let mut out = Vec::new();
        for (cj, segs) in self.conn_segs.iter().enumerate() {
            if cj == ci || segs.is_empty() || self.conns[cj].net == c.net {
                continue;
            }
            let near = segs.iter().any(|&s| {
                let s = self.segs[s as usize].as_ref().unwrap();
                let m = self.m(s.li as usize);
                s.crossings.iter().any(|cr| {
                    let [u, v] = m.edges[cr.e as usize];
                    let mid = (m.verts[u as usize] + m.verts[v as usize]) * 0.5;
                    point_seg_dist(mid, a, b) < radius
                })
            });
            if near {
                out.push(cj);
            }
        }
        out.truncate(6);
        out
    }

    pub fn unrouted(&self) -> Vec<usize> {
        (0..self.conns.len())
            .filter(|&ci| self.conn_segs[ci].is_empty())
            .collect()
    }

    /// Chords of triangle t (debugging aid).
    pub fn chords_debug(&self, li: usize, t: u32) -> Vec<String> {
        self.chords[li]
            .get(&t)
            .map(|l| l.iter().map(|c| format!("{}:{:?}->{:?}", c.seg, c.a, c.b)).collect())
            .unwrap_or_default()
    }

    /// Consistency checks of the topological state (debugging aid).
    pub fn validate(&self) -> Vec<String> {
        let mut out = Vec::new();
        let mut over = 0;
        for li in 0..self.nl() {
            for (e, list) in self.cross[li].iter().enumerate() {
                if list.is_empty() {
                    continue;
                }
                let items = self.items_on(li, e as u32);
                if self.pack(li, e as u32, &items, None).is_none() {
                    over += 1;
                }
                for &s in list {
                    let seg = self.segs[s as usize].as_ref();
                    if seg.is_none_or(|sg| sg.li as usize != li || !sg.crossings.iter().any(|c| c.e as usize == e)) {
                        out.push(format!("edge {e} lists seg {s} which does not cross it"));
                    }
                }
            }
        }
        if over > 0 {
            out.push(format!("{over} edges over capacity"));
        }
        for (sid, s) in self.segs.iter().enumerate() {
            let Some(s) = s else { continue };
            let m = self.m(s.li as usize);
            for w in s.crossings.windows(2) {
                if m.other_tri(w[0].e, w[0].from) != w[1].from {
                    out.push(format!(
                        "seg {sid}: crossing chain broken at edge {} -> {}",
                        w[0].e, w[1].e
                    ));
                    break;
                }
            }
            for c in &s.crossings {
                if !self.cross[s.li as usize][c.e as usize].contains(&(sid as u32)) {
                    out.push(format!("seg {sid}: missing in list of edge {}", c.e));
                }
            }
        }
        out
    }
}

/// Intersection of the lines ab and cd: (parameter on ab, parameter on cd).
fn line_params(a: Point, b: Point, c: Point, d: Point) -> (f64, f64) {
    let r = b - a;
    let s = d - c;
    let den = r.cross(s);
    if den.abs() < 1e-300 {
        return (0.5, 0.5);
    }
    ((c - a).cross(s) / den, (c - a).cross(r) / den)
}

/// Intersection of segments ab and cd: (parameter on ab, parameter on cd).
#[allow(dead_code)]
fn seg_param_intersection(a: Point, b: Point, c: Point, d: Point) -> Option<(f64, f64)> {
    let r = b - a;
    let s = d - c;
    let den = r.cross(s);
    if den.abs() < 1e-15 {
        return None;
    }
    let t = (c - a).cross(s) / den;
    let u = (c - a).cross(r) / den;
    if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
        Some((t, u))
    } else {
        None
    }
}

fn jitter(seed: u64, x: u64) -> f64 {
    if seed == 0 {
        return 1.0;
    }
    let mut z = seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ x.wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z ^= z >> 31;
    z = z.wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^= z >> 29;
    0.5 + (z % 1000) as f64 / 1000.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topo::SubstrateConfig;

    #[test]
    fn routes_two_pads() {
        let b = crate::topo::tests::two_pad_board();
        let sub = Substrate::build(&b, &SubstrateConfig::default());
        let mut r = Router::new(&b, &sub, RouterConfig::default());
        r.plan();
        assert_eq!(r.conns.len(), 1);
        assert_eq!(r.run(), 1);
        let geo = r.geometry();
        assert_eq!(geo.tracks.len(), 1);
        let p = &geo.tracks[0].pts;
        assert_eq!(p.len(), 2, "straight track expected: {p:?}");
        assert!((p[0].dist(p[1]) - 6.0).abs() < 1e-9);
        assert!(crate::check::check(&b, &geo).is_empty());
    }
}
