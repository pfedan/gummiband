//! Topological router: A* over triangle edges with slot positions. Every route segment on a
//! layer is a chain of edge crossings; inside a free triangle it is a chord between two boundary
//! positions, and no two chords of different nets may interleave. So each layer stays planar
//! and the realization can pull every segment taut without crossings.
//!
//! Vias sit on via sites inside free triangles. Placing one splits the triangle in three; the
//! routes that already cross it are threaded through the new triangles.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use crate::board::{Board, NetId, ObstacleKind};
use crate::geom::{Point, orient, point_seg_dist, segs_intersect};
use crate::topo::{NONE, OUTSIDE, Substrate};

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
}

impl Default for RouterConfig {
    fn default() -> Self {
        RouterConfig {
            via_cost: 1.5,
            fanout_first: true,
            max_expansions: 400_000,
            seed: 0,
            ripup_passes: 3,
            sites_per_tri: 3,
            margin: 0.01,
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
    Vertex(u32),
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
    /// Free triangles with a chord of this segment.
    pub chord_tris: Vec<u32>,
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

/// Mutable routing state. Owns its copy of the mesh, because vias split triangles.
#[derive(Clone)]
pub struct Router<'a> {
    pub board: &'a Board,
    pub sub: Substrate,
    pub cfg: RouterConfig,
    /// [layer][edge] -> segment ids ordered from edges[e][0] to edges[e][1]
    pub cross: Vec<Vec<Vec<u32>>>,
    chords: Vec<HashMap<u32, Vec<Chord>>>,
    pub segs: Vec<Option<Seg>>,
    pub vias: HashMap<u32, ViaInfo>,
    via_cells: HashMap<(i32, i32), Vec<u32>>,
    pub conns: Vec<Conn>,
    /// Segment ids per connection (empty = unrouted).
    pub conn_segs: Vec<Vec<u32>>,
    pub conn_vias: Vec<Vec<u32>>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum NK {
    /// In triangle t on layer li, entered through local edge `entry` (3 = none) at `slot`.
    Tri { t: u32, li: u8, entry: u8, slot: u16 },
    /// At an existing via vertex v on layer li.
    Vert { v: u32, li: u8 },
    /// At a new via on site `site` inside free triangle t, continuing on layer li.
    Site { t: u32, site: u32, li: u8 },
    GoalPin { pin: usize },
    GoalVia { v: u32 },
    GoalSite { t: u32, site: u32 },
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
        NK::Vert { v, li } => (1, v, li as u32),
        NK::GoalPin { pin } => (2, pin as u32, 0),
        NK::GoalVia { v } => (3, v, 0),
        NK::Site { site, li, .. } => (4, site, li as u32),
        NK::GoalSite { site, .. } => (5, site, 0),
    }
}

impl<'a> Router<'a> {
    pub fn new(board: &'a Board, sub: &Substrate, cfg: RouterConfig) -> Router<'a> {
        let nl = sub.layers.len();
        Router {
            board,
            sub: sub.clone(),
            cfg,
            cross: vec![vec![Vec::new(); sub.edges.len()]; nl],
            chords: vec![HashMap::new(); nl],
            segs: Vec::new(),
            vias: HashMap::new(),
            via_cells: HashMap::new(),
            conns: Vec::new(),
            conn_segs: Vec::new(),
            conn_vias: Vec::new(),
        }
    }

    // ------------------------------------------------------------------ clearances

    /// Clearance a route of clearance `c` keeps from obstacle `obs`.
    pub fn eff_clearance(&self, obs: u32, c: f64) -> f64 {
        if obs == OUTSIDE {
            return self.board.rules.edge_clearance;
        }
        let o = &self.board.obstacles[obs as usize];
        match o.kind {
            ObstacleKind::Keepout { .. } => 0.0,
            ObstacleKind::Hole => self.board.rules.hole_clearance.max(o.clearance),
            _ => o.clearance.max(c),
        }
    }

    fn obs_net(&self, obs: u32) -> Option<NetId> {
        if obs == OUTSIDE { None } else { self.board.obstacles[obs as usize].net }
    }

    /// Distance from the edge end vertex to the centre line of a track of `it`.
    fn end_need(&self, li: usize, e: u32, end: usize, it: &Item) -> f64 {
        let mut need: f64 = 0.0;
        for d in self.sub.demands(li, e, end) {
            if self.obs_net(d.obs) == Some(it.net) {
                continue;
            }
            need = need.max((self.eff_clearance(d.obs, it.c) + it.w / 2.0) * d.factor as f64);
        }
        let v = self.sub.edges[e as usize][end];
        if let Some(via) = self.vias.get(&v)
            && via.net != it.net
        {
            need = need.max(via.r + via.clearance.max(it.c) + it.w / 2.0);
        }
        need
    }

    /// Need of a track at an obstacle vertex v (vertex demands only), for a new edge.
    fn vertex_need(&self, li: usize, v: u32, it: &Item) -> f64 {
        let mut need: f64 = 0.0;
        for &t in &self.sub.vert_tris[v as usize] {
            let o = self.sub.tri_status[li][t as usize];
            if o == NONE || self.obs_net(o) == Some(it.net) {
                continue;
            }
            need = need.max(self.eff_clearance(o, it.c) + it.w / 2.0);
        }
        if let Some(via) = self.vias.get(&v)
            && via.net != it.net
        {
            need = need.max(via.r + via.clearance.max(it.c) + it.w / 2.0);
        }
        need
    }

    fn item_of(&self, seg: u32) -> Item {
        let s = self.segs[seg as usize].as_ref().unwrap();
        Item { net: s.net, w: s.width, c: s.clearance, sides: [2, 2] }
    }

    /// Item of segment `seg` crossing edge e, with its sides relative to the two apexes.
    fn item_on(&self, li: usize, e: u32, seg: u32) -> Item {
        let mut it = self.item_of(seg);
        for (k, &t) in self.sub.edge_tris[e as usize].iter().enumerate() {
            if t != NONE {
                it.sides[k] = self.chord_side(li, e, t, seg);
            }
        }
        it
    }

    pub fn items_on(&self, li: usize, e: u32) -> Vec<Item> {
        self.cross[li][e as usize].iter().map(|&s| self.item_on(li, e, s)).collect()
    }

    /// Side of the apex of t the chord of `seg` in t passes, seen on edge e (see [`Item`]).
    fn chord_side(&self, li: usize, e: u32, t: u32, seg: u32) -> u8 {
        let Some(list) = self.chords[li].get(&t) else { return 2 };
        let Some(ch) = list.iter().find(|c| c.seg == seg) else { return 2 };
        let ke = self.sub.local_edge(t, e) as u8;
        let other = if ch.a == CPos::Edge(ke) { ch.b } else { ch.a };
        self.side_of_pos(t, e, other)
    }

    /// Side (0/1 = towards edges[e][0]/[1]) for a chord from edge e to `other` in triangle t.
    fn side_of_pos(&self, t: u32, e: u32, other: CPos) -> u8 {
        let [u, v] = self.sub.edges[e as usize];
        let tv = self.sub.tris[t as usize];
        let x = match other {
            CPos::Edge(k) => {
                // the edge (x, apex): x is its endpoint on e
                let (p, q) = (tv[k as usize], tv[(k as usize + 1) % 3]);
                if p == u || q == u { u } else if p == v || q == v { v } else { return 2 }
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
        let mut b = vec![(f64::NEG_INFINITY, f64::INFINITY); items.len()];
        let [u, v] = self.sub.edges[e as usize];
        let pu = self.sub.verts[u as usize];
        let pv = self.sub.verts[v as usize];
        let len = pu.dist(pv);
        if len <= 0.0 {
            return b;
        }
        let dir = (pv - pu) * (1.0 / len);
        let m = self.cfg.margin;
        for (k, &t) in self.sub.edge_tris[e as usize].iter().enumerate() {
            if t == NONE || self.sub.tri_status[li][t as usize] != NONE {
                continue;
            }
            let w = *self.sub.tris[t as usize].iter().find(|&&x| x != u && x != v).unwrap();
            let pw = self.sub.verts[w as usize];
            let h = (pw - pu).cross(dir).abs();
            let proj = (pw - pu).dot(dir);
            // obstacles at the apex: (net, clearance rule obstacle or via)
            let via_here = self.vias.get(&w).copied().or(match extra {
                Some(x) if x.side == k => Some(x.via),
                _ => None,
            });
            let extra_p = match extra {
                Some(x) if x.side == k => Some(x.p),
                _ => None,
            };
            let (h, proj) = match extra_p {
                Some(p) => ((p - pu).cross(dir).abs(), (p - pu).dot(dir)),
                None => (h, proj),
            };
            let mut obs: Vec<u32> = Vec::new();
            if extra_p.is_none() {
                for &tt in &self.sub.vert_tris[w as usize] {
                    let o = self.sub.tri_status[li][tt as usize];
                    if o != NONE && !obs.contains(&o) {
                        obs.push(o);
                    }
                }
            }
            if obs.is_empty() && via_here.is_none() {
                continue;
            }
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
                if let Some(via) = via_here
                    && via.net != it.net
                {
                    need = need.max(via.r + via.clearance.max(it.c) + it.w / 2.0);
                }
                if need <= 0.0 {
                    continue;
                }
                let need = need + m;
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

    fn gap(a: &Item, b: &Item) -> f64 {
        if a.net == b.net { 0.0 } else { a.w / 2.0 + a.c.max(b.c) + b.w / 2.0 }
    }

    /// Pack items along a segment of length `len` with end needs; centre line intervals.
    fn pack_with(
        items: &[Item],
        len: f64,
        need_u: impl Fn(&Item) -> f64,
        need_v: impl Fn(&Item) -> f64,
        bounds: &[(f64, f64)],
    ) -> Option<Vec<(f64, f64)>> {
        let n = items.len();
        let mut lo = vec![0.0; n];
        let mut hi = vec![0.0; n];
        for i in 0..n {
            let mut x = need_u(&items[i]).max(bounds.get(i).map_or(f64::NEG_INFINITY, |b| b.0));
            if i > 0 {
                x = x.max(lo[i - 1] + Self::gap(&items[i - 1], &items[i]));
            }
            lo[i] = x;
        }
        for i in (0..n).rev() {
            let mut y = (len - need_v(&items[i])).min(bounds.get(i).map_or(f64::INFINITY, |b| b.1));
            if i + 1 < n {
                y = y.min(hi[i + 1] - Self::gap(&items[i], &items[i + 1]));
            }
            hi[i] = y;
        }
        if (0..n).any(|i| lo[i] > hi[i] + 1e-9) {
            return None;
        }
        Some(lo.into_iter().zip(hi).map(|(a, b)| if a > b { ((a + b) / 2.0, (a + b) / 2.0) } else { (a, b) }).collect())
    }

    /// Centre line intervals of all items on edge e (u to v), or None if they do not fit.
    pub fn pack(&self, li: usize, e: u32, items: &[Item], extra_via: Option<(u32, &ViaInfo)>) -> Option<Vec<(f64, f64)>> {
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
        let mut extra_apex = extra_apex;
        // a via about to be placed at the apex vertex of an adjacent triangle
        if let Some((vv, via)) = extra_via {
            for (k, &t) in self.sub.edge_tris[e as usize].iter().enumerate() {
                if t != NONE && self.sub.tris[t as usize].contains(&vv) && !self.sub.edges[e as usize].contains(&vv) {
                    extra_apex = Some(ExtraApex { side: k, p: self.sub.verts[vv as usize], via: *via });
                }
            }
        }
        let bounds = self.apex_bounds(li, e, items, extra_apex);
        let len = self.sub.edge_len(e);
        let [u, v] = self.sub.edges[e as usize];
        let m = self.cfg.margin;
        let via_need = |vert: u32, it: &Item| -> f64 {
            match extra_via {
                Some((vv, via)) if vv == vert && via.net != it.net => via.r + via.clearance.max(it.c) + it.w / 2.0,
                _ => 0.0,
            }
        };
        Self::pack_with(
            items,
            len,
            |it| self.end_need(li, e, 0, it).max(via_need(u, it)) + m,
            |it| self.end_need(li, e, 1, it).max(via_need(v, it)) + m,
            &bounds,
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
                let e = self.sub.tri_edges[t as usize][k as usize];
                let f = self.edge_frac(li, e, seg, slot);
                let forward = self.sub.tris[t as usize][k as usize] == self.sub.edges[e as usize][0];
                k as f64 + if forward { f } else { 1.0 - f }
            }
        }
    }

    /// Approximate point of a boundary position (for geometric chord tests).
    fn pos_point(&self, t: u32, li: usize, p: CPos, seg: Option<u32>, slot: Option<usize>) -> Point {
        match p {
            CPos::Vertex(i) => self.sub.verts[self.sub.tris[t as usize][i as usize] as usize],
            CPos::Edge(k) => {
                let e = self.sub.tri_edges[t as usize][k as usize];
                let f = self.edge_frac(li, e, seg, slot);
                let [u, v] = self.sub.edges[e as usize];
                self.sub.verts[u as usize].lerp(self.sub.verts[v as usize], f)
            }
        }
    }

    /// Can a chord of `net` from `a` (slot sa) to `b` (slot sb) be added to triangle t?
    #[allow(clippy::too_many_arguments)]
    fn chord_ok(&self, t: u32, li: usize, net: NetId, a: CPos, sa: Option<usize>, b: CPos, sb: Option<usize>) -> bool {
        let Some(list) = self.chords[li].get(&t) else { return true };
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
        let Some(list) = self.chords[li].get(&t) else { return true };
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
        (c.via_diameter / 2.0, c.via_drill, c.clearance.max(self.board.rules.min_clearance))
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
                        let o = &self.vias[&w];
                        let d = p.dist(self.sub.verts[w as usize]);
                        let holes = drill / 2.0 + o.drill / 2.0 + self.board.rules.hole_to_hole;
                        let need = if o.net == net { holes } else { (r + o.r + clr.max(o.clearance)).max(holes) };
                        if d < need {
                            return false;
                        }
                    }
                }
            }
        }
        true
    }

    /// Can a via of `net` be placed at existing vertex v? Some(true) if one of the net is there.
    fn via_ok_vertex(&self, v: u32, net: NetId) -> Option<bool> {
        let site = self.sub.vert_site[v as usize];
        if site == NONE {
            return None;
        }
        if let Some(via) = self.vias.get(&v) {
            return if via.net == net { Some(true) } else { None };
        }
        if !self.site_clear(site, net) {
            return None;
        }
        let (r, drill, clr) = self.via_size(net);
        let info = ViaInfo { net, r, drill, clearance: clr, users: 0 };
        for &t in &self.sub.vert_tris[v as usize] {
            for &e in &self.sub.tri_edges[t as usize] {
                for li in 0..self.sub.layers.len() {
                    if self.cross[li][e as usize].is_empty() {
                        continue;
                    }
                    let items = self.items_on(li, e);
                    self.pack(li, e, &items, Some((v, &info)))?;
                }
            }
        }
        Some(false)
    }

    /// Can a via of `net` be placed at site `site` inside free triangle t (splitting it)?
    fn via_ok_site(&self, t: u32, site: u32, net: NetId) -> bool {
        if !self.site_clear(site, net) {
            return false;
        }
        let p = self.sub.shared.sites[site as usize].p;
        let (r, _, clr) = self.via_size(net);
        let tv = self.sub.tris[t as usize];
        for li in 0..self.sub.layers.len() {
            let Some(list) = self.chords[li].get(&t) else { continue };
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
                let len = p.dist(self.sub.verts[vi as usize]);
                let m = self.cfg.margin;
                let ok = Self::pack_with(
                    &items,
                    len,
                    |it| self.vertex_need(li, vi, it) + m,
                    |it| if it.net == net { 0.0 } else { r + clr.max(it.c) + it.w / 2.0 + m },
                    &[],
                );
                if ok.is_none() {
                    return false;
                }
            }
        }
        // the old edges get the via as apex of their new triangle
        let via = ViaInfo { net, r, drill: 0.0, clearance: clr, users: 0 };
        for li in 0..self.sub.layers.len() {
            for k in 0..3 {
                let e = self.sub.tri_edges[t as usize][k];
                if self.cross[li][e as usize].is_empty() {
                    continue;
                }
                let side_idx = self.sub.edge_tris[e as usize].iter().position(|&x| x == t).unwrap();
                let mut items = self.items_on(li, e);
                let chords = self.chords[li].get(&t);
                for (it, &sg) in items.iter_mut().zip(&self.cross[li][e as usize]) {
                    it.sides[side_idx] = 2;
                    let Some(ch) = chords.and_then(|l| l.iter().find(|c| c.seg == sg)) else { continue };
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
                    let [u, _] = self.sub.edges[e as usize];
                    it.sides[side_idx] = if x == u { 0 } else { 1 };
                }
                if self.pack_ext(li, e, &items, None, Some(ExtraApex { side: side_idx, p, via })).is_none() {
                    return false;
                }
            }
        }
        true
    }

    /// Where an existing chord of triangle t crosses the new edges (s, v_i) if t is split at p:
    /// (i, u) with u the position along s -> v_i (0 at s, 1 at v_i), in chord order.
    fn rethread_hits(&self, t: u32, li: usize, ch: &Chord, p: Point) -> Vec<(usize, f64)> {
        let a = self.pos_point(t, li, ch.a, Some(ch.seg), None);
        let b = self.pos_point(t, li, ch.b, Some(ch.seg), None);
        let tv = self.sub.tris[t as usize];
        let mut hits = Vec::new();
        for i in 0..3 {
            let vi = self.sub.verts[tv[i] as usize];
            if let Some((tc, u)) = seg_param_intersection(a, b, p, vi)
                && tc > 1e-9
                && tc < 1.0 - 1e-9
                && u < 1.0 - 1e-9
            {
                hits.push((tc, i, u));
            }
        }
        hits.sort_by(|x, y| x.0.total_cmp(&y.0));
        hits.into_iter().map(|(_, i, u)| (i, u)).collect()
    }

    /// Split free triangle t at `site` and thread the existing chords through the new triangles.
    /// Returns the new vertex and the triangles X_k (X_k holds old edge k).
    fn split_at(&mut self, t: u32, site: u32) -> (u32, [u32; 3]) {
        let p = self.sub.shared.sites[site as usize].p;
        let nl = self.sub.layers.len();
        // geometry of the old chords, before the mesh changes
        let mut old: Vec<(usize, Chord, Vec<(usize, f64)>, Point)> = Vec::new();
        for li in 0..nl {
            if let Some(list) = self.chords[li].remove(&t) {
                for ch in list {
                    let hits = self.rethread_hits(t, li, &ch, p);
                    let a = self.pos_point(t, li, ch.a, Some(ch.seg), None);
                    let b = self.pos_point(t, li, ch.b, Some(ch.seg), None);
                    // a point just inside the triangle along the chord, to find the start triangle
                    old.push((li, ch, hits, a.lerp(b, 1e-6)));
                }
            }
        }
        let old_tv = self.sub.tris[t as usize];
        let old_edges = self.sub.tri_edges[t as usize];
        let (s, x) = self.sub.split(t, site);
        for li in 0..nl {
            self.cross[li].resize(self.sub.edges.len(), Vec::new());
        }
        let new_edge = |k: usize, sub: &Substrate| -> u32 { sub.tri_edges[x[k] as usize][2] }; // E_k = (s, v_k)
        let mut on_edge: Vec<HashMap<u32, Vec<(f64, u32)>>> = vec![HashMap::new(); nl];
        for (li, ch, hits, start_pt) in old {
            // start triangle
            let mut cur = match ch.a {
                CPos::Edge(k) => x[k as usize],
                CPos::Vertex(_) => *x
                    .iter()
                    .find(|&&xt| {
                        let tv = self.sub.tris[xt as usize];
                        let (a, b, c) = (self.sub.verts[tv[0] as usize], self.sub.verts[tv[1] as usize], self.sub.verts[tv[2] as usize]);
                        orient(a, b, start_pt) >= -1e-12 && orient(b, c, start_pt) >= -1e-12 && orient(c, a, start_pt) >= -1e-12
                    })
                    .unwrap_or(&x[0]),
            };
            let local_of_vertex = |xt: u32, v: u32, sub: &Substrate| sub.tris[xt as usize].iter().position(|&q| q == v).unwrap() as u8;
            let mut pos = match ch.a {
                CPos::Edge(_) => CPos::Edge(0),
                CPos::Vertex(i) => CPos::Vertex(local_of_vertex(cur, old_tv[i as usize], &self.sub)),
            };
            let mut new_cross: Vec<Crossing> = Vec::new();
            let mut new_chords: Vec<(u32, Chord)> = Vec::new();
            for (i, u) in hits {
                let e = new_edge(i, &self.sub);
                let k_out = self.sub.local_edge(cur, e) as u8;
                new_chords.push((cur, Chord { seg: ch.seg, a: pos, b: CPos::Edge(k_out) }));
                new_cross.push(Crossing { e, from: cur });
                on_edge[li].entry(e).or_default().push((u, ch.seg));
                let next = self.sub.other_tri(e, cur);
                pos = CPos::Edge(self.sub.local_edge(next, e) as u8);
                cur = next;
            }
            let end = match ch.b {
                CPos::Edge(_) => CPos::Edge(0),
                CPos::Vertex(i) => CPos::Vertex(local_of_vertex(cur, old_tv[i as usize], &self.sub)),
            };
            new_chords.push((cur, Chord { seg: ch.seg, a: pos, b: end }));
            // splice the crossings into the segment
            let seg = self.segs[ch.seg as usize].as_mut().unwrap();
            let exit = match ch.b {
                CPos::Edge(k) => seg.crossings.iter().position(|c| c.e == old_edges[k as usize] && c.from == t),
                CPos::Vertex(_) => None,
            };
            match exit {
                Some(ix) => {
                    seg.crossings[ix].from = cur;
                    for (j, c) in new_cross.into_iter().enumerate() {
                        seg.crossings.insert(ix + j, c);
                    }
                }
                None => seg.crossings.extend(new_cross),
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
        for (li, m) in on_edge.into_iter().enumerate() {
            for (e, mut list) in m {
                list.sort_by(|a, b| b.0.total_cmp(&a.0));
                self.cross[li][e as usize] = list.into_iter().map(|(_, sg)| sg).collect();
            }
        }
        (s, x)
    }

    fn in_plane(&self, p: Point, net: NetId) -> bool {
        let (r, _, _) = self.via_size(net);
        self.board
            .planes
            .iter()
            .any(|pl| pl.net == net && pl.outline.contains(p) && pl.outline.boundary_dist(p) >= r + 0.5)
    }

    fn add_via(&mut self, v: u32, net: NetId) {
        if let Some(via) = self.vias.get_mut(&v) {
            via.users += 1;
            return;
        }
        let (r, drill, clearance) = self.via_size(net);
        self.vias.insert(v, ViaInfo { net, r, drill, clearance, users: 1 });
        self.via_cells.entry(Self::via_cell(self.sub.verts[v as usize])).or_default().push(v);
    }

    fn remove_via(&mut self, v: u32) {
        let Some(via) = self.vias.get_mut(&v) else { return };
        via.users -= 1;
        if via.users == 0 {
            self.vias.remove(&v);
            if let Some(list) = self.via_cells.get_mut(&Self::via_cell(self.sub.verts[v as usize])) {
                list.retain(|&x| x != v);
            }
        }
    }

    // ------------------------------------------------------------------ search

    fn tri_pin(&self, li: usize, t: u32) -> Option<usize> {
        let s = self.sub.tri_status[li][t as usize];
        if s == NONE || s == OUTSIDE {
            return None;
        }
        self.board.obstacles[s as usize].pin
    }

    fn all_free(&self, t: u32) -> bool {
        (0..self.sub.layers.len()).all(|li| self.sub.tri_status[li][t as usize] == NONE)
    }

    /// A* for one connection. Returns the node chain from start to goal.
    fn search(&self, ci: usize) -> Option<Vec<Node>> {
        let conn = &self.conns[ci];
        let net = conn.net;
        let cls = self.board.class_of(net);
        let item = Item { net, w: cls.track_width, c: cls.clearance.max(self.board.rules.min_clearance), sides: [2, 2] };
        let src = &self.board.pins[conn.from];
        let target_pts: Vec<Point> = match &conn.goal {
            Goal::Pins(p) => p.iter().map(|&p| self.board.pins[p].pos).collect(),
            Goal::Plane => Vec::new(),
        };
        let h = |p: Point| -> f64 { target_pts.iter().map(|&q| q.dist(p)).fold(f64::INFINITY, f64::min).min(1e9) * if target_pts.is_empty() { 0.0 } else { 1.0 } };
        let is_target = |pin: usize| -> bool {
            match &conn.goal {
                Goal::Pins(p) => p.contains(&pin),
                Goal::Plane => {
                    let pn = &self.board.pins[pin];
                    pin != conn.from
                        && pn.net == net
                        && self.board.planes.iter().any(|pl| pl.net == net && pn.layers.has(pl.layer))
                }
            }
        };
        let nl = self.sub.layers.len();

        let mut nodes: Vec<Node> = Vec::new();
        let mut best: HashMap<(u8, u32, u32), f64> = HashMap::new();
        let mut open = BinaryHeap::new();
        let push = |best: &mut HashMap<(u8, u32, u32), f64>, nodes: &mut Vec<Node>, open: &mut BinaryHeap<Open>, kind: NK, pos: Point, g: f64, parent: u32| {
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

        for (li, &l) in self.sub.layers.iter().enumerate() {
            if !src.layers.has(l) {
                continue;
            }
            for &o in &src.obstacles {
                for &t in self.sub.obs_tris[li].get(&(o as u32)).map(|v| v.as_slice()).unwrap_or(&[]) {
                    push(&mut best, &mut nodes, &mut open, NK::Tri { t, li: li as u8, entry: 3, slot: 0 }, src.pos, 0.0, u32::MAX);
                }
            }
        }

        // crossings out of triangle t (layer lu) through local edge k, from boundary position `from`
        let cross_out = |best: &mut HashMap<(u8, u32, u32), f64>,
                         nodes: &mut Vec<Node>,
                         open: &mut BinaryHeap<Open>,
                         t: u32,
                         lu: usize,
                         k: u8,
                         chord_from: Option<(CPos, Option<usize>)>,
                         interior: Option<Point>,
                         node: &Node,
                         idx: u32| {
            let t2 = self.sub.tri_nbrs[t as usize][k as usize];
            if t2 == NONE {
                return;
            }
            let s2 = self.sub.tri_status[lu][t2 as usize];
            if !Substrate::passable(self.board, s2, net) {
                return;
            }
            let free = self.sub.tri_status[lu][t as usize] == NONE;
            let e = self.sub.tri_edges[t as usize][k as usize];
            let k2 = self.sub.local_edge(t2, e) as u8;
            if !free && s2 != NONE {
                push(best, nodes, open, NK::Tri { t: t2, li: lu as u8, entry: 3, slot: 0 }, node.pos, node.g, idx);
                return;
            }
            let mut item = item;
            if free && let Some((cp, cs)) = chord_from {
                // the chord in t fixes the side of t's apex on the exit edge ...
                let side_idx = self.sub.edge_tris[e as usize].iter().position(|&x| x == t).unwrap();
                item.sides[side_idx] = self.side_of_pos(t, e, cp);
                // ... and on the entry edge, which must still fit
                if let (CPos::Edge(ke), Some(js)) = (cp, cs) {
                    let e_in = self.sub.tri_edges[t as usize][ke as usize];
                    let mut it_in = item;
                    it_in.sides = [2, 2];
                    let si = self.sub.edge_tris[e_in as usize].iter().position(|&x| x == t).unwrap();
                    it_in.sides[si] = self.side_of_pos(t, e_in, CPos::Edge(k));
                    if self.slot_interval(lu, e_in, js, it_in).is_none() {
                        return;
                    }
                }
            }
            let [u, v] = self.sub.edges[e as usize];
            let (pu, pv) = (self.sub.verts[u as usize], self.sub.verts[v as usize]);
            let len = pu.dist(pv);
            let dir = (pv - pu) * (1.0 / len);
            let n = self.cross[lu][e as usize].len();
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
                let Some((lo, hi)) = self.slot_interval(lu, e, j, item) else { continue };
                let q = pu + dir * (node.pos - pu).dot(dir).clamp(lo, hi);
                push(best, nodes, open, NK::Tri { t: t2, li: lu as u8, entry: k2, slot: j as u16 }, q, node.g + node.pos.dist(q), idx);
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
                    let status = self.sub.tri_status[lu][t as usize];
                    let free = status == NONE;
                    let entry_pos = if entry < 3 { Some((CPos::Edge(entry), Some(slot as usize))) } else { None };
                    if let Some(pin) = self.tri_pin(lu, t)
                        && is_target(pin)
                    {
                        let q = self.board.pins[pin].pos;
                        push(&mut best, &mut nodes, &mut open, NK::GoalPin { pin }, q, node.g + node.pos.dist(q), idx);
                        continue;
                    }
                    // through hole copper of the net: switch layers for free
                    if !free && status != OUTSIDE && self.board.obstacles[status as usize].hole.is_some() {
                        for l2 in 0..nl {
                            if l2 != lu && self.sub.tri_status[l2][t as usize] == status {
                                push(&mut best, &mut nodes, &mut open, NK::Tri { t, li: l2 as u8, entry: 3, slot: 0 }, node.pos, node.g, idx);
                            }
                        }
                    }
                    for k in 0..3u8 {
                        if k != entry {
                            cross_out(&mut best, &mut nodes, &mut open, t, lu, k, entry_pos, None, &node, idx);
                        }
                    }
                    if !free || !self.all_free(t) {
                        continue;
                    }
                    // existing via vertices at the corners
                    for i in 0..3u8 {
                        let v = self.sub.tris[t as usize][i as usize];
                        let Some(existing) = self.via_ok_vertex(v, net) else { continue };
                        if let Some((ep, es)) = entry_pos
                            && !self.chord_ok(t, lu, net, ep, es, CPos::Vertex(i), None)
                        {
                            continue;
                        }
                        let q = self.sub.verts[v as usize];
                        let d = node.pos.dist(q);
                        let vc = if existing { 0.1 } else { self.cfg.via_cost };
                        if conn.goal == Goal::Plane && self.in_plane(q, net) {
                            push(&mut best, &mut nodes, &mut open, NK::GoalVia { v }, q, node.g + d + vc * 0.5, idx);
                            continue;
                        }
                        for l2 in 0..nl {
                            if l2 != lu {
                                push(&mut best, &mut nodes, &mut open, NK::Vert { v, li: l2 as u8 }, q, node.g + d + vc, idx);
                            }
                        }
                    }
                    // new vias on sites inside the triangle (nearest few)
                    let sites = &self.sub.tri_sites[t as usize];
                    if sites.is_empty() {
                        continue;
                    }
                    let mut cand: Vec<(f64, u32)> = sites.iter().map(|&s| (self.sub.shared.sites[s as usize].p.dist(node.pos), s)).collect();
                    cand.sort_by(|a, b| a.0.total_cmp(&b.0));
                    let mut tried = 0;
                    for (d, site) in cand {
                        if tried >= self.cfg.sites_per_tri {
                            break;
                        }
                        let q = self.sub.shared.sites[site as usize].p;
                        if let Some((ep, es)) = entry_pos
                            && !self.half_chord_ok(t, lu, net, q, ep, es)
                        {
                            continue;
                        }
                        if !self.via_ok_site(t, site, net) {
                            continue;
                        }
                        tried += 1;
                        if conn.goal == Goal::Plane && self.in_plane(q, net) {
                            push(&mut best, &mut nodes, &mut open, NK::GoalSite { t, site }, q, node.g + d + self.cfg.via_cost * 0.5, idx);
                            continue;
                        }
                        for l2 in 0..nl {
                            if l2 != lu {
                                push(&mut best, &mut nodes, &mut open, NK::Site { t, site, li: l2 as u8 }, q, node.g + d + self.cfg.via_cost, idx);
                            }
                        }
                    }
                }
                NK::Vert { v, li } => {
                    let lu = li as usize;
                    for &t in &self.sub.vert_tris[v as usize] {
                        let status = self.sub.tri_status[lu][t as usize];
                        if !Substrate::passable(self.board, status, net) {
                            continue;
                        }
                        let i = self.sub.tris[t as usize].iter().position(|&x| x == v).unwrap() as u8;
                        let k = (i + 1) % 3;
                        cross_out(&mut best, &mut nodes, &mut open, t, lu, k, Some((CPos::Vertex(i), None)), None, &node, idx);
                    }
                }
                NK::Site { t, li, .. } => {
                    let lu = li as usize;
                    for k in 0..3u8 {
                        cross_out(&mut best, &mut nodes, &mut open, t, lu, k, None, Some(node.pos), &node, idx);
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
        let mut placed: Vec<(u32, u32)> = Vec::new(); // (old triangle, site)
        for i in 0..chain.len() {
            let (t, site, li) = match chain[i].kind {
                NK::Site { t, site, li } => (t, site, Some(li)),
                NK::GoalSite { t, site } => (t, site, None),
                _ => continue,
            };
            if placed.iter().any(|&(_, s)| s == site) {
                continue;
            }
            let (v, x) = self.split_at(t, site);
            placed.push((t, site));
            chain[i].kind = match li {
                Some(li) => NK::Vert { v, li },
                None => NK::GoalVia { v },
            };
            // the arriving triangle node: old t entered through edge k -> X_k entered through 0
            if i > 0
                && let NK::Tri { t: tp, li: lp, entry, slot } = chain[i - 1].kind
                && tp == t
                && entry < 3
            {
                chain[i - 1].kind = NK::Tri { t: x[entry as usize], li: lp, entry: 0, slot };
            }
        }
        // every consecutive pair of triangle nodes must still be adjacent
        for w in chain.windows(2) {
            if let (NK::Tri { t, li, .. }, NK::Tri { t: t2, li: l2, entry, .. }) = (w[0].kind, w[1].kind)
                && li == l2
                && entry < 3
                && !self.sub.tri_edges[t as usize].contains(&self.sub.tri_edges[t2 as usize][entry as usize])
            {
                return false;
            }
            if let NK::Tri { t, .. } = w[0].kind
                && placed.iter().any(|&(old, _)| old == t)
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

        // Split the chain into runs of triangle nodes on one layer.
        let mut i = 0;
        let mut start = End::Pin(conn.from);
        while i < chain.len() {
            let NK::Tri { li, .. } = chain[i].kind else {
                if let NK::Vert { v, .. } = chain[i].kind {
                    start = End::Vertex(v);
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
                Some(NK::Vert { v, .. }) | Some(NK::GoalVia { v }) => {
                    self.add_via(v, net);
                    via_ids.push(v);
                    End::Vertex(v)
                }
                Some(NK::GoalPin { pin }) => End::Pin(pin),
                // through hole layer change: the run ends at that pin
                _ => End::Pin(self.tri_pin(li as usize, last_t).unwrap_or(conn.from)),
            };
            let seg = Seg { net, conn: ci as u32, li, width, clearance, start, end, crossings: Vec::new(), chord_tris: Vec::new() };
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
        let vpos = |t: u32, v: u32| CPos::Vertex(self.sub.tris[t as usize].iter().position(|&x| x == v).unwrap_or(0) as u8);
        let mut chords: Vec<(u32, Chord)> = Vec::new();
        // leaving a via vertex: the chord in the triangle before the first crossing
        if let (End::Vertex(v), Some(&(t0, e0, _))) = (s.start, run.first())
            && e0 < 3
        {
            let e = self.sub.tri_edges[t0 as usize][e0 as usize];
            let from = self.sub.other_tri(e, t0);
            if from != NONE && self.sub.tri_status[li][from as usize] == NONE && self.sub.tris[from as usize].contains(&v) {
                chords.push((from, Chord { seg: id, a: vpos(from, v), b: CPos::Edge(self.sub.local_edge(from, e) as u8) }));
            }
        }
        for (idx, &(t, entry, _)) in run.iter().enumerate() {
            if self.sub.tri_status[li][t as usize] != NONE {
                continue;
            }
            let a = if entry < 3 {
                CPos::Edge(entry)
            } else if let End::Vertex(v) = s.start {
                vpos(t, v)
            } else {
                continue;
            };
            let b = if let Some(&(t2, e2, _)) = run.get(idx + 1) {
                if e2 >= 3 {
                    continue;
                }
                CPos::Edge(self.sub.local_edge(t, self.sub.tri_edges[t2 as usize][e2 as usize]) as u8)
            } else if let End::Vertex(v) = s.end {
                vpos(t, v)
            } else {
                continue;
            };
            chords.push((t, Chord { seg: id, a, b }));
        }
        for &(t, entry, slot) in run {
            if entry >= 3 {
                continue;
            }
            let e = self.sub.tri_edges[t as usize][entry as usize];
            let from = self.sub.other_tri(e, t);
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
            let Some(s) = self.segs[sid as usize].take() else { continue };
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
            if has_plane {
                for &p in &n.pins {
                    let pin = &b.pins[p];
                    if b.planes.iter().any(|pl| pl.net == net && pin.layers.has(pl.layer)) {
                        continue; // through hole pin reaches the plane itself
                    }
                    let key = if self.cfg.fanout_first { i64::MIN / 2 + prio } else { prio };
                    conns.push((key, 0.0, Conn { net, from: p, goal: Goal::Plane }));
                }
                continue;
            }
            // Prim order: each new pin connects to the pins already in the tree
            let pins = &n.pins;
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
                conns.push((prio, d, Conn { net, from: pins[k], goal: Goal::Pins(tree) }));
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

    /// Route all planned connections. Returns the number routed.
    pub fn run(&mut self) -> usize {
        let mut failed = Vec::new();
        for ci in 0..self.conns.len() {
            if !self.route_one(ci) {
                failed.push(ci);
            }
        }
        // Fan-outs that found no plane: connect to the nearest pin of the net instead.
        for &ci in &failed {
            if self.conns[ci].goal == Goal::Plane {
                let c = &self.conns[ci];
                let pins: Vec<usize> = self.board.nets[c.net as usize].pins.iter().copied().filter(|&p| p != c.from).collect();
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
                    if self.commit(ci, &chain) {
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
            Goal::Pins(p) => p.iter().map(|&q| self.board.pins[q].pos).min_by(|x, y| x.dist(a).total_cmp(&y.dist(a))).unwrap_or(a),
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
                s.crossings.iter().any(|cr| {
                    let [u, v] = self.sub.edges[cr.e as usize];
                    let m = (self.sub.verts[u as usize] + self.sub.verts[v as usize]) * 0.5;
                    point_seg_dist(m, a, b) < radius
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
        (0..self.conns.len()).filter(|&ci| self.conn_segs[ci].is_empty()).collect()
    }
}

/// Intersection of segments ab and cd: (parameter on ab, parameter on cd).
fn seg_param_intersection(a: Point, b: Point, c: Point, d: Point) -> Option<(f64, f64)> {
    let r = b - a;
    let s = d - c;
    let den = r.cross(s);
    if den.abs() < 1e-15 {
        return None;
    }
    let t = (c - a).cross(s) / den;
    let u = (c - a).cross(r) / den;
    if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) { Some((t, u)) } else { None }
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

impl Router<'_> {
    /// Consistency checks of the topological state (debugging aid).
    pub fn validate(&self) -> Vec<String> {
        let mut out = Vec::new();
        let mut over = 0;
        for li in 0..self.sub.layers.len() {
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
            for w in s.crossings.windows(2) {
                let next_from = self.sub.other_tri(w[0].e, w[0].from);
                if next_from != w[1].from {
                    out.push(format!("seg {sid}: crossing chain broken at edge {} -> {}", w[0].e, w[1].e));
                    break;
                }
            }
            for c in &s.crossings {
                if !self.cross[s.li as usize][c.e as usize].contains(&(sid as u32)) {
                    out.push(format!("seg {sid}: missing in list of edge {}", c.e));
                }
                if !self.sub.tri_edges[c.from as usize].contains(&c.e) {
                    out.push(format!("seg {sid}: crossing from tri {} not on edge {}", c.from, c.e));
                }
            }
            if let (End::Vertex(v), Some(c)) = (s.start, s.crossings.first())
                && !self.sub.tris[c.from as usize].contains(&v)
            {
                out.push(format!("seg {sid}: first crossing not at start via"));
            }
            if let (End::Vertex(v), Some(c)) = (s.end, s.crossings.last()) {
                let last = self.sub.other_tri(c.e, c.from);
                if !self.sub.tris[last as usize].contains(&v) {
                    out.push(format!("seg {sid}: last crossing not at end via"));
                }
            }
        }
        out
    }
}
