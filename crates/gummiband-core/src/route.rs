//! Topological router: A* over triangle edges with slot positions. Every route segment on a
//! layer is a chain of edge crossings; inside a free triangle it is a chord between two boundary
//! positions, and no two chords of different nets may interleave. So each layer stays planar
//! and the realization can pull every segment taut without crossings.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

use crate::board::{Board, NetId, ObstacleKind};
use crate::geom::{Point, point_seg_dist};
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
}

impl Default for RouterConfig {
    fn default() -> Self {
        RouterConfig {
            via_cost: 1.5,
            fanout_first: true,
            max_expansions: 400_000,
            seed: 0,
            ripup_passes: 3,
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

/// Mutable routing state on top of a shared substrate.
#[derive(Clone)]
pub struct Router<'a> {
    pub board: &'a Board,
    pub sub: &'a Substrate,
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
    Tri {
        t: u32,
        li: u8,
        entry: u8,
        slot: u16,
    },
    /// At vertex v (a via) on layer li.
    Vert {
        v: u32,
        li: u8,
    },
    GoalPin {
        pin: usize,
    },
    GoalVia {
        v: u32,
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
}

impl<'a> Router<'a> {
    pub fn new(board: &'a Board, sub: &'a Substrate, cfg: RouterConfig) -> Router<'a> {
        let nl = sub.layers.len();
        Router {
            board,
            sub,
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
        if obs == OUTSIDE {
            None
        } else {
            self.board.obstacles[obs as usize].net
        }
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

    fn item_of(&self, seg: u32) -> Item {
        let s = self.segs[seg as usize].as_ref().unwrap();
        Item {
            net: s.net,
            w: s.width,
            c: s.clearance,
        }
    }

    /// Centre line intervals of all items on edge e (u to v), or None if they do not fit.
    pub fn pack(
        &self,
        li: usize,
        e: u32,
        items: &[Item],
        extra_via: Option<(u32, &ViaInfo)>,
    ) -> Option<Vec<(f64, f64)>> {
        let len = self.sub.edge_len(e);
        let n = items.len();
        let [u, v] = self.sub.edges[e as usize];
        let via_need = |vert: u32, it: &Item| -> f64 {
            match extra_via {
                Some((vv, via)) if vv == vert && via.net != it.net => via.r + via.clearance.max(it.c) + it.w / 2.0,
                _ => 0.0,
            }
        };
        let gap = |a: &Item, b: &Item| -> f64 {
            if a.net == b.net {
                0.0
            } else {
                a.w / 2.0 + a.c.max(b.c) + b.w / 2.0
            }
        };
        let mut lo = vec![0.0; n];
        let mut hi = vec![0.0; n];
        for i in 0..n {
            let mut x = self.end_need(li, e, 0, &items[i]).max(via_need(u, &items[i]));
            if i > 0 {
                x = x.max(lo[i - 1] + gap(&items[i - 1], &items[i]));
            }
            lo[i] = x;
        }
        for i in (0..n).rev() {
            let mut y = len - self.end_need(li, e, 1, &items[i]).max(via_need(v, &items[i]));
            if i + 1 < n {
                y = y.min(hi[i + 1] - gap(&items[i], &items[i + 1]));
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

    /// Interval for a new item at slot j of edge e.
    fn slot_interval(&self, li: usize, e: u32, j: usize, it: Item) -> Option<(f64, f64)> {
        let list = &self.cross[li][e as usize];
        let mut items: Vec<Item> = Vec::with_capacity(list.len() + 1);
        for (i, &s) in list.iter().enumerate() {
            if i == j {
                items.push(it);
            }
            items.push(self.item_of(s));
        }
        if j == list.len() {
            items.push(it);
        }
        self.pack(li, e, &items, None).map(|v| v[j])
    }

    // ------------------------------------------------------------------ chords

    /// Cyclic boundary coordinate of a position in triangle t. `slot` is the insertion slot of a
    /// new crossing on that edge; otherwise the position of `seg` in the edge list is used.
    fn coord(&self, t: u32, li: usize, p: CPos, seg: Option<u32>, slot: Option<usize>) -> f64 {
        match p {
            CPos::Vertex(i) => i as f64,
            CPos::Edge(k) => {
                let e = self.sub.tri_edges[t as usize][k as usize];
                let list = &self.cross[li][e as usize];
                let n = list.len() as f64;
                let f = match (slot, seg) {
                    (Some(j), _) => (j as f64 + 0.5) / (n + 1.0),
                    (None, Some(s)) => {
                        let idx = list.iter().position(|&x| x == s).unwrap_or(0) as f64;
                        (idx + 1.0) / (n + 1.0)
                    }
                    _ => 0.5,
                };
                let forward = self.sub.tris[t as usize][k as usize] == self.sub.edges[e as usize][0];
                k as f64 + if forward { f } else { 1.0 - f }
            }
        }
    }

    /// Can a chord of `net` from `a` (slot sa) to `b` (slot sb) be added to triangle t?
    fn chord_ok(&self, t: u32, li: usize, net: NetId, a: CPos, sa: Option<usize>, b: CPos, sb: Option<usize>) -> bool {
        let Some(list) = self.chords[li].get(&t) else {
            return true;
        };
        let x = self.coord(t, li, a, None, sa);
        let y = self.coord(t, li, b, None, sb);
        let (lo, hi) = if x < y { (x, y) } else { (y, x) };
        for ch in list {
            let s = self.segs[ch.seg as usize].as_ref().unwrap();
            if s.net == net {
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

    /// Can a via of `net` be placed at site v? Ok(true) if a via of the net is already there.
    pub fn via_ok(&self, v: u32, net: NetId) -> Option<bool> {
        if !self.sub.is_site[v as usize] {
            return None;
        }
        if let Some(via) = self.vias.get(&v) {
            return if via.net == net { Some(true) } else { None };
        }
        let (r, drill, clr) = self.via_size(net);
        let p = self.sub.verts[v as usize];
        if let Some(nbrs) = self.sub.site_nbrs.get(&v) {
            for n in nbrs {
                let o = &self.board.obstacles[n.obs as usize];
                let need = if o.net == Some(net) && o.kind != ObstacleKind::Hole {
                    match o.hole {
                        Some(_) => drill / 2.0 + self.board.rules.hole_to_hole,
                        None => 0.0,
                    }
                } else {
                    r + self.eff_clearance(n.obs, clr)
                };
                if (n.dist as f64) < need {
                    return None;
                }
            }
        }
        if self.board.edge_dist(p) < r + self.board.rules.edge_clearance {
            return None;
        }
        let (cx, cy) = Self::via_cell(p);
        for dx in -2..=2 {
            for dy in -2..=2 {
                if let Some(list) = self.via_cells.get(&(cx + dx, cy + dy)) {
                    for &w in list {
                        let o = &self.vias[&w];
                        let d = p.dist(self.sub.verts[w as usize]);
                        let need = if o.net == net {
                            drill / 2.0 + o.drill / 2.0 + self.board.rules.hole_to_hole
                        } else {
                            (r + o.r + clr.max(o.clearance))
                                .max(drill / 2.0 + o.drill / 2.0 + self.board.rules.hole_to_hole)
                        };
                        if d < need {
                            return None;
                        }
                    }
                }
            }
        }
        // existing tracks on the incident edges must still fit
        let info = ViaInfo {
            net,
            r,
            drill,
            clearance: clr,
            users: 0,
        };
        for &t in &self.sub.vert_tris[v as usize] {
            for &e in &self.sub.tri_edges[t as usize] {
                let [a, b] = self.sub.edges[e as usize];
                if a != v && b != v {
                    continue;
                }
                for li in 0..self.sub.layers.len() {
                    let list = &self.cross[li][e as usize];
                    if list.is_empty() {
                        continue;
                    }
                    let items: Vec<Item> = list.iter().map(|&s| self.item_of(s)).collect();
                    self.pack(li, e, &items, Some((v, &info)))?;
                }
            }
        }
        Some(false)
    }

    fn in_plane(&self, v: u32, net: NetId) -> bool {
        let p = self.sub.verts[v as usize];
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
        self.vias.insert(
            v,
            ViaInfo {
                net,
                r,
                drill,
                clearance,
                users: 1,
            },
        );
        self.via_cells
            .entry(Self::via_cell(self.sub.verts[v as usize]))
            .or_default()
            .push(v);
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

    fn key(k: &NK) -> u64 {
        match *k {
            NK::Tri { t, li, entry, slot } => {
                ((t as u64) << 24) | ((li as u64) << 20) | ((entry as u64) << 16) | slot as u64
            }
            NK::Vert { v, li } => (1u64 << 63) | ((v as u64) << 8) | li as u64,
            NK::GoalPin { pin } => (1u64 << 62) | pin as u64,
            NK::GoalVia { v } => (3u64 << 61) | v as u64,
        }
    }

    fn tri_pin(&self, li: usize, t: u32) -> Option<usize> {
        let s = self.sub.tri_status[li][t as usize];
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
        };
        let src = &self.board.pins[conn.from];
        let targets: Vec<usize> = match &conn.goal {
            Goal::Pins(p) => p.clone(),
            Goal::Plane => Vec::new(),
        };
        let target_pts: Vec<Point> = targets.iter().map(|&p| self.board.pins[p].pos).collect();
        let h = |p: Point| -> f64 {
            if target_pts.is_empty() {
                0.0
            } else {
                target_pts.iter().map(|&q| q.dist(p)).fold(f64::INFINITY, f64::min)
            }
        };
        let is_target = |pin: usize| -> bool {
            match &conn.goal {
                Goal::Pins(p) => p.contains(&pin),
                Goal::Plane => {
                    let pn = &self.board.pins[pin];
                    pin != conn.from
                        && pn.net == net
                        && self
                            .board
                            .planes
                            .iter()
                            .any(|pl| pl.net == net && pn.layers.has(pl.layer))
                }
            }
        };

        let mut nodes: Vec<Node> = Vec::new();
        let mut best: HashMap<u64, f64> = HashMap::new();
        let mut open = BinaryHeap::new();
        let push = |best: &mut HashMap<u64, f64>,
                    nodes: &mut Vec<Node>,
                    open: &mut BinaryHeap<Open>,
                    kind: NK,
                    pos: Point,
                    g: f64,
                    parent: u32| {
            let k = Self::key(&kind);
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
                for &t in self.sub.obs_tris[li]
                    .get(&(o as u32))
                    .map(|v| v.as_slice())
                    .unwrap_or(&[])
                {
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

        let mut expansions = 0;
        while let Some(Open(_, idx)) = open.pop() {
            let node = nodes[idx as usize];
            if best.get(&Self::key(&node.kind)).is_some_and(|&b| b < node.g - 1e-9) {
                continue;
            }
            expansions += 1;
            if expansions > self.cfg.max_expansions {
                return None;
            }
            match node.kind {
                NK::GoalPin { .. } | NK::GoalVia { .. } => {
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
                    let entry_pos = if entry < 3 { Some(CPos::Edge(entry)) } else { None };
                    // reached a target pin?
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
                    // through hole copper of the net: switch layers for free
                    if !free && status != OUTSIDE {
                        let o = &self.board.obstacles[status as usize];
                        if o.hole.is_some() {
                            for (l2, &bl) in self.sub.layers.iter().enumerate() {
                                if l2 != lu && o.layers.has(bl) && self.sub.tri_status[l2][t as usize] == status {
                                    push(
                                        &mut best,
                                        &mut nodes,
                                        &mut open,
                                        NK::Tri {
                                            t,
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
                    }
                    // cross an edge
                    for k in 0..3u8 {
                        if k == entry {
                            continue;
                        }
                        let t2 = self.sub.tri_nbrs[t as usize][k as usize];
                        if t2 == NONE {
                            continue;
                        }
                        let s2 = self.sub.tri_status[lu][t2 as usize];
                        if !Substrate::passable(self.board, s2, net) {
                            continue;
                        }
                        let e = self.sub.tri_edges[t as usize][k as usize];
                        let k2 = self.sub.local_edge(t2, e) as u8;
                        if !free && s2 != NONE {
                            push(
                                &mut best,
                                &mut nodes,
                                &mut open,
                                NK::Tri {
                                    t: t2,
                                    li,
                                    entry: 3,
                                    slot: 0,
                                },
                                node.pos,
                                node.g,
                                idx,
                            );
                            continue;
                        }
                        let [u, v] = self.sub.edges[e as usize];
                        let (pu, pv) = (self.sub.verts[u as usize], self.sub.verts[v as usize]);
                        let n = self.cross[lu][e as usize].len();
                        for j in 0..=n {
                            if free && let Some(ep) = entry_pos {
                                if !self.chord_ok(t, lu, net, ep, Some(slot as usize), CPos::Edge(k), Some(j)) {
                                    continue;
                                }
                            }
                            let Some((lo, hi)) = self.slot_interval(lu, e, j, item) else {
                                continue;
                            };
                            let len = pu.dist(pv);
                            let dir = (pv - pu) * (1.0 / len);
                            let tproj = (node.pos - pu).dot(dir).clamp(lo, hi);
                            let q = pu + dir * tproj;
                            push(
                                &mut best,
                                &mut nodes,
                                &mut open,
                                NK::Tri {
                                    t: t2,
                                    li,
                                    entry: k2,
                                    slot: j as u16,
                                },
                                q,
                                node.g + node.pos.dist(q),
                                idx,
                            );
                        }
                    }
                    // via at a corner of a free triangle
                    if free {
                        for i in 0..3u8 {
                            let v = self.sub.tris[t as usize][i as usize];
                            let Some(existing) = self.via_ok(v, net) else { continue };
                            if let Some(ep) = entry_pos
                                && !self.chord_ok(t, lu, net, ep, Some(slot as usize), CPos::Vertex(i), None)
                            {
                                continue;
                            }
                            let q = self.sub.verts[v as usize];
                            let d = node.pos.dist(q);
                            let vc = if existing { 0.1 } else { self.cfg.via_cost };
                            if conn.goal == Goal::Plane && self.in_plane(v, net) {
                                push(
                                    &mut best,
                                    &mut nodes,
                                    &mut open,
                                    NK::GoalVia { v },
                                    q,
                                    node.g + d + vc * 0.5,
                                    idx,
                                );
                                continue;
                            }
                            for l2 in 0..self.sub.layers.len() {
                                if l2 != lu {
                                    push(
                                        &mut best,
                                        &mut nodes,
                                        &mut open,
                                        NK::Vert { v, li: l2 as u8 },
                                        q,
                                        node.g + d + vc,
                                        idx,
                                    );
                                }
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
                        let i = self.sub.tris[t as usize].iter().position(|&x| x == v).unwrap();
                        let k = ((i + 1) % 3) as u8;
                        let t2 = self.sub.tri_nbrs[t as usize][k as usize];
                        if t2 == NONE {
                            continue;
                        }
                        let s2 = self.sub.tri_status[lu][t2 as usize];
                        if !Substrate::passable(self.board, s2, net) {
                            continue;
                        }
                        let e = self.sub.tri_edges[t as usize][k as usize];
                        let k2 = self.sub.local_edge(t2, e) as u8;
                        let [u, w] = self.sub.edges[e as usize];
                        let (pu, pw) = (self.sub.verts[u as usize], self.sub.verts[w as usize]);
                        let n = self.cross[lu][e as usize].len();
                        for j in 0..=n {
                            if status == NONE
                                && !self.chord_ok(t, lu, net, CPos::Vertex(i as u8), None, CPos::Edge(k), Some(j))
                            {
                                continue;
                            }
                            let Some((lo, hi)) = self.slot_interval(lu, e, j, item) else {
                                continue;
                            };
                            let len = pu.dist(pw);
                            let dir = (pw - pu) * (1.0 / len);
                            let q = pu + dir * (node.pos - pu).dot(dir).clamp(lo, hi);
                            push(
                                &mut best,
                                &mut nodes,
                                &mut open,
                                NK::Tri {
                                    t: t2,
                                    li,
                                    entry: k2,
                                    slot: j as u16,
                                },
                                q,
                                node.g + node.pos.dist(q),
                                idx,
                            );
                        }
                    }
                }
            }
        }
        None
    }

    // ------------------------------------------------------------------ commit / rip-up

    fn commit(&mut self, ci: usize, chain: &[Node]) {
        let conn = self.conns[ci].clone();
        let net = conn.net;
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
            let seg = Seg {
                net,
                conn: ci as u32,
                li,
                width,
                clearance,
                start,
                end,
                crossings: Vec::new(),
                chord_tris: Vec::new(),
            };
            seg_ids.push(self.insert_seg(seg, &run));
            start = end;
            i = j;
        }
        self.conn_segs[ci] = seg_ids;
        self.conn_vias[ci] = via_ids;
    }

    /// Insert a segment: its chords (against the lists as the search saw them), then its
    /// crossings at their slots.
    fn insert_seg(&mut self, mut s: Seg, run: &[(u32, u8, u16)]) -> u32 {
        let id = self.segs.len() as u32;
        let li = s.li as usize;
        let vpos =
            |t: u32, v: u32| CPos::Vertex(self.sub.tris[t as usize].iter().position(|&x| x == v).unwrap_or(0) as u8);
        let mut chords: Vec<(u32, Chord)> = Vec::new();
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
            let from = *self.sub.edge_tris[e as usize].iter().find(|&&x| x != t).unwrap_or(&t);
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
            if has_plane {
                for &p in &n.pins {
                    let pin = &b.pins[p];
                    if b.planes.iter().any(|pl| pl.net == net && pin.layers.has(pl.layer)) {
                        continue; // through hole pin reaches the plane itself
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
                let pins: Vec<usize> = self.board.nets[c.net as usize]
                    .pins
                    .iter()
                    .copied()
                    .filter(|&p| p != c.from)
                    .collect();
                self.conns[ci].goal = Goal::Pins(pins);
            }
        }
        // Rip-up and reroute: move failed connections to the front, reroute the blockers after.
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
        self.conn_segs
            .iter()
            .enumerate()
            .filter(|(ci, s)| !s.is_empty() || self.conn_trivial(*ci))
            .count()
    }

    fn conn_trivial(&self, _ci: usize) -> bool {
        false
    }

    pub fn route_one(&mut self, ci: usize) -> bool {
        match self.search(ci) {
            Some(chain) => {
                self.commit(ci, &chain);
                true
            }
            None => false,
        }
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
        (0..self.conns.len())
            .filter(|&ci| self.conn_segs[ci].is_empty())
            .collect()
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
