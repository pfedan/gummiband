//! Proof that a search finds no way, from the goal side.
//!
//! A search that fails walks every state it can reach from its start before it gives up. On a
//! full board that is most of the board, while the goal sits shut in by the tracks of other nets
//! a few triangles around it. Seen from the goal, the places a way into the goal could come from
//! are few: if none of them is a start of the search, the search can only fail, and it may stop.
//!
//! The places are those of the topology: the gaps between the crossings on an edge, the via
//! vertices and via sites, and the pads of the net. Two of them belong together if a track of the
//! net could go from one to the other in one triangle without crossing the chord of another net
//! (as `chord_ok` and `half_chord_ok` decide in the search). Everything else is looser than the
//! search: every gap that the edge has room for with the loosest sides, any via site and via
//! vertex, any layer, through holes. So whatever the search can reach is reached here as well,
//! and a goal side that never meets a start proves that the search fails. The result of a search
//! stays exactly the same; it only ends sooner.

use std::collections::{HashMap, HashSet};

use super::{CPos, Goal, Item, Router};
use crate::geom::{Point, Rect, segs_intersect};
use crate::topo::{NONE, OUTSIDE, PAD_SITE, STEINER, Substrate};

/// A place of the topology (see the module documentation).
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
enum Place {
    /// Gap `j` (before crossing `j`) on edge `e` of layer `l`.
    Gap { l: u8, e: u32, j: u32 },
    /// A triangle covered by an obstacle of the net (a pad, kept copper): one place.
    Pad { l: u8, t: u32 },
    /// The via vertex `v` of layer `l`.
    Vertex { l: u8, v: u32 },
    /// The unused via site `s` on layer `l`.
    Site { l: u8, s: u32 },
}

/// A boundary position of a triangle and its point, for the chord tests.
#[derive(Clone, Copy)]
struct Pos {
    coord: f64,
    at: Point,
}

/// What the walk from the goal keeps: the room of the edges and scratch for the chord tests.
struct Walk<'n> {
    net: crate::board::NetId,
    item: Item,
    necks: &'n [Rect],
    neck_w: f64,
    room: HashMap<(u8, u32), Vec<bool>>,
    scratch: Vec<f64>,
    coords: Vec<(f64, f64)>,
    points: Vec<(Point, Point)>,
    /// Can a new via of the net go at a site, at an unused via vertex (as the search asks).
    site_ok: HashMap<u32, bool>,
    vertex_ok: HashMap<u32, bool>,
}

impl Router<'_> {
    /// True if the search of connection `ci` (with the item, necks and via in pad it searches
    /// with) can only fail; false if it may find a way, or this cannot tell within `budget`
    /// places, for a plane as goal, taps and soft capacity.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn goal_shut_in(
        &self,
        ci: usize,
        item: Item,
        necks: &[Rect],
        neck_w: f64,
        tappable: &HashSet<u32>,
        via_in_pad: bool,
        budget: usize,
    ) -> bool {
        let conn = &self.conns[ci];
        let Goal::Pins(goal) = &conn.goal else { return false };
        if self.soft() || self.soft_on || !tappable.is_empty() {
            return false;
        }
        let nl = self.nl();
        let mut w = Walk {
            net: conn.net,
            item,
            necks,
            neck_w,
            room: HashMap::new(),
            scratch: Vec::new(),
            coords: Vec::new(),
            points: Vec::new(),
            site_ok: HashMap::new(),
            vertex_ok: HashMap::new(),
        };
        // the pads of a group of pins and the places of a via in them
        let in_groups = |groups: &[u32]| -> Vec<usize> {
            let groups: HashSet<u32> = groups.iter().copied().collect();
            (0..self.board.pins.len())
                .filter(|&q| groups.contains(&self.pin_group[q]))
                .collect()
        };
        let pads = |q: usize, out: &mut Vec<Place>| {
            let pin = &self.board.pins[q];
            // every layer: an obstacle of the pin can reach a layer the pin itself does not
            // name (the merged ends of a pair), and the search ends there as well
            for (l, m) in self.sub.m.iter().enumerate() {
                for &o in &pin.obstacles {
                    for &t in m.obs_tris.get(&(o as u32)).map(|v| v.as_slice()).unwrap_or(&[]) {
                        out.push(Place::Pad { l: l as u8, t });
                    }
                }
            }
            if via_in_pad {
                for &s in self
                    .sub
                    .shared
                    .pad_sites
                    .get(&(q as u32))
                    .map(|v| v.as_slice())
                    .unwrap_or(&[])
                {
                    for (l, m) in self.sub.m.iter().enumerate() {
                        out.push(Place::Site { l: l as u8, s });
                        if let Some(&v) = m.site_vert.get(&s) {
                            out.push(Place::Vertex { l: l as u8, v });
                        }
                    }
                }
            }
        };
        let mut start = Vec::new();
        for q in in_groups(&[self.pin_group[conn.from]]) {
            pads(q, &mut start);
        }
        let start: HashSet<Place> = start.into_iter().collect();
        let groups: Vec<u32> = goal.iter().map(|&q| self.pin_group[q]).collect();
        let mut seeds = Vec::new();
        for q in in_groups(&groups) {
            pads(q, &mut seeds);
        }
        // a goal as large as the budget (all other pins of a large net) cannot be shut in here
        if seeds.len() > budget {
            return false;
        }
        let mut seen: HashSet<Place> = HashSet::new();
        let mut queue = Vec::new();
        for p in seeds {
            let usable = match p {
                Place::Pad { l, t } => self.walk_ok(&w, l as usize, t),
                _ => true,
            };
            if usable && seen.insert(p) {
                queue.push(p);
            }
        }
        let mut next: Vec<Place> = Vec::new();
        let mut at = 0;
        while at < queue.len() {
            let place = queue[at];
            at += 1;
            if start.contains(&place) || seen.len() > budget {
                return false;
            }
            next.clear();
            match place {
                Place::Gap { l, e, j } => {
                    let (l, m) = (l as usize, self.m(l as usize));
                    for &t in &m.edge_tris[e as usize] {
                        if t == NONE {
                            continue;
                        }
                        let k = CPos::Edge(m.local_edge(t, e) as u8);
                        let from = Pos {
                            coord: self.coord(t, l, k, None, Some(j as usize)),
                            at: self.pos_point(t, l, k, None, Some(j as usize)),
                        };
                        self.walk_enter(&mut w, l, t, Ok(from), &mut next);
                    }
                }
                Place::Pad { l, t } => {
                    let (l, m) = (l as usize, self.m(l as usize));
                    let status = m.status[t as usize];
                    for k in 0..3 {
                        let t2 = m.tri_nbrs[t as usize][k];
                        if t2 == NONE || !self.walk_ok(&w, l, t2) {
                            continue;
                        }
                        let e = m.tri_edges[t as usize][k];
                        if m.status[t2 as usize] != NONE {
                            next.push(Place::Pad { l: l as u8, t: t2 });
                        }
                        for (j, &fits) in self.walk_gaps(&mut w, l, e).iter().enumerate() {
                            if fits {
                                next.push(Place::Gap {
                                    l: l as u8,
                                    e,
                                    j: j as u32,
                                });
                            }
                        }
                    }
                    for &v in &m.tris[t as usize] {
                        if self.walk_vertex(&mut w, l, v) {
                            next.push(Place::Vertex { l: l as u8, v });
                        }
                    }
                    if status != OUTSIDE && self.board.obstacles[status as usize].hole.is_some() {
                        for l2 in (0..nl).filter(|&l2| l2 != l) {
                            for &t2 in self.m(l2).obs_tris.get(&status).map(|v| v.as_slice()).unwrap_or(&[]) {
                                if self.walk_ok(&w, l2, t2) {
                                    next.push(Place::Pad { l: l2 as u8, t: t2 });
                                }
                            }
                        }
                    }
                }
                Place::Vertex { l, v } => {
                    let (l, m) = (l as usize, self.m(l as usize));
                    for &t in &m.vert_tris[v as usize] {
                        let i = m.tris[t as usize].iter().position(|&x| x == v).unwrap();
                        let from = Pos {
                            coord: i as f64,
                            at: m.verts[v as usize],
                        };
                        self.walk_enter(&mut w, l, t, Ok(from), &mut next);
                    }
                    self.walk_other_layers(&mut w, l, m.vert_site[v as usize], &mut next);
                }
                Place::Site { l, s } => {
                    let l = l as usize;
                    let t = self.m(l).site_tri[s as usize];
                    if t != NONE && t != PAD_SITE {
                        self.walk_enter(&mut w, l, t, Err(self.site_point(s)), &mut next);
                    }
                    self.walk_other_layers(&mut w, l, s, &mut next);
                }
            }
            for &p in &next {
                if seen.insert(p) {
                    queue.push(p);
                }
            }
        }
        true
    }

    fn walk_ok(&self, w: &Walk, l: usize, t: u32) -> bool {
        let m = self.m(l);
        t != NONE && Substrate::passable(self.board, m.status[t as usize], w.net) && !m.closed(t, w.net)
    }

    /// A via vertex the net may use: a via of the net, or one where it may place a via.
    fn walk_vertex(&self, w: &mut Walk, l: usize, v: u32) -> bool {
        let s = self.m(l).vert_site[v as usize];
        if s == NONE || s == STEINER {
            return false;
        }
        let net = w.net;
        *w.vertex_ok
            .entry(s)
            .or_insert_with(|| self.via_ok_vertex(s, net).is_some())
    }

    /// An unused via site where the net may place a via.
    fn walk_site(&self, w: &mut Walk, s: u32) -> bool {
        let net = w.net;
        *w.site_ok.entry(s).or_insert_with(|| self.via_ok_site(s, net))
    }

    /// The gaps of an edge that have room for the track, with the loosest sides.
    fn walk_gaps(&self, w: &mut Walk, l: usize, e: u32) -> Vec<bool> {
        if let Some(r) = w.room.get(&(l as u8, e)) {
            return r.clone();
        }
        let mut it = w.item;
        it.w = self.width_at_edge(l, e, w.item.w, w.necks, w.neck_w);
        let view = self.edge_view(l, e, &it);
        let r: Vec<bool> = (0..=self.cross[l][e as usize].len())
            .map(|j| Self::slot_interval(&view, [2, 2], j, &mut w.scratch).is_some())
            .collect();
        w.room.insert((l as u8, e), r.clone());
        r
    }

    /// The same via site or vertex on the other layers.
    fn walk_other_layers(&self, w: &mut Walk, l: usize, s: u32, next: &mut Vec<Place>) {
        for l2 in (0..self.nl()).filter(|&l2| l2 != l) {
            let m2 = self.m(l2);
            if let Some(&v2) = m2.site_vert.get(&s)
                && self.walk_vertex(w, l2, v2)
            {
                next.push(Place::Vertex { l: l2 as u8, v: v2 });
            }
            let t2 = m2.site_tri[s as usize];
            if t2 != NONE && t2 != PAD_SITE && self.walk_site(w, s) {
                next.push(Place::Site { l: l2 as u8, s });
            }
        }
    }

    /// A place in triangle t of layer l: the triangle as a whole if it is a pad, else the
    /// places of t that a track from `from` (a boundary position, or an interior point) reaches
    /// without crossing a chord of another net.
    fn walk_enter(&self, w: &mut Walk, l: usize, t: u32, from: Result<Pos, Point>, next: &mut Vec<Place>) {
        let m = self.m(l);
        if t == NONE || !Substrate::passable(self.board, m.status[t as usize], w.net) {
            return;
        }
        if m.status[t as usize] != NONE {
            next.push(Place::Pad { l: l as u8, t });
            return;
        }
        // a triangle in the mask opening of another net: the search never enters it across an
        // edge, but a via placed on one of its sites leaves it across its edges
        let closed = m.closed(t, w.net);
        let net = w.net;
        let mut coords = std::mem::take(&mut w.coords);
        let mut points = std::mem::take(&mut w.points);
        self.chord_coords(t, l, net, &mut coords);
        self.chord_points(t, l, net, &mut points);
        let reaches = |to: Pos| match from {
            Ok(p) => Self::chord_fits(&coords, p.coord, to.coord),
            Err(p) => !points.iter().any(|&(c, d)| segs_intersect(p, to.at, c, d)),
        };
        for k in 0..3u8 {
            // a gap toward a triangle the search cannot enter is no state of the search
            let t2 = m.tri_nbrs[t as usize][k as usize];
            if (closed && from.is_ok()) || !self.walk_ok(w, l, t2) {
                continue;
            }
            let e = m.tri_edges[t as usize][k as usize];
            for (j, &fits) in self.walk_gaps(w, l, e).iter().enumerate() {
                if !fits {
                    continue;
                }
                let to = Pos {
                    coord: self.coord(t, l, CPos::Edge(k), None, Some(j)),
                    at: self.pos_point(t, l, CPos::Edge(k), None, Some(j)),
                };
                if reaches(to) {
                    next.push(Place::Gap {
                        l: l as u8,
                        e,
                        j: j as u32,
                    });
                }
            }
        }
        for i in 0..3u8 {
            let v = m.tris[t as usize][i as usize];
            let to = Pos {
                coord: i as f64,
                at: m.verts[v as usize],
            };
            if !closed && reaches(to) && self.walk_vertex(w, l, v) {
                next.push(Place::Vertex { l: l as u8, v });
            }
        }
        if let Ok(p) = from {
            for &s in &m.tri_sites[t as usize] {
                let q = self.site_point(s);
                if !points.iter().any(|&(c, d)| segs_intersect(q, p.at, c, d)) && self.walk_site(w, s) {
                    next.push(Place::Site { l: l as u8, s });
                }
            }
        }
        w.coords = coords;
        w.points = points;
    }
}
