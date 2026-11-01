//! Import of existing copper into the topology. With [`Board::soften`] the unlocked tracks and
//! vias of the nets that are not routed stop being obstacles: every run of track between two
//! nodes (via, pad, branch, change of width, open end) is traced through the layer mesh into the
//! same crossings and chords the router builds, in the order the tracks really have on each edge.
//! A new connection threads in between them under the usual capacity checks, and the relaxation
//! pushes aside what it touches. Everything it does not touch rests and stays in the file as it
//! is.

use crate::board::{Board, CopperKind, LayerId, LayerSet, NetId, ObstacleKind};
use crate::geom::{Point, arc_polyline, orient, point_seg_dist, segs_intersect};
use crate::index::{FastMap, GridIndex};
use crate::realize::{Elem, Geometry, Track, Via, flatten};
use crate::route::{CPos, Chord, Crossing, End, Item, Router, Seg};
use crate::topo::{Mesh, NONE, PAD_SITE};

/// The soft copper of a board.
#[derive(Debug, Clone, Default)]
pub struct SoftCopper {
    pub nodes: Vec<SoftNode>,
    pub chains: Vec<SoftChain>,
}

/// A mesh vertex soft tracks end on: a via, or (drill 0) a point on one layer where tracks branch
/// or end.
#[derive(Debug, Clone)]
pub struct SoftNode {
    pub at: Point,
    pub net: NetId,
    /// The layer of a branch point; a via has none.
    pub layer: Option<LayerId>,
    pub r: f64,
    pub drill: f64,
    pub clearance: f64,
    /// File items of the via.
    pub items: Vec<u32>,
    /// A change of track width: it stays where it is, a neck may be allowed only there.
    pub fixed: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SoftEnd {
    Node(u32),
    /// Inside a pad or other fixed copper of the net: the end stays where it is.
    Fixed,
}

/// A run of track of one width between two ends.
#[derive(Debug, Clone)]
pub struct SoftChain {
    pub net: NetId,
    pub layer: LayerId,
    pub width: f64,
    pub clearance: f64,
    pub pts: Vec<Point>,
    pub elems: Vec<Elem>,
    pub start: SoftEnd,
    pub end: SoftEnd,
    /// File items of the track.
    pub items: Vec<u32>,
}

#[derive(Clone, Copy)]
struct Piece {
    item: u32,
    a: Point,
    mid: Option<Point>,
    b: Point,
    width: f64,
    layer: LayerId,
    net: NetId,
}

#[derive(Default)]
struct RawNode {
    at: Point,
    net: NetId,
    layer: LayerId,
    via: Option<usize>,
    pad: bool,
    /// (piece, 0 = its start, 1 = its end)
    ends: Vec<(u32, u8)>,
}

impl Board {
    /// Turn the unlocked copper of all nets but `nets` into soft copper.
    pub fn soften(&mut self, nets: &[String]) {
        let routing = self.routing_layers();
        let keep: Vec<bool> = self.nets.iter().map(|n| nets.contains(&n.name)).collect();
        let mut pieces: Vec<Piece> = Vec::new();
        let mut vias: Vec<usize> = Vec::new();
        for (i, c) in self.copper.iter().enumerate() {
            let Some(net) = c.net else { continue };
            let first = c.first as usize;
            // what gave way already has no layers left
            if c.locked || keep[net as usize] || c.first == c.last || self.obstacles[first].layers.is_empty() {
                continue;
            }
            let (a, mid, b, width, layer) = match c.kind {
                CopperKind::Line { a, b, width, layer } => (a, None, b, width, layer),
                CopperKind::Arc {
                    a,
                    mid,
                    b,
                    width,
                    layer,
                } => (a, Some(mid), b, width, layer),
                CopperKind::Via { layers, .. } => {
                    if routing.0 & !layers.0 == 0 {
                        vias.push(i);
                    }
                    continue;
                }
            };
            // a neck below a minimum width may only be where the rules allow it: it stays
            if self.class_of(net).min_width.is_some_and(|w| width < w - 1e-9) {
                continue;
            }
            if routing.has(layer) && a.dist(b) > 1e-9 {
                pieces.push(Piece {
                    item: c.source,
                    a,
                    mid,
                    b,
                    width,
                    layer,
                    net,
                });
            }
        }
        for i in pieces.iter().map(|p| p.item).collect::<Vec<_>>() {
            let c = self.copper.iter().find(|c| c.source == i).unwrap();
            for o in c.first..c.last {
                self.obstacles[o as usize].layers = LayerSet::default();
            }
        }
        for &i in &vias {
            let c = &self.copper[i];
            for o in c.first..c.last {
                let o = &mut self.obstacles[o as usize];
                o.layers = LayerSet::default();
                o.hole = None;
            }
        }
        let mut raw = self.raw_nodes(&mut pieces, &vias);
        for n in &mut raw {
            n.pad = self.on_fixed_copper(n.at, n.net, n.layer);
        }
        self.split_branches(&mut pieces, &mut raw);
        self.soft = self.build_chains(&pieces, raw);
    }

    fn on_fixed_copper(&self, p: Point, net: NetId, layer: LayerId) -> bool {
        self.obstacles.iter().any(|o| {
            o.net == Some(net)
                && o.layers.has(layer)
                && !matches!(o.kind, ObstacleKind::Zone | ObstacleKind::Keepout { .. })
                && o.shape.bbox().contains(p)
                && o.shape.dist_to_point(p) <= 0.0
        })
    }

    /// Nodes at the ends of the pieces: an end inside a via of the net belongs to that via.
    fn raw_nodes(&self, pieces: &mut [Piece], vias: &[usize]) -> Vec<RawNode> {
        let mut raw: Vec<RawNode> = Vec::new();
        let mut at_via: Vec<(usize, Point, f64, NetId)> = Vec::new();
        for &i in vias {
            let c = &self.copper[i];
            if let CopperKind::Via { at, diameter, .. } = c.kind {
                at_via.push((raw.len(), at, diameter / 2.0, c.net.unwrap()));
                raw.push(RawNode {
                    at,
                    net: c.net.unwrap(),
                    via: Some(i),
                    ..Default::default()
                });
            }
        }
        let mut by_net: FastMap<NetId, Vec<(usize, Point, f64)>> = FastMap::default();
        for &(n, p, r, net) in &at_via {
            by_net.entry(net).or_default().push((n, p, r));
        }
        let mut exact: FastMap<(u32, u8, i64, i64), usize> = FastMap::default();
        for (pi, piece) in pieces.iter().enumerate() {
            for (end, p) in [(0u8, piece.a), (1, piece.b)] {
                let via = by_net.get(&piece.net).and_then(|l| {
                    l.iter()
                        .filter(|v| v.1.dist(p) < v.2)
                        .min_by(|x, y| x.1.dist(p).total_cmp(&y.1.dist(p)))
                });
                let n = match via {
                    Some(v) => v.0,
                    None => {
                        let key = (
                            piece.net,
                            piece.layer,
                            (p.x * 1e5).round() as i64,
                            (p.y * 1e5).round() as i64,
                        );
                        *exact.entry(key).or_insert_with(|| {
                            raw.push(RawNode {
                                at: p,
                                net: piece.net,
                                layer: piece.layer,
                                ..Default::default()
                            });
                            raw.len() - 1
                        })
                    }
                };
                raw[n].ends.push((pi as u32, end));
            }
        }
        raw
    }

    /// An open end on another track of the net is a branch: split that track there.
    fn split_branches(&self, pieces: &mut Vec<Piece>, raw: &mut Vec<RawNode>) {
        for n in 0..raw.len() {
            if raw[n].ends.len() != 1 || raw[n].via.is_some() || raw[n].pad {
                continue;
            }
            let (own, end) = raw[n].ends[0];
            let p = raw[n].at;
            let hit = (0..pieces.len()).find(|&q| {
                let o = &pieces[q];
                q != own as usize
                    && o.net == raw[n].net
                    && o.layer == raw[n].layer
                    && o.mid.is_none()
                    && point_seg_dist(p, o.a, o.b) <= o.width / 2.0
                    && o.a.dist(p) > 1e-6
                    && o.b.dist(p) > 1e-6
            });
            let Some(q) = hit else { continue };
            let o = pieces[q];
            let ab = o.b - o.a;
            let on = o.a + ab * ((p - o.a).dot(ab) / ab.len2()).clamp(0.0, 1.0);
            // the second half of the other track
            let second = pieces.len() as u32;
            pieces.push(Piece { a: on, ..o });
            pieces[q].b = on;
            for r in raw.iter_mut() {
                for e in r.ends.iter_mut() {
                    if *e == (q as u32, 1) {
                        *e = (second, 1);
                    }
                }
            }
            // the branch ends at the split point
            let (own_piece, own_end) = (own as usize, end);
            if own_end == 0 {
                pieces[own_piece].a = on;
            } else {
                pieces[own_piece].b = on;
            }
            raw[n].at = on;
            raw[n].ends.push((q as u32, 1));
            raw[n].ends.push((second, 0));
        }
    }

    fn build_chains(&self, pieces: &[Piece], mut raw: Vec<RawNode>) -> SoftCopper {
        let me = self.rules.max_error;
        let breaks = |n: &RawNode| {
            n.via.is_some()
                || n.pad
                || n.ends.len() != 2
                || pieces[n.ends[0].0 as usize].width != pieces[n.ends[1].0 as usize].width
        };
        let mut is_break: Vec<bool> = raw.iter().map(breaks).collect();
        let mut node_of_end: FastMap<(u32, u8), u32> = FastMap::default();
        for (n, r) in raw.iter().enumerate() {
            for &e in &r.ends {
                node_of_end.insert(e, n as u32);
            }
        }
        let mut soft = SoftCopper::default();
        let mut soft_node: Vec<Option<u32>> = vec![None; raw.len()];
        let mut done = vec![false; pieces.len()];
        let node_for = |k: usize, raw: &[RawNode], soft: &mut SoftCopper, soft_node: &mut Vec<Option<u32>>| -> u32 {
            let r = &raw[k];
            if let Some(n) = soft_node[k] {
                return n;
            }
            let clearance = self.class_of(r.net).clearance.max(self.rules.min_clearance);
            soft_node[k] = Some(soft.nodes.len() as u32);
            soft.nodes.push(match r.via {
                Some(i) => {
                    let c = &self.copper[i];
                    let CopperKind::Via { diameter, drill, .. } = c.kind else {
                        unreachable!()
                    };
                    SoftNode {
                        at: r.at,
                        net: r.net,
                        layer: None,
                        r: diameter / 2.0,
                        drill,
                        clearance,
                        items: vec![c.source],
                        fixed: false,
                    }
                }
                None => SoftNode {
                    at: r.at,
                    net: r.net,
                    layer: Some(r.layer),
                    r: r.ends
                        .iter()
                        .map(|e| pieces[e.0 as usize].width / 2.0)
                        .fold(0.0, f64::max),
                    drill: 0.0,
                    clearance,
                    items: Vec::new(),
                    fixed: r.ends.len() == 2,
                },
            });
            soft_node[k].unwrap()
        };
        let walk = |n0: usize,
                    e0: (u32, u8),
                    raw: &[RawNode],
                    is_break: &[bool],
                    soft: &mut SoftCopper,
                    soft_node: &mut Vec<Option<u32>>,
                    done: &mut [bool]| {
            let mut pts = vec![raw[n0].at];
            let mut elems = Vec::new();
            let mut items = Vec::new();
            let mut cur = e0;
            let width = pieces[e0.0 as usize].width;
            let n = loop {
                let pc = &pieces[cur.0 as usize];
                done[cur.0 as usize] = true;
                if !items.contains(&pc.item) {
                    items.push(pc.item);
                }
                let (a, b) = if cur.1 == 0 { (pc.a, pc.b) } else { (pc.b, pc.a) };
                let at = *pts.last().unwrap();
                if at.dist(a) > 1e-9 {
                    elems.push(Elem::Line(at, a));
                    pts.push(a);
                }
                match pc.mid {
                    Some(m) => {
                        elems.push(Elem::Arc(a, m, b));
                        pts.extend(arc_polyline(a, m, b, me).into_iter().skip(1));
                    }
                    None => {
                        elems.push(Elem::Line(a, b));
                        pts.push(b);
                    }
                }
                let other = (cur.0, 1 - cur.1);
                let n = node_of_end[&other] as usize;
                if is_break[n] {
                    break n;
                }
                cur = raw[n].ends.iter().copied().find(|&x| x != other).unwrap();
            };
            let at = raw[n].at;
            if pts.last().unwrap().dist(at) > 1e-9 {
                let last = *pts.last().unwrap();
                elems.push(Elem::Line(last, at));
                pts.push(at);
            }
            let (net, layer) = (pieces[e0.0 as usize].net, pieces[e0.0 as usize].layer);
            let clearance = self.class_of(net).clearance.max(self.rules.min_clearance);
            let mut end_of = |k: usize, soft: &mut SoftCopper| {
                if raw[k].via.is_none() && raw[k].pad {
                    SoftEnd::Fixed
                } else {
                    SoftEnd::Node(node_for(k, raw, soft, soft_node))
                }
            };
            let (start, end) = (end_of(n0, soft), end_of(n, soft));
            soft.chains.push(SoftChain {
                net,
                layer,
                width,
                clearance,
                pts,
                elems,
                start,
                end,
                items,
            });
        };
        for n in 0..raw.len() {
            if !is_break[n] {
                continue;
            }
            for e in raw[n].ends.clone() {
                if !done[e.0 as usize] {
                    walk(n, e, &raw, &is_break, &mut soft, &mut soft_node, &mut done);
                }
            }
        }
        // vias without tracks
        for (n, r) in raw.iter().enumerate() {
            if r.via.is_some() && r.ends.is_empty() {
                node_for(n, &raw, &mut soft, &mut soft_node);
            }
        }
        // closed rings have no break yet
        for n in 0..raw.len() {
            if raw[n].ends.iter().any(|e| !done[e.0 as usize]) {
                is_break[n] = true;
                raw[n].pad = false;
                for e in raw[n].ends.clone() {
                    if !done[e.0 as usize] {
                        walk(n, e, &raw, &is_break, &mut soft, &mut soft_node, &mut done);
                    }
                }
            }
        }
        soft
    }
}

/// A crossing of imported track: where it really is, and after which point of the track.
#[derive(Debug, Clone, Copy)]
pub struct Mark {
    pub p: Point,
    pub i: u32,
}

/// What an imported segment keeps of its copper.
#[derive(Debug, Clone)]
pub struct Imported {
    pub chain: u32,
    /// Parallel to the crossings of the segment.
    pub marks: Vec<Mark>,
    /// Per stretch between two marks (one more than marks): the stretch is a straight chord now
    /// and its original points no longer count.
    pub flat: Vec<bool>,
}

impl Imported {
    /// New crossings at index `at`, inside the stretch there: a via split its triangle.
    pub fn thread(&mut self, at: usize, new: &[Point], last: u32) {
        let i = self.marks.get(at).map_or(last, |m| m.i);
        for (j, &p) in new.iter().enumerate() {
            self.marks.insert(at + j, Mark { p, i });
            self.flat.insert(at, true);
        }
        self.flat[at] = true;
    }

    /// The track with its flat stretches.
    fn path(&self, pts: &[Point], flat: &[bool]) -> Vec<Point> {
        let n = self.marks.len();
        let mut out = vec![pts[0]];
        let mut from = 0usize;
        for k in 0..n {
            let to = self.marks[k].i as usize;
            if !flat[k] {
                out.extend_from_slice(&pts[(from + 1).min(to + 1)..=to.min(pts.len() - 1)]);
            }
            out.push(self.marks[k].p);
            from = to;
        }
        if !flat[n] && from + 1 < pts.len() - 1 {
            out.extend_from_slice(&pts[from + 1..pts.len() - 1]);
        }
        out.push(*pts.last().unwrap());
        out.dedup_by(|a, b| a.dist(*b) < 1e-9);
        out
    }
}

/// Which soft copper the routing changed.
#[derive(Debug, Default)]
pub struct SoftChanges {
    /// File items to remove.
    pub strip: Vec<u32>,
    /// Per chain and per node: written again.
    pub chains: Vec<bool>,
    pub nodes: Vec<bool>,
    /// Nets that moved, and the largest distance of a moved track from where it was.
    pub nets: Vec<NetId>,
    pub shift: f64,
}

/// What changed in `geo` against the soft copper of the board. File items do not always match
/// a chain one to one (a track split at a branch), so what shares an item with a changed chain
/// is written again as well.
pub fn soft_changes(board: &Board, geo: &Geometry) -> SoftChanges {
    let soft = &board.soft;
    let mut c = SoftChanges {
        chains: vec![false; soft.chains.len()],
        nodes: vec![false; soft.nodes.len()],
        ..Default::default()
    };
    for t in &geo.tracks {
        if let (Some(o), false) = (t.origin, t.resting) {
            c.chains[o as usize] = true;
        }
    }
    for v in &geo.vias {
        if let (Some(o), false) = (v.origin, v.resting) {
            c.nodes[o as usize] = true;
        }
    }
    loop {
        let strip: Vec<u32> = soft
            .chains
            .iter()
            .zip(&c.chains)
            .filter(|x| *x.1)
            .flat_map(|x| x.0.items.iter().copied())
            .collect();
        let mut more = false;
        for (ch, on) in soft.chains.iter().zip(c.chains.iter_mut()) {
            if !*on && ch.items.iter().any(|i| strip.contains(i)) {
                *on = true;
                more = true;
            }
        }
        if !more {
            c.strip = strip;
            break;
        }
    }
    c.strip.extend(
        soft.nodes
            .iter()
            .zip(&c.nodes)
            .filter(|x| *x.1)
            .flat_map(|x| x.0.items.iter().copied()),
    );
    c.strip.sort_unstable();
    c.strip.dedup();
    for t in &geo.tracks {
        let Some(o) = t.origin.filter(|&o| c.chains[o as usize]) else {
            continue;
        };
        let old = &soft.chains[o as usize].pts;
        let d = t
            .pts
            .iter()
            .map(|&p| {
                old.windows(2)
                    .map(|w| point_seg_dist(p, w[0], w[1]))
                    .fold(f64::INFINITY, f64::min)
            })
            .fold(0.0, f64::max);
        c.shift = c.shift.max(d);
        if !c.nets.contains(&t.net) {
            c.nets.push(t.net);
        }
    }
    c
}

const EPS: f64 = 1e-13;

struct Traced {
    crossings: Vec<Crossing>,
    marks: Vec<Mark>,
    /// Triangle of the end of the track.
    last: u32,
    /// Triangle of its start.
    first: u32,
}

impl Router<'_> {
    /// Thread the soft copper of the board into this router's meshes.
    pub(crate) fn import_soft(&mut self) {
        let board = self.board;
        let soft = &board.soft;
        self.soft_on = true;
        let base = self.sub.shared.soft_base as u32;
        for (i, n) in soft.nodes.iter().enumerate() {
            let site = base + i as u32;
            // right on the border of a triangle that was split for another node: no vertex
            for m in &mut self.sub.m {
                if m.site_tri[site as usize] == NONE {
                    m.site_tri[site as usize] = PAD_SITE;
                }
            }
            self.split_at(site);
            self.place_soft_via(site, n);
        }
        let index: Vec<GridIndex> = self.sub.m.iter().map(tri_index).collect();
        let mut lists: FastMap<(u8, u32), Vec<(f64, u32)>> = FastMap::default();
        for (ci, ch) in soft.chains.iter().enumerate() {
            let Some(li) = self.sub.layers.iter().position(|&l| l == ch.layer) else {
                continue;
            };
            self.import_chain(ci as u32, ch, li, &index[li], &mut lists);
        }
        for ((li, e), mut list) in lists {
            list.sort_by(|a, b| a.0.total_cmp(&b.0));
            self.cross[li as usize][e as usize] = list.into_iter().map(|x| x.1).collect();
        }
    }

    fn soft_vertex(&self, li: usize, end: SoftEnd) -> Option<u32> {
        let SoftEnd::Node(n) = end else { return None };
        let site = self.sub.shared.soft_base as u32 + n;
        self.sub.m[li].site_vert.get(&site).copied()
    }

    fn import_chain(
        &mut self,
        ci: u32,
        ch: &SoftChain,
        li: usize,
        index: &GridIndex,
        lists: &mut FastMap<(u8, u32), Vec<(f64, u32)>>,
    ) {
        let (vs, ve) = (self.soft_vertex(li, ch.start), self.soft_vertex(li, ch.end));
        let m = &self.sub.m[li];
        let mut pts = ch.pts.clone();
        // the mesh vertex of a node can lie a tiny bit off the node
        if let Some(v) = vs {
            pts[0] = m.verts[v as usize];
        }
        if let Some(v) = ve {
            *pts.last_mut().unwrap() = m.verts[v as usize];
        }
        let Some(tr) = trace(m, &pts, vs, index) else {
            self.soft_lost.push(ci);
            return;
        };
        let id = self.segs.len() as u32;
        let n = tr.crossings.len();
        let mut chords: Vec<(u32, Chord)> = Vec::new();
        let vpos = |t: u32, v: u32| {
            m.tris[t as usize]
                .iter()
                .position(|&x| x == v)
                .map(|i| CPos::Vertex(i as u8))
        };
        let edge_pos = |t: u32, e: u32| {
            m.tri_edges[t as usize]
                .iter()
                .position(|&x| x == e)
                .map(|k| CPos::Edge(k as u8))
        };
        let mut chord_tris = Vec::new();
        for k in 0..=n {
            let t = if k == n { tr.last } else { tr.crossings[k].from };
            if m.status[t as usize] != NONE {
                continue;
            }
            let a = if k == 0 {
                vs.and_then(|v| vpos(tr.first, v))
            } else {
                edge_pos(t, tr.crossings[k - 1].e)
            };
            let b = if k == n {
                ve.and_then(|v| vpos(t, v))
            } else {
                edge_pos(t, tr.crossings[k].e)
            };
            if let (Some(a), Some(b)) = (a, b) {
                chords.push((t, Chord { seg: id, a, b }));
                chord_tris.push(t);
            }
        }
        for (c, mk) in tr.crossings.iter().zip(&tr.marks) {
            let [u, v] = m.edges[c.e as usize];
            let (pu, pv) = (m.verts[u as usize], m.verts[v as usize]);
            let f = (mk.p - pu).dot(pv - pu) / (pv - pu).len2();
            lists.entry((li as u8, c.e)).or_default().push((f, id));
        }
        for (t, c) in chords {
            self.chords[li][t as usize].push(c);
        }
        let flat = vec![false; n + 1];
        let widths = vec![ch.width; n];
        self.segs.push(Some(Seg {
            net: ch.net,
            conn: u32::MAX,
            li: li as u8,
            width: ch.width,
            clearance: ch.clearance,
            start: end_of(ch.start, vs, self.sub.shared.soft_base, pts[0]),
            end: end_of(ch.end, ve, self.sub.shared.soft_base, *pts.last().unwrap()),
            crossings: tr.crossings,
            widths,
            chord_tris,
            necks: Vec::new(),
            neck_w: ch.width,
            imported: Some(Box::new(Imported {
                chain: ci,
                marks: tr.marks,
                flat,
            })),
        }));
    }

    /// Where imported `seg` crosses edge e, for its `nth` place in the list of the edge.
    fn mark_frac(&self, li: usize, e: u32, seg: u32, nth: usize) -> Option<f64> {
        let s = self.segs[seg as usize].as_ref()?;
        let imp = s.imported.as_ref()?;
        let m = &self.sub.m[li];
        let [u, v] = m.edges[e as usize];
        let (pu, pv) = (m.verts[u as usize], m.verts[v as usize]);
        let frac = |k: usize| ((imp.marks[k].p - pu).dot(pv - pu) / (pv - pu).len2()).clamp(0.0, 1.0);
        let mut on = s.crossings.iter().enumerate().filter(|(_, c)| c.e == e).map(|x| x.0);
        let first = on.next()?;
        if nth == 0 && on.next().is_none() {
            return Some(frac(first));
        }
        // the rare track that crosses an edge twice
        let mut all: Vec<f64> = s
            .crossings
            .iter()
            .enumerate()
            .filter(|(_, c)| c.e == e)
            .map(|x| frac(x.0))
            .collect();
        all.sort_by(f64::total_cmp);
        all.get(nth).copied()
    }

    /// Nearest imported crossings around the place `x` in the list of edge e, as (index, fraction
    /// along the edge), with the edge ends as sentinels.
    fn around(&self, li: usize, e: u32, x: f64) -> ((isize, f64), (isize, f64)) {
        let list = &self.cross[li][e as usize];
        let mut lo = (-1, 0.0);
        let mut hi = (list.len() as isize, 1.0);
        for (i, &s) in list.iter().enumerate() {
            let nth = list[..i].iter().filter(|&&q| q == s).count();
            let Some(f) = self.mark_frac(li, e, s, nth) else {
                continue;
            };
            if (i as f64) < x {
                lo = (i as isize, f);
            } else if (i as f64) > x {
                hi = (i as isize, f);
                break;
            }
        }
        (lo, hi)
    }

    /// `edge_frac` with imported tracks: they are where they are, everything else lies evenly
    /// between them.
    pub(crate) fn soft_frac(&self, li: usize, e: u32, seg: Option<u32>, slot: Option<usize>) -> f64 {
        let x = match (slot, seg) {
            (Some(j), _) => j as f64 - 0.5,
            (None, Some(s)) => {
                if let Some(f) = self.mark_frac(li, e, s, 0) {
                    return f;
                }
                self.cross[li][e as usize].iter().position(|&q| q == s).unwrap_or(0) as f64
            }
            _ => return 0.5,
        };
        let (lo, hi) = self.around(li, e, x);
        lo.1 + (hi.1 - lo.1) * (x - lo.0 as f64) / (hi.0 - lo.0) as f64
    }

    /// Portal interval of the routed crossing at place `idx` of edge e: between the imported
    /// tracks next to it, with their clearance if that much room is there. If not, the track
    /// goes in the middle and the relaxation makes the room.
    pub(crate) fn soft_interval(&self, li: usize, e: u32, idx: usize, items: &[Item]) -> (f64, f64) {
        let len = self.sub.m[li].edge_len(e);
        let (l, r) = self.around(li, e, idx as f64);
        let it = &items[idx];
        let (pa, pb) = (l.1 * len, r.1 * len);
        let margin = self.cfg.margin;
        let lo = if l.0 >= 0 {
            pa + Self::gap(self.board, &items[l.0 as usize], it)
        } else {
            self.end_need(li, e, 0, it) + margin
        };
        let hi = if (r.0 as usize) < items.len() {
            pb - Self::gap(self.board, it, &items[r.0 as usize])
        } else {
            len - self.end_need(li, e, 1, it) - margin
        };
        let n = (r.0 - l.0 - 1) as f64;
        let j = (idx as isize - l.0 - 1) as f64;
        if lo <= hi {
            let w = (hi - lo) / n;
            (lo + w * j, lo + w * (j + 1.0))
        } else {
            let x = pa + (pb - pa) * (j + 1.0) / (n + 1.0);
            (x, x)
        }
    }

    /// Geometry with soft copper: the routed tracks through their portals, the imported ones as
    /// they are, except where a routed track or a via needs them straight.
    pub(crate) fn soft_geometry(&self) -> Geometry {
        use rayon::prelude::*;
        let board = self.board;
        let soft = &board.soft;
        let routed: Vec<u32> = (0..self.segs.len() as u32)
            .filter(|&i| self.segs[i as usize].as_ref().is_some_and(|s| s.imported.is_none()))
            .collect();
        let mut tracks: Vec<Track> = routed.par_iter().flat_map_iter(|&sid| self.seg_tracks(sid)).collect();
        // a stretch can bend far inside a triangle, or even out of it and back over the edge it
        // came over, where the chords do not show it: what a routed track really runs into gets
        // straight, and straight chords do not cross
        let mut flats: FastMap<u32, Vec<bool>> = FastMap::default();
        let paths: Vec<(usize, Vec<Point>)> = routed
            .iter()
            .map(|&sid| {
                let s = self.segs[sid as usize].as_ref().unwrap();
                (s.li as usize, flatten(&self.seg_path(sid)))
            })
            .collect();
        for (sid, o) in self.segs.iter().enumerate() {
            let Some(o) = o else { continue };
            let Some(imp) = o.imported.as_ref() else { continue };
            let pts = &soft.chains[imp.chain as usize].pts;
            for (li, path) in &paths {
                if *li != o.li as usize {
                    continue;
                }
                for (i, a) in pts.windows(2).enumerate() {
                    if path.windows(2).any(|b| segs_intersect(a[0], a[1], b[0], b[1])) {
                        let flat = flats.entry(sid as u32).or_insert_with(|| imp.flat.clone());
                        for end in [i, i + 1] {
                            let k = imp.marks.iter().position(|m| m.i as usize >= end);
                            flat[k.unwrap_or(imp.marks.len())] = true;
                        }
                    }
                }
            }
        }
        for (sid, s) in self.segs.iter().enumerate() {
            let Some(s) = s else { continue };
            let Some(imp) = s.imported.as_ref() else { continue };
            let ch = &soft.chains[imp.chain as usize];
            let flat = flats.get(&(sid as u32)).unwrap_or(&imp.flat);
            let bent = flat.iter().any(|&f| f);
            let (pts, elems) = if bent {
                let p = imp.path(&ch.pts, flat);
                let e = p.windows(2).map(|w| Elem::Line(w[0], w[1])).collect();
                (p, e)
            } else {
                (ch.pts.clone(), ch.elems.clone())
            };
            tracks.push(Track {
                pts,
                elems,
                width: ch.width,
                layer: ch.layer,
                net: ch.net,
                necks: Vec::new(),
                neck_w: ch.width,
                origin: Some(imp.chain),
                resting: !bent,
                fitted: Vec::new(),
                tap: false,
            });
        }
        // what could not be traced is still there
        for &c in &self.soft_lost {
            let ch = &soft.chains[c as usize];
            tracks.push(Track {
                pts: ch.pts.clone(),
                elems: ch.elems.clone(),
                width: ch.width,
                layer: ch.layer,
                net: ch.net,
                necks: Vec::new(),
                neck_w: ch.width,
                origin: Some(c),
                resting: true,
                fitted: Vec::new(),
                tap: false,
            });
        }
        let base = self.sub.shared.soft_base as u32;
        let mut vias: Vec<Via> = self
            .vias
            .iter()
            .filter(|(v, _)| v.checked_sub(base).is_none_or(|n| !soft.nodes[n as usize].fixed))
            .map(|(&v, info)| {
                let node = v.checked_sub(base);
                Via {
                    at: self.site_point(v),
                    diameter: info.r * 2.0,
                    drill: info.drill,
                    net: info.net,
                    layer: node.and_then(|n| soft.nodes[n as usize].layer),
                    origin: node,
                    resting: node.is_some(),
                }
            })
            .collect();
        vias.sort_by(|a, b| a.at.x.total_cmp(&b.at.x).then(a.at.y.total_cmp(&b.at.y)));
        Geometry { tracks, vias }
    }
}

fn end_of(e: SoftEnd, vertex: Option<u32>, base: usize, p: Point) -> End {
    match (e, vertex) {
        (SoftEnd::Node(n), Some(_)) => End::Via(base as u32 + n),
        _ => End::Point(p),
    }
}

/// Triangles by position.
fn tri_index(m: &Mesh) -> GridIndex {
    let mut b = crate::geom::Rect::empty();
    for &p in &m.verts {
        b.add(p);
    }
    let mut idx = GridIndex::new(b, 1.0);
    for (i, t) in m.tris.iter().enumerate() {
        let mut r = crate::geom::Rect::empty();
        for &v in t {
            r.add(m.verts[v as usize]);
        }
        idx.insert(i as u32, &r);
    }
    idx
}

fn holds(m: &Mesh, t: u32, p: Point) -> bool {
    let v = m.tris[t as usize];
    let (a, b, c) = (m.verts[v[0] as usize], m.verts[v[1] as usize], m.verts[v[2] as usize]);
    orient(a, b, p) >= -EPS && orient(b, c, p) >= -EPS && orient(c, a, p) >= -EPS
}

/// The triangle a track leaves vertex v into, towards q.
fn fan_tri(m: &Mesh, v: u32, q: Point) -> Option<u32> {
    let pv = m.verts[v as usize];
    let list = &m.vert_tris[v as usize];
    list.iter()
        .copied()
        .find(|&t| {
            let tv = m.tris[t as usize];
            let i = tv.iter().position(|&x| x == v).unwrap();
            let (b, c) = (m.verts[tv[(i + 1) % 3] as usize], m.verts[tv[(i + 2) % 3] as usize]);
            orient(pv, b, q) >= 0.0 && orient(pv, c, q) <= 0.0
        })
        .or(list.first().copied())
}

/// Walk a polyline through the mesh, from vertex `from` if given, else from where its first
/// point lies. A stretch that comes straight back over the edge it just crossed cancels.
fn trace(m: &Mesh, pts: &[Point], from: Option<u32>, index: &GridIndex) -> Option<Traced> {
    let mut t = match from {
        Some(v) => fan_tri(m, v, pts.iter().copied().find(|p| p.dist(pts[0]) > 1e-9)?)?,
        None => index
            .query_point(pts[0], 0.0)
            .into_iter()
            .filter(|&t| holds(m, t, pts[0]))
            .min()?,
    };
    let first = t;
    let mut entry = NONE;
    let mut crossings: Vec<Crossing> = Vec::new();
    let mut marks: Vec<Mark> = Vec::new();
    for i in 0..pts.len() - 1 {
        let mut p = pts[i];
        let q = pts[i + 1];
        let mut steps = 0;
        loop {
            steps += 1;
            if steps > 20_000 {
                return None;
            }
            let tv = m.tris[t as usize];
            let mut exit: Option<(f64, usize)> = None;
            for k in 0..3 {
                let (a, b) = (m.verts[tv[k] as usize], m.verts[tv[(k + 1) % 3] as usize]);
                let oq = orient(a, b, q);
                if oq >= -EPS || m.tri_edges[t as usize][k] == entry {
                    continue;
                }
                let op = orient(a, b, p).max(0.0);
                let s = op / (op - oq);
                if exit.is_none_or(|x| s < x.0) {
                    exit = Some((s, k));
                }
            }
            let Some((s, k)) = exit else { break };
            let e = m.tri_edges[t as usize][k];
            let next = m.tri_nbrs[t as usize][k];
            if next == NONE {
                return None;
            }
            let x = p.lerp(q, s.clamp(0.0, 1.0));
            if crossings
                .last()
                .is_some_and(|c| c.e == e && m.other_tri(e, c.from) == t)
            {
                crossings.pop();
                marks.pop();
                entry = crossings.last().map_or(NONE, |c| c.e);
            } else {
                if m.status[t as usize] == NONE || m.status[next as usize] == NONE {
                    crossings.push(Crossing { e, from: t });
                    marks.push(Mark { p: x, i: i as u32 });
                }
                entry = e;
            }
            p = x;
            t = next;
        }
    }
    Some(Traced {
        crossings,
        marks,
        last: t,
        first,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::*;
    use crate::geom::{Polygon, Shape, pt};
    use crate::route::RouterConfig;
    use crate::topo::{Substrate, SubstrateConfig};
    use crate::{SearchConfig, route_board};

    /// Net A runs from (2, 5) to (8, 5) as an unlocked track through a channel that ends at
    /// y = 4.4 and 5.6 at x = 5; net B has to pass the same channel from below.
    fn channel_board() -> Board {
        let mut b = crate::topo::tests::two_pad_board();
        b.layers.truncate(1);
        b.nets.push(Net {
            name: "B".into(),
            class: 0,
            pins: vec![2, 3],
            planes: LayerSet::default(),
            priority: 0,
        });
        for (i, x) in [3.5, 6.5].into_iter().enumerate() {
            let at = b.obstacles.len();
            b.obstacles.push(Obstacle {
                shape: Shape::Polygon(crate::geom::roundrect_polygon(pt(x, 3.0), 0.6, 0.6, 0.0, 0.0, 0.005)),
                layers: LayerSet::single(0),
                net: Some(1),
                kind: ObstacleKind::Pad,
                hole: None,
                clearance: 0.2,
                pin: Some(2 + i),
                locked: true,
            });
            b.pins.push(Pin {
                name: format!("Q{i}"),
                footprint: format!("Q{i}"),
                net: 1,
                pos: pt(x, 3.0),
                layers: LayerSet::single(0),
                obstacles: vec![at],
                size: (0.6, 0.6),
            });
        }
        for (y0, y1) in [(-1.0, 4.4), (5.6, 11.0)] {
            b.obstacles.push(Obstacle {
                shape: Shape::Polygon(Polygon::new(vec![pt(4.6, y0), pt(5.4, y0), pt(5.4, y1), pt(4.6, y1)])),
                layers: LayerSet::single(0),
                net: None,
                kind: ObstacleKind::Graphic,
                hole: None,
                clearance: 0.2,
                pin: None,
                locked: true,
            });
        }
        let first = b.obstacles.len() as u32;
        let (a, z) = (pt(2.0, 5.0), pt(8.0, 5.0));
        b.obstacles.push(Obstacle {
            shape: Shape::Capsule { a, b: z, r: 0.1 },
            layers: LayerSet::single(0),
            net: Some(0),
            kind: ObstacleKind::Track,
            hole: None,
            clearance: 0.2,
            pin: None,
            locked: false,
        });
        b.copper.push(CopperItem {
            kind: CopperKind::Line {
                a,
                b: z,
                width: 0.2,
                layer: 0,
            },
            net: Some(0),
            locked: false,
            source: 7,
            first,
            last: first + 1,
        });
        b
    }

    #[test]
    fn a_track_becomes_a_chain_between_its_pads() {
        let mut b = channel_board();
        b.soften(&["B".to_string()]);
        assert_eq!(b.soft.chains.len(), 1);
        let c = &b.soft.chains[0];
        assert_eq!((c.start, c.end), (SoftEnd::Fixed, SoftEnd::Fixed));
        assert_eq!(c.items, vec![7]);
        assert!(b.obstacles.last().unwrap().layers.is_empty());
    }

    #[test]
    fn a_resting_track_keeps_its_geometry_and_crosses_in_order() {
        let mut b = channel_board();
        b.soften(&["B".to_string()]);
        let sub = Substrate::build(&b, &SubstrateConfig::default());
        let r = Router::new(&b, &sub, RouterConfig::default());
        assert!(r.soft_lost.is_empty());
        let s = r.segs[0].as_ref().unwrap();
        assert!(!s.crossings.is_empty());
        assert!(r.validate().iter().all(|l| !l.contains("does not cross")));
        let g = r.geometry();
        assert!(g.tracks[0].resting);
        assert_eq!(g.tracks[0].pts, vec![pt(2.0, 5.0), pt(8.0, 5.0)]);
    }

    #[test]
    fn a_new_connection_threads_in_and_pushes_the_track_aside() {
        let mut b = channel_board();
        b.nets[0].pins.clear();
        b.soften(&["B".to_string()]);
        let search = SearchConfig {
            population: 2,
            generations: 0,
            geometry_generations: 0,
            frames: false,
            progress: None,
            seed: None,
            open_end: None,
            air: 0.0,
            prefer_width: 0.0,
        };
        let (_, res) = route_board(&b, &SubstrateConfig::default(), &RouterConfig::default(), &search);
        assert_eq!(res.stats.routed, 1, "{:?}", res.stats.unrouted);
        assert!(crate::check::check(&b, &res.geometry).is_empty());
        let c = soft_changes(&b, &res.geometry);
        assert_eq!(c.strip, vec![7]);
        assert_eq!(c.chains, vec![true]);
        assert!(c.shift >= 0.09 && c.shift < 0.6, "shift {}", c.shift);
    }

    fn quick() -> SearchConfig {
        SearchConfig {
            population: 2,
            generations: 0,
            geometry_generations: 0,
            frames: false,
            progress: None,
            seed: None,
            open_end: None,
            air: 0.0,
            prefer_width: 0.0,
        }
    }

    fn route(b: &Board) -> (Substrate, crate::RouteResult, ()) {
        let (sub, res) = route_board(b, &SubstrateConfig::default(), &RouterConfig::default(), &quick());
        (sub, res, ())
    }

    #[test]
    fn finishing_a_net_shoves_where_plain_finds_no_way() {
        let b = channel_board();
        let nets = ["B".to_string()];
        let plain = crate::finish_nets(&b, &nets, false, 0.0, route);
        assert_eq!(plain.res.stats.unrouted.len(), 1);
        assert!(plain.res.stats.unrouted[0].starts_with("B from "));
        let f = crate::finish_nets(&b, &nets, true, crate::DEFAULT_GIVE, route);
        assert!(f.res.stats.unrouted.is_empty(), "{:?}", f.res.stats.unrouted);
        assert_eq!((f.tries.len(), f.kept), (1, 0), "nothing has to give way");
        assert!(f.given.is_empty());
    }

    #[test]
    fn a_net_near_the_pins_gives_way_and_is_routed_again() {
        let b = channel_board();
        let f = crate::finish_nets(&b, &["B".to_string()], false, crate::DEFAULT_GIVE, route);
        assert_eq!(f.tries.len(), 2);
        assert_eq!((f.tries[0].open, f.tries[1].radius), (1, 2.0));
        assert_eq!(f.kept, 1);
        assert_eq!(f.given.iter().collect::<Vec<_>>(), ["A"]);
        assert!(f.res.stats.unrouted.is_empty(), "{:?}", f.res.stats.unrouted);
        assert_eq!(f.res.stats.conns, 2);
    }

    #[test]
    fn a_try_that_leaves_more_open_loses_against_the_one_before() {
        let b = channel_board();
        // A gives way and finds no way back: that try has more open than the first
        let worse = |b: &Board| {
            let sub = Substrate::build(b, &SubstrateConfig::default());
            let mut unrouted = vec!["B from Q0".to_string()];
            if !b.nets[0].pins.is_empty() {
                unrouted.insert(0, "A from P0".to_string());
            }
            let stats = crate::RunStats {
                unrouted,
                ..Default::default()
            };
            let res = crate::RouteResult {
                geometry: Default::default(),
                stats,
                portfolio: Vec::new(),
                frames: Vec::new(),
                offsets: Default::default(),
                alternatives: Vec::new(),
            };
            (sub, res, ())
        };
        let f = crate::finish_nets(&b, &["B".to_string()], false, crate::DEFAULT_GIVE, worse);
        assert_eq!(
            f.tries.iter().map(|t| (t.given, t.open)).collect::<Vec<_>>(),
            [(0, 1), (1, 2)]
        );
        assert_eq!(f.kept, 0);
        assert!(f.given.is_empty(), "the copper of A stays");
    }
}
