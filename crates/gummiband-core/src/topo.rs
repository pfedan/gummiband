//! Routing substrate: per routing layer one constrained Delaunay triangulation of the obstacles
//! of that layer. There are no free Steiner points: every edge runs between real obstacles of
//! its layer, so the capacity of an edge is the room between the obstacles at its ends. Vias
//! are inserted as new vertices into every layer mesh when they are placed (the free triangle
//! that holds the via site is split in three).
//!
//! Every triangle is free or covered by an obstacle; edge ends carry the clearance demands of
//! the obstacles that meet there.

use std::sync::Arc;

use rayon::prelude::*;
use spade::handles::FixedVertexHandle;
use spade::{ConstrainedDelaunayTriangulation, Point2, PositionInTriangulation, Triangulation};

use crate::board::{Board, LayerId, NetId, ObstacleKind};
use crate::geom::{Point, Polygon, Rect, orient, pt};
use crate::index::{FastMap, GridIndex};

pub const NONE: u32 = u32::MAX;
/// Obstacle id used for "outside the board".
pub const OUTSIDE: u32 = u32::MAX - 1;
/// `vert_site` marker of a Steiner vertex (inserted to avoid sliver triangles).
pub const STEINER: u32 = u32::MAX - 2;
/// `site_tri` marker of a via-in-pad site on the layer of its pad.
pub const PAD_SITE: u32 = u32::MAX - 3;

#[derive(Debug, Clone)]
pub struct SubstrateConfig {
    /// Pitch of the via site grid in mm.
    pub via_pitch: f64,
    /// Pitch of the Steiner grid in free space (0 = none). Steiner vertices break up long sliver
    /// triangles in empty regions; tracks keep clearance from them like from a tiny obstacle.
    pub steiner_pitch: f64,
}

impl Default for SubstrateConfig {
    fn default() -> Self {
        SubstrateConfig {
            via_pitch: 0.4,
            steiner_pitch: 2.5,
        }
    }
}

/// Clearance demand at one end of an edge.
#[derive(Debug, Clone, Copy)]
pub struct Demand {
    /// Obstacle index or [`OUTSIDE`].
    pub obs: u32,
    /// 1 for the obstacle vertex itself, 1/sin(angle) for an obstacle side leaving the vertex.
    pub factor: f32,
}

/// Nearby obstacle of a via site.
#[derive(Debug, Clone, Copy)]
pub struct SiteNeighbor {
    pub obs: u32,
    /// Distance from the site to the obstacle copper (or hole edge).
    pub dist: f32,
}

/// Candidate via position.
#[derive(Debug, Clone)]
pub struct Site {
    pub p: Point,
    pub nbrs: Vec<SiteNeighbor>,
    /// Via in pad: the pin whose pad holds the site.
    pub pad: Option<u32>,
    /// Distance to the board edge (0 outside), as `Board::edge_dist` gives it; NaN where it is not
    /// known yet (the nodes of soft copper). A board with many cutouts makes that distance dear,
    /// and the router asks for it at every via site of every search.
    pub edge: f64,
}

/// Immutable data shared by all router runs.
pub struct Shared {
    pub obs_polys: Vec<Polygon>,
    pub obs_index: GridIndex,
    pub sites: Vec<Site>,
    /// Via-in-pad sites per pin.
    pub pad_sites: FastMap<u32, Vec<u32>>,
    /// The sites from here on are the nodes of the board's soft copper.
    pub soft_base: usize,
    pub bounds: Rect,
    /// What the relaxation needs of the board alone, by widest track, margin, reach and layer
    /// count (see `relax::BoardStatic`); filled on first use.
    pub(crate) relax_cache: RelaxCache,
    /// The reference planes of the routing layers, rastered on first use (see
    /// [`crate::refplane`]); None inside when no routing layer has a plane next to it.
    pub(crate) ref_map: std::sync::OnceLock<Option<Arc<crate::refplane::RefMap>>>,
}

pub(crate) type RelaxCache = std::sync::Mutex<Vec<((u64, u64, u64, usize), Arc<crate::relax::BoardStatic>)>>;

/// Triangulation of one routing layer.
#[derive(Clone)]
pub struct Mesh {
    pub layer: LayerId,
    pub verts: Vec<Point>,
    /// Site id of a vertex that was inserted for a via (NONE for obstacle vertices).
    pub vert_site: Vec<u32>,
    /// Triangles, vertices counter-clockwise (y up).
    pub tris: Vec<[u32; 3]>,
    /// Edge k of a triangle goes from vertex k to vertex k+1.
    pub tri_edges: Vec<[u32; 3]>,
    /// Neighbour across edge k, or NONE.
    pub tri_nbrs: Vec<[u32; 3]>,
    /// Edge endpoints (u < v by index) and the triangles on both sides.
    pub edges: Vec<[u32; 2]>,
    pub edge_tris: Vec<[u32; 2]>,
    pub vert_tris: Vec<Vec<u32>>,
    /// Per triangle: NONE (free), OUTSIDE or the covering obstacle.
    pub status: Vec<u32>,
    /// Free triangles in a solder mask opening around copper of one net: that net routes through
    /// them as through free space (no copper of its own lies there), for the others they are
    /// closed.
    pub opening: FastMap<u32, NetId>,
    /// Triangles covered by each obstacle (fixed: vias only split free triangles).
    pub obs_tris: Arc<FastMap<u32, Vec<u32>>>,
    /// The outline of the pads with exits (`exit.rs`): for each edge between such a pad and free
    /// room the exit whose far end it lies on (an index into `Board::exits`), or NONE for an
    /// edge that is shut. A track of the net crosses the outline at an exit only.
    pub exit_edges: Arc<FastMap<u32, u32>>,
    /// Obstacles around each original vertex (via vertices have none).
    pub vert_obs: Arc<Vec<Vec<u32>>>,
    /// Per edge end (2 * edge + end): demands.
    pub demands: Vec<Vec<Demand>>,
    /// Unused via sites inside each triangle.
    pub tri_sites: Vec<Vec<u32>>,
    /// Triangle that holds each unused site (NONE once placed or if not usable).
    pub site_tri: Vec<u32>,
    /// Vertex of each placed site.
    pub site_vert: FastMap<u32, u32>,
    /// Where a site's vertex lies if not on the site: a soft node right on a mesh edge.
    pub site_p: FastMap<u32, Point>,
}

/// All layer meshes. Cheap to clone (static parts shared); every router run owns one, because
/// placing vias splits triangles.
#[derive(Clone)]
pub struct Substrate {
    pub shared: Arc<Shared>,
    pub layers: Vec<LayerId>,
    pub m: Vec<Mesh>,
}

fn snap(v: f64) -> f64 {
    (v * 1e5).round() / 1e5
}

impl Substrate {
    pub fn build(board: &Board, cfg: &SubstrateConfig) -> Substrate {
        let me = board.rules.max_error;
        let layers: Vec<LayerId> = board.routing_layers().iter().collect();
        let obs_polys: Vec<Polygon> = board.obstacles.par_iter().map(|o| o.shape.to_polygon(me)).collect();
        let mut bounds = board.bbox();
        if bounds.width() <= 0.0 {
            for p in &obs_polys {
                let b = p.bbox();
                bounds.add(b.min);
                bounds.add(b.max);
            }
        }
        let mut obs_index = GridIndex::new(bounds.grow(5.0), 1.0);
        for (i, p) in obs_polys.iter().enumerate() {
            obs_index.insert(i as u32, &p.bbox());
        }
        let mut shared = Shared {
            obs_polys,
            obs_index,
            sites: Vec::new(),
            pad_sites: FastMap::default(),
            soft_base: 0,
            bounds,
            relax_cache: Default::default(),
            ref_map: Default::default(),
        };
        shared.sites = build_sites(board, &shared, cfg.via_pitch);
        shared.sites.extend(build_pad_sites(board, &shared));
        for (i, s) in shared.sites.iter().enumerate() {
            if let Some(pin) = s.pad {
                shared.pad_sites.entry(pin).or_default().push(i as u32);
            }
        }
        shared.soft_base = shared.sites.len();
        shared.sites.extend(board.soft.nodes.iter().map(|n| Site {
            p: n.at,
            nbrs: Vec::new(),
            pad: None,
            edge: f64::NAN,
        }));
        let nsites = shared.sites.len();
        let mut meshes: Vec<Mesh> = layers
            .par_iter()
            .map(|&l| Mesh::build(board, &shared, l, cfg.steiner_pitch))
            .collect();
        // a site is usable only if it lies in a free triangle on every layer
        let usable: Vec<bool> = (0..nsites)
            .map(|s| meshes.iter().all(|m| m.site_tri[s] != NONE))
            .collect();
        for m in &mut meshes {
            for (s, &ok) in usable.iter().enumerate() {
                if !ok && m.site_tri[s] != NONE {
                    let t = m.site_tri[s];
                    if t != PAD_SITE {
                        m.tri_sites[t as usize].retain(|&q| q != s as u32);
                    }
                    m.site_tri[s] = NONE;
                }
            }
        }
        Substrate {
            shared: Arc::new(shared),
            layers,
            m: meshes,
        }
    }

    /// True if routing triangle status `s` lets net `net` in.
    pub fn passable(board: &Board, s: u32, net: NetId) -> bool {
        s == NONE
            || (s != OUTSIDE
                && board.obstacles[s as usize].net == Some(net)
                && board.obstacles[s as usize].kind != ObstacleKind::Hole)
    }

    pub fn site_usable(&self, site: u32) -> bool {
        self.m.iter().all(|m| m.site_tri[site as usize] != NONE)
    }
}

impl Mesh {
    fn build(board: &Board, shared: &Shared, l: LayerId, steiner_pitch: f64) -> Mesh {
        let bounds = shared.bounds;
        let mut cdt: ConstrainedDelaunayTriangulation<Point2<f64>> = ConstrainedDelaunayTriangulation::new();
        let mut handle_of: FastMap<(i64, i64), FixedVertexHandle> = FastMap::default();
        let mut constraints: Vec<(FixedVertexHandle, FixedVertexHandle)> = Vec::new();
        // long obstacle sides get extra vertices: a channel between a pin and the long side of
        // an exposed pad (or the board edge) is then measured across, by a short mesh edge,
        // and not by the long diagonals to the far corners
        let refine = crate::knobs::REFINE.num::<f64>();
        let on_layer = |o: &crate::board::Obstacle| {
            o.layers.has(l) && !matches!(o.kind, ObstacleKind::Keepout { tracks: false, .. })
        };
        // The width of a channel between a corner and a side is the distance from the corner to
        // its foot on the side. That foot becomes a vertex, so that a mesh edge measures the
        // channel straight across; the edges to the vertices the side happens to have are longer
        // than the channel is wide (by 75 um at 0.6 mm width and 0.3 mm to the side).
        let reach = crate::knobs::FEET.num::<f64>();
        let mut feet: FastMap<(usize, usize), Vec<f64>> = FastMap::default();
        if reach > 0.0 {
            let n_out = board.outline.len();
            let poly_of = |id: usize| {
                if id < n_out {
                    &board.outline[id]
                } else {
                    &shared.obs_polys[id - n_out]
                }
            };
            let mut drop = |from: usize, v: Point, onto: usize| {
                let poly = poly_of(onto);
                for (si, (a, b)) in poly.edges().enumerate() {
                    let ab = b - a;
                    let len = ab.len();
                    if len < 0.02 {
                        continue;
                    }
                    let t = (v - a).dot(ab) / (len * len);
                    if t * len < 0.01 || (1.0 - t) * len < 0.01 {
                        continue;
                    }
                    let f = a + ab * t;
                    let d = v.dist(f);
                    if d > reach || d < 1e-6 {
                        continue;
                    }
                    // not through the copper the corner belongs to
                    if from >= n_out && poly_of(from).contains(v + (f - v) * (1e-4 / d)) {
                        continue;
                    }
                    feet.entry((onto, si)).or_default().push(t);
                }
            };
            for id in 0..n_out + board.obstacles.len() {
                if id >= n_out && !on_layer(&board.obstacles[id - n_out]) {
                    continue;
                }
                for &v in &poly_of(id).pts {
                    for k in (0..n_out).filter(|&k| k != id) {
                        drop(id, v, k);
                    }
                    for oi in shared.obs_index.query_point(v, reach) {
                        let onto = n_out + oi as usize;
                        if onto != id && on_layer(&board.obstacles[oi as usize]) {
                            drop(id, v, onto);
                        }
                    }
                }
            }
        }
        let mut add_poly = |cdt: &mut ConstrainedDelaunayTriangulation<Point2<f64>>,
                            poly: &Polygon,
                            split: bool,
                            id: Option<usize>| {
            let mut pts = Vec::with_capacity(poly.pts.len());
            for i in 0..poly.pts.len() {
                let (a, b) = (poly.pts[i], poly.pts[(i + 1) % poly.pts.len()]);
                pts.push(a);
                let len = a.dist(b);
                let n = if split && refine > 0.0 {
                    (len / refine).floor() as usize
                } else {
                    0
                };
                // the feet first, then the even split where no foot lies
                let mut ts: Vec<f64> = id.and_then(|id| feet.get(&(id, i))).cloned().unwrap_or_default();
                ts.sort_by(f64::total_cmp);
                ts.dedup_by(|x, y| (*x - *y).abs() * len < 0.01);
                let n_feet = ts.len();
                for k in 1..=n {
                    let t = k as f64 / (n + 1) as f64;
                    if ts[..n_feet].iter().all(|f| (f - t).abs() * len >= 0.01) {
                        ts.push(t);
                    }
                }
                ts.sort_by(f64::total_cmp);
                for t in ts {
                    pts.push(a + (b - a) * t);
                }
            }
            let mut hs = Vec::new();
            for &p in &pts {
                let key = ((p.x * 1e5).round() as i64, (p.y * 1e5).round() as i64);
                let h = match handle_of.get(&key) {
                    Some(&h) => h,
                    None => {
                        let Ok(h) = cdt.insert(Point2::new(snap(p.x), snap(p.y))) else {
                            continue;
                        };
                        handle_of.insert(key, h);
                        h
                    }
                };
                hs.push(h);
            }
            for i in 0..hs.len() {
                let (a, b) = (hs[i], hs[(i + 1) % hs.len()]);
                if a != b {
                    constraints.push((a, b));
                }
            }
        };
        // A frame around the board so that outside triangles exist and get classified.
        let frame = bounds.grow(2.0);
        add_poly(
            &mut cdt,
            &Polygon::new(vec![
                frame.min,
                pt(frame.max.x, frame.min.y),
                frame.max,
                pt(frame.min.x, frame.max.y),
            ]),
            false,
            None,
        );
        for (k, o) in board.outline.iter().enumerate() {
            add_poly(&mut cdt, o, true, Some(k));
        }
        for (i, o) in board.obstacles.iter().enumerate() {
            if !on_layer(o) {
                continue;
            }
            let p = &shared.obs_polys[i];
            if p.bbox().overlaps(&frame) {
                add_poly(&mut cdt, p, true, Some(board.outline.len() + i));
            }
        }
        // Steiner points in free space, away from all obstacles of this layer
        let mut steiner = Vec::new();
        if steiner_pitch > 0.0 {
            let mut row = 0;
            let mut y = bounds.min.y + steiner_pitch / 2.0;
            while y < bounds.max.y {
                let off = if row % 2 == 1 { steiner_pitch / 2.0 } else { 0.0 };
                let mut x = bounds.min.x + steiner_pitch / 2.0 + off;
                while x < bounds.max.x {
                    let p = pt(x, y);
                    let clear = board.edge_dist(p) >= steiner_pitch * 0.5
                        && shared.obs_index.query_point(p, steiner_pitch * 0.5).iter().all(|&id| {
                            !board.obstacles[id as usize].layers.has(l)
                                || shared.obs_polys[id as usize].dist_to_point(p) >= steiner_pitch * 0.5
                        });
                    if clear {
                        steiner.push(p);
                    }
                    x += steiner_pitch;
                }
                y += steiner_pitch * 0.866;
                row += 1;
            }
        }
        quiet_spade_panics();
        let mut skipped_constraints = 0usize;
        let mut steiner_handles = Vec::new();
        for p in steiner {
            if let Ok(h) = cdt.insert(Point2::new(snap(p.x), snap(p.y))) {
                steiner_handles.push(h.index());
            }
        }
        // spade can trip over nearly degenerate crossings of constraint edges (an assertion
        // after it already split the edge); the triangulation stays valid, only that constraint
        // piece is missing, so such a constraint is skipped
        for (a, b) in constraints {
            if a != b {
                let ok = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    cdt.add_constraint_and_split(a, b, |v| v);
                }));
                if ok.is_err() {
                    skipped_constraints += 1;
                }
            }
        }
        if skipped_constraints > 0 {
            eprintln!("layer {l}: {skipped_constraints} degenerate constraint edges skipped");
        }
        let verts: Vec<Point> = cdt.vertices().map(|v| pt(v.position().x, v.position().y)).collect();
        let mut tris = Vec::new();
        let mut face_to_tri: FastMap<usize, u32> = FastMap::default();
        for f in cdt.inner_faces() {
            let vs = f.vertices();
            face_to_tri.insert(f.fix().index(), tris.len() as u32);
            tris.push([
                vs[0].fix().index() as u32,
                vs[1].fix().index() as u32,
                vs[2].fix().index() as u32,
            ]);
        }
        let mut m = Mesh {
            layer: l,
            vert_site: vec![NONE; verts.len()],
            verts,
            tris,
            tri_edges: Vec::new(),
            tri_nbrs: Vec::new(),
            edges: Vec::new(),
            edge_tris: Vec::new(),
            vert_tris: Vec::new(),
            status: Vec::new(),
            opening: FastMap::default(),
            obs_tris: Arc::new(FastMap::default()),
            exit_edges: Default::default(),
            vert_obs: Default::default(),
            demands: Vec::new(),
            tri_sites: Vec::new(),
            site_tri: vec![NONE; shared.sites.len()],
            site_vert: FastMap::default(),
            site_p: FastMap::default(),
        };
        for h in steiner_handles {
            m.vert_site[h] = STEINER;
        }
        m.build_topology();
        m.status = m
            .tris
            .iter()
            .map(|t| {
                let c = (m.verts[t[0] as usize] + m.verts[t[1] as usize] + m.verts[t[2] as usize]) * (1.0 / 3.0);
                classify(board, shared, l, c)
            })
            .collect();
        for (i, o) in board.obstacles.iter().enumerate() {
            if let (ObstacleKind::Keepout { .. }, Some(net), true) = (o.kind, o.net, o.layers.has(l)) {
                for t in 0..m.tris.len() {
                    if m.status[t] == NONE && shared.obs_polys[i].contains(m.tri_center(t as u32)) {
                        m.opening.insert(t as u32, net);
                    }
                }
            }
        }
        let mut ot: FastMap<u32, Vec<u32>> = FastMap::default();
        for (t, &o) in m.status.iter().enumerate() {
            if o != NONE && o != OUTSIDE {
                ot.entry(o).or_default().push(t as u32);
            }
        }
        m.obs_tris = Arc::new(ot);
        m.vert_obs = Arc::new(
            m.vert_tris
                .iter()
                .map(|list| {
                    let mut obs: Vec<u32> = Vec::new();
                    for &t in list {
                        let o = m.status[t as usize];
                        if o != NONE && !obs.contains(&o) {
                            obs.push(o);
                        }
                    }
                    obs
                })
                .collect(),
        );
        m.exit_edges = Arc::new(m.outline_of_exit_pads(board));
        m.demands = (0..m.edges.len() as u32)
            .flat_map(|e| {
                let [a, b] = m.edge_demands(e);
                [a, b]
            })
            .collect();
        m.tri_sites = vec![Vec::new(); m.tris.len()];
        for (i, site) in shared.sites.iter().enumerate().take(shared.soft_base) {
            let face = match cdt.locate(Point2::new(site.p.x, site.p.y)) {
                PositionInTriangulation::OnFace(f) => f.index(),
                _ => continue,
            };
            let Some(&t) = face_to_tri.get(&face) else { continue };
            let st = m.status[t as usize];
            if st == NONE {
                m.tri_sites[t as usize].push(i as u32);
                m.site_tri[i] = t;
            } else if let Some(pin) = site.pad
                && st != OUTSIDE
                && board.obstacles[st as usize].pin == Some(pin as usize)
            {
                m.site_tri[i] = PAD_SITE;
            }
        }
        // soft nodes: a vertex where the node has free room on this layer, nothing on other
        // copper of the net or on other layers than a branch point's own
        for (k, n) in board.soft.nodes.iter().enumerate() {
            let i = shared.soft_base + k;
            m.site_tri[i] = PAD_SITE;
            if n.layer.is_some_and(|x| x != l) {
                continue;
            }
            for d in [0.0, 2e-5, 4e-5, 8e-5] {
                let p = pt(n.at.x + d, n.at.y + d * 0.7);
                let PositionInTriangulation::OnFace(f) = cdt.locate(Point2::new(p.x, p.y)) else {
                    continue;
                };
                if let Some(&t) = face_to_tri.get(&f.index())
                    && m.status[t as usize] == NONE
                {
                    // (in the list so that a split passes it on; all are placed before any search)
                    m.site_tri[i] = t;
                    m.tri_sites[t as usize].push(i as u32);
                    if d > 0.0 {
                        m.site_p.insert(i as u32, p);
                    }
                }
                break;
            }
        }
        m
    }

    fn build_topology(&mut self) {
        let mut edge_map: FastMap<(u32, u32), u32> =
            FastMap::with_capacity_and_hasher(self.tris.len() * 2, Default::default());
        self.edges.clear();
        self.edge_tris.clear();
        self.tri_edges = vec![[0u32; 3]; self.tris.len()];
        for (ti, t) in self.tris.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                let key = (a.min(b), a.max(b));
                let e = *edge_map.entry(key).or_insert_with(|| {
                    self.edges.push([key.0, key.1]);
                    self.edge_tris.push([NONE, NONE]);
                    (self.edges.len() - 1) as u32
                });
                let slot = if self.edge_tris[e as usize][0] == NONE { 0 } else { 1 };
                self.edge_tris[e as usize][slot] = ti as u32;
                self.tri_edges[ti][k] = e;
            }
        }
        self.tri_nbrs = (0..self.tris.len())
            .map(|ti| self.tri_edges[ti].map(|e| self.other_tri(e, ti as u32)))
            .collect();
        self.vert_tris = vec![Vec::new(); self.verts.len()];
        for (ti, t) in self.tris.iter().enumerate() {
            for &v in t {
                self.vert_tris[v as usize].push(ti as u32);
            }
        }
    }

    /// See `exit_edges`. An edge of the outline stays open, as on a pad without exits, where the
    /// free triangle behind it touches other copper of the net: two pads of one net side by
    /// side join directly.
    fn outline_of_exit_pads(&self, board: &Board) -> FastMap<u32, u32> {
        let mut out = FastMap::default();
        if board.exits.is_empty() {
            return out;
        }
        for (e, &[a, b]) in self.edge_tris.iter().enumerate() {
            if a == NONE || b == NONE {
                continue;
            }
            let (o, free) = match (self.status[a as usize], self.status[b as usize]) {
                (NONE, o) if o != NONE && o != OUTSIDE => (o, a),
                (o, NONE) if o != NONE && o != OUTSIDE => (o, b),
                _ => continue,
            };
            let ob = &board.obstacles[o as usize];
            let Some(pin) = ob.pin else { continue };
            let first = board.exits.partition_point(|x| x.pin < pin);
            let exits = board.exits_of(pin);
            if exits.first().is_none_or(|x| x.obs != o as usize) {
                continue;
            }
            let [u, v] = self.edges[e];
            let (pu, pv) = (self.verts[u as usize], self.verts[v as usize]);
            let mouth = exits
                .iter()
                .position(|x| crate::exit::on_mouth(x, pu) && crate::exit::on_mouth(x, pv));
            if let Some(k) = mouth {
                out.insert(e as u32, (first + k) as u32);
                continue;
            }
            let w = self.tris[free as usize].iter().copied().find(|&x| x != u && x != v);
            let joins = w.is_some_and(|w| {
                self.vert_obs[w as usize].iter().any(|&q| {
                    q != o && q != OUTSIDE && {
                        let other = &board.obstacles[q as usize];
                        other.net == ob.net && !matches!(other.kind, ObstacleKind::Keepout { .. } | ObstacleKind::Hole)
                    }
                })
            });
            if !joins {
                out.insert(e as u32, NONE);
            }
        }
        out
    }

    /// True if triangle t lies in a solder mask opening around copper of another net.
    pub fn closed(&self, t: u32, net: NetId) -> bool {
        self.opening.get(&t).is_some_and(|&n| n != net)
    }

    pub fn other_tri(&self, e: u32, t: u32) -> u32 {
        let [a, b] = self.edge_tris[e as usize];
        if a == t { b } else { a }
    }

    /// Demands at both ends of edge e.
    fn edge_demands(&self, e: u32) -> [Vec<Demand>; 2] {
        let st = &self.status;
        let [a, b] = self.edges[e as usize];
        let mut res: [Vec<Demand>; 2] = [Vec::new(), Vec::new()];
        for (end, (u, v)) in [(a, b), (b, a)].into_iter().enumerate() {
            let out = &mut res[end];
            for &t in &self.vert_tris[u as usize] {
                let o = st[t as usize];
                if o != NONE && !out.iter().any(|d| d.obs == o) {
                    out.push(Demand { obs: o, factor: 1.0 });
                }
            }
            let pu = self.verts[u as usize];
            let dir = (self.verts[v as usize] - pu).norm();
            for &t in &self.edge_tris[e as usize] {
                if t == NONE {
                    continue;
                }
                // the other edge of t at u, and the triangle beyond it
                let tv = self.tris[t as usize];
                let w = *tv.iter().find(|&&x| x != u && x != v).unwrap();
                let k = (0..3)
                    .find(|&k| {
                        let (x, y) = (tv[k], tv[(k + 1) % 3]);
                        (x == u && y == w) || (x == w && y == u)
                    })
                    .unwrap();
                let beyond = self.tri_nbrs[t as usize][k];
                let ob = if beyond == NONE { OUTSIDE } else { st[beyond as usize] };
                if ob == NONE {
                    continue;
                }
                let side = (self.verts[w as usize] - pu).norm();
                let sin = dir.cross(side).abs();
                if dir.dot(side) <= 0.0 {
                    continue; // obtuse: the vertex demand covers it
                }
                let factor = (1.0 / sin.max(0.25)) as f32;
                if let Some(d) = out.iter_mut().find(|d| d.obs == ob) {
                    d.factor = d.factor.max(factor);
                } else {
                    out.push(Demand { obs: ob, factor });
                }
            }
        }
        res
    }

    /// Demands at the `end` (0 = edges[e][0], 1 = edges[e][1]) of edge e.
    pub fn demands(&self, e: u32, end: usize) -> &[Demand] {
        &self.demands[e as usize * 2 + end]
    }

    pub fn tri_center(&self, t: u32) -> Point {
        let v = self.tris[t as usize];
        (self.verts[v[0] as usize] + self.verts[v[1] as usize] + self.verts[v[2] as usize]) * (1.0 / 3.0)
    }

    pub fn edge_len(&self, e: u32) -> f64 {
        let [a, b] = self.edges[e as usize];
        self.verts[a as usize].dist(self.verts[b as usize])
    }

    /// The middle of triangle t.
    pub fn tri_mid(&self, t: u32) -> Point {
        let [a, b, c] = self.tris[t as usize].map(|v| self.verts[v as usize]);
        (a + b + c) * (1.0 / 3.0)
    }

    /// Local index (0..3) of edge e in triangle t.
    #[inline]
    pub fn local_edge(&self, t: u32, e: u32) -> usize {
        let te = &self.tri_edges[t as usize];
        if te[0] == e {
            0
        } else if te[1] == e {
            1
        } else if te[2] == e {
            2
        } else {
            panic!("edge not in triangle")
        }
    }

    pub fn check_orientation(&self) -> usize {
        self.tris
            .iter()
            .filter(|t| {
                orient(
                    self.verts[t[0] as usize],
                    self.verts[t[1] as usize],
                    self.verts[t[2] as usize],
                ) <= 0.0
            })
            .count()
    }

    /// Split free triangle t at site `site` (point p). Returns the new vertex and the three
    /// triangles X_k = (v_k, v_k+1, s), where X_0 keeps the id t. Edge k of the old triangle is
    /// local edge 0 of X_k, the new edge (v_k+1, s) local edge 1 and (s, v_k) local edge 2.
    pub fn split(&mut self, t: u32, site: u32, p: Point, sites: &[Site]) -> (u32, [u32; 3]) {
        let s = self.verts.len() as u32;
        self.verts.push(p);
        self.vert_site.push(site);
        self.vert_tris.push(Vec::new());
        self.site_vert.insert(site, s);
        self.site_tri[site as usize] = NONE;
        let [v0, v1, v2] = self.tris[t as usize];
        let old_edges = self.tri_edges[t as usize];
        let old_nbrs = self.tri_nbrs[t as usize];
        let x = [t, self.tris.len() as u32, self.tris.len() as u32 + 1];
        let vs = [v0, v1, v2];
        let e_new: Vec<u32> = (0..3)
            .map(|i| {
                self.edges.push([vs[i].min(s), vs[i].max(s)]);
                self.edge_tris.push([NONE, NONE]);
                (self.edges.len() - 1) as u32
            })
            .collect();
        for k in 0..3 {
            let tri = [vs[k], vs[(k + 1) % 3], s];
            let edges = [old_edges[k], e_new[(k + 1) % 3], e_new[k]];
            let nbrs = [old_nbrs[k], x[(k + 1) % 3], x[(k + 2) % 3]];
            if k == 0 {
                self.tris[t as usize] = tri;
                self.tri_edges[t as usize] = edges;
                self.tri_nbrs[t as usize] = nbrs;
            } else {
                self.tris.push(tri);
                self.tri_edges.push(edges);
                self.tri_nbrs.push(nbrs);
                self.status.push(NONE);
                self.tri_sites.push(Vec::new());
                if let Some(&net) = self.opening.get(&t) {
                    self.opening.insert(x[k], net);
                }
            }
            let oe = old_edges[k] as usize;
            for side in 0..2 {
                if self.edge_tris[oe][side] == t {
                    self.edge_tris[oe][side] = x[k];
                }
            }
            let nb = old_nbrs[k];
            if nb != NONE {
                for j in 0..3 {
                    if self.tri_edges[nb as usize][j] == old_edges[k] {
                        self.tri_nbrs[nb as usize][j] = x[k];
                    }
                }
            }
        }
        for k in 0..3 {
            // E_k is shared by X_k (local 2) and X_k-1 (local 1)
            self.edge_tris[e_new[k] as usize] = [x[k], x[(k + 2) % 3]];
        }
        for (k, &v) in vs.iter().enumerate() {
            let list = &mut self.vert_tris[v as usize];
            list.retain(|&q| q != t);
            list.push(x[k]);
            list.push(x[(k + 2) % 3]);
        }
        self.vert_tris[s as usize] = x.to_vec();
        // redistribute the remaining sites
        let rest: Vec<u32> = std::mem::take(&mut self.tri_sites[t as usize])
            .into_iter()
            .filter(|&q| q != site)
            .collect();
        for q in rest {
            let qp = sites[q as usize].p;
            let mut home = NONE;
            for &xt in &x {
                let tv = self.tris[xt as usize];
                let (a, b, c) = (
                    self.verts[tv[0] as usize],
                    self.verts[tv[1] as usize],
                    self.verts[tv[2] as usize],
                );
                if orient(a, b, qp) > 0.0 && orient(b, c, qp) > 0.0 && orient(c, a, qp) > 0.0 {
                    home = xt;
                    break;
                }
            }
            self.site_tri[q as usize] = home;
            if home != NONE {
                self.tri_sites[home as usize].push(q);
            }
        }
        // demands of the touched edges
        self.demands.resize(self.edges.len() * 2, Vec::new());
        for &e in old_edges.iter().chain(e_new.iter()) {
            let [a, b] = self.edge_demands(e);
            self.demands[e as usize * 2] = a;
            self.demands[e as usize * 2 + 1] = b;
        }
        (s, x)
    }
}

fn classify(board: &Board, shared: &Shared, l: LayerId, c: Point) -> u32 {
    if !board.inside(c) {
        return OUTSIDE;
    }
    let mut found = NONE;
    for id in shared.obs_index.query_point(c, 0.0) {
        let o = &board.obstacles[id as usize];
        if !o.layers.has(l) {
            continue;
        }
        // (an opening with a net is no obstacle to its net, see Mesh::opening)
        if let ObstacleKind::Keepout { tracks, .. } = o.kind
            && (!tracks || o.net.is_some())
        {
            continue;
        }
        if shared.obs_polys[id as usize].contains(c) {
            // an obstacle without net wins (it blocks everyone)
            if found == NONE || o.net.is_none() {
                found = id;
            }
        }
    }
    found
}

fn build_sites(board: &Board, shared: &Shared, pitch: f64) -> Vec<Site> {
    let routing = board.routing_layers();
    let bounds = shared.bounds;
    let min_via_r = board.rules.min_via_diameter / 2.0;
    let mut candidates = Vec::new();
    let mut row = 0;
    let mut y = bounds.min.y + pitch / 2.0;
    while y < bounds.max.y {
        let off = if row % 2 == 1 { pitch / 2.0 } else { 0.0 };
        let mut x = bounds.min.x + pitch / 2.0 + off;
        while x < bounds.max.x {
            candidates.push(pt(x, y));
            x += pitch;
        }
        y += pitch * 0.866;
        row += 1;
    }
    // a pad grid (BGA) also gets via sites where a person puts them: in the middle between four
    // balls and half a pitch outside the outer ones, the dog-bone places. On the raster above a
    // via between the balls lands off the diagonal and closes the channel next to it. (The
    // raster sites stay: a field with only dog-bone sites routed worse on the DDR5 board, with
    // clearance errors between neighbouring vias.)
    for g in crate::patterns::grids(board) {
        let (u, v) = (g.dir, g.dir.perp());
        let balls: Vec<Point> = g.pins.iter().map(|&p| board.pins[p].pos).collect();
        let h = g.pitch / 2.0;
        let mut dog: Vec<Point> = Vec::new();
        for &b in &balls {
            for (su, sv) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
                let q = b + u * (su * h) + v * (sv * h);
                // (balls a little off the lattice give the same place four times)
                if !dog.iter().any(|&c| c.dist(q) < 0.1 * g.pitch) {
                    dog.push(q);
                }
            }
        }
        candidates.extend(dog);
    }
    candidates
        .par_iter()
        .filter_map(|&p| {
            let edge = board.edge_dist(p);
            if edge < min_via_r + board.rules.edge_clearance {
                return None;
            }
            let mut nbrs = Vec::new();
            for id in shared.obs_index.query_point(p, 2.5) {
                let o = &board.obstacles[id as usize];
                let d = shared.obs_polys[id as usize].dist_to_point(p);
                // (a mask opening is seen by net, like copper)
                if let ObstacleKind::Keepout {
                    vias,
                    tracks,
                    mask: false,
                } = o.kind
                {
                    if vias && d == 0.0 {
                        return None;
                    }
                    // A keepout for tracks next to the site: a track that ends in a via here
                    // reaches half its width into it (`Router::site_clear`). Inside the
                    // keepout no track of its layers gets to the site at all.
                    if tracks && d > 0.0 && d < 2.5 && o.layers.intersects(routing) {
                        nbrs.push(SiteNeighbor {
                            obs: id,
                            dist: d as f32,
                        });
                    }
                    continue;
                }
                if !o.layers.intersects(routing) && o.hole.is_none() {
                    continue;
                }
                let mut dd = d;
                if let Some(h) = o.hole {
                    // hole to hole spacing counts from the drill edge
                    let c = o.shape.bbox().center();
                    dd = dd.min((p.dist(c) - h).max(0.0));
                }
                if dd < min_via_r + board.rules.min_clearance && o.net.is_none() {
                    return None;
                }
                if dd < 2.5 {
                    nbrs.push(SiteNeighbor {
                        obs: id,
                        dist: dd as f32,
                    });
                }
            }
            let copper =
                |n: &SiteNeighbor| !matches!(board.obstacles[n.obs as usize].kind, ObstacleKind::Keepout { .. });
            if nbrs.iter().any(|n| (n.dist as f64) < min_via_r && copper(n)) {
                return None;
            }
            Some(Site {
                p,
                nbrs,
                pad: None,
                edge,
            })
        })
        .collect()
}

/// Via-in-pad candidates: the centre (and the ends of the long axis) of SMD pads of plane nets.
fn build_pad_sites(board: &Board, shared: &Shared) -> Vec<Site> {
    let routing = board.routing_layers();
    let min_via_r = board.rules.min_via_diameter / 2.0;
    let mut out = Vec::new();
    for (pi, pin) in board.pins.iter().enumerate() {
        let net = &board.nets[pin.net as usize];
        // plane nets for fan-outs; other nets only on pads big enough for a via
        let big = pin.size.0.min(pin.size.1) >= board.rules.min_via_diameter;
        if (net.planes.is_empty() && !big) || pin.layers.iter().filter(|&l| routing.has(l)).count() != 1 {
            continue;
        }
        let (w, h) = pin.size;
        let mut pts = vec![pin.pos];
        let long = w.max(h);
        if long > 1.2 {
            let d = long / 2.0 - 0.35;
            let dir = if w >= h { pt(1.0, 0.0) } else { pt(0.0, 1.0) };
            pts.push(pin.pos + dir * d);
            pts.push(pin.pos - dir * d);
        }
        for p in pts {
            let edge = board.edge_dist(p);
            if edge < min_via_r + board.rules.edge_clearance {
                continue;
            }
            let mut nbrs = Vec::new();
            let mut ok = true;
            for id in shared.obs_index.query_point(p, 2.5) {
                let o = &board.obstacles[id as usize];
                if o.pin == Some(pi) {
                    continue;
                }
                let d = shared.obs_polys[id as usize].dist_to_point(p);
                if let ObstacleKind::Keepout { vias, mask: false, .. } = o.kind {
                    if vias && d == 0.0 {
                        ok = false;
                    }
                    continue;
                }
                if !o.layers.intersects(routing) && o.hole.is_none() {
                    continue;
                }
                let mut dd = d;
                if let Some(hr) = o.hole {
                    let c = o.shape.bbox().center();
                    dd = dd.min((p.dist(c) - hr).max(0.0));
                }
                if dd < 2.5 {
                    nbrs.push(SiteNeighbor {
                        obs: id,
                        dist: dd as f32,
                    });
                }
            }
            if ok {
                out.push(Site {
                    p,
                    nbrs,
                    pad: Some(pi as u32),
                    edge,
                });
            }
        }
    }
    out
}

/// The assertions spade trips over on nearly degenerate constraints are caught above and only
/// cost that constraint; their panic message would still reach the console once per mesh. The
/// hook stays for every other panic.
fn quiet_spade_panics() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        let previous = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            let spade = info.location().is_some_and(|l| l.file().contains("spade"));
            if !spade {
                previous(info);
            }
        }));
    });
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::board::*;
    use crate::geom::*;

    pub fn two_pad_board() -> Board {
        let mut b = Board::default();
        b.layers = vec![
            Layer {
                name: "F.Cu".into(),
                is_plane: false,
            },
            Layer {
                name: "B.Cu".into(),
                is_plane: false,
            },
        ];
        b.outline = vec![Polygon::new(vec![pt(0., 0.), pt(10., 0.), pt(10., 10.), pt(0., 10.)])];
        b.classes = vec![NetClass {
            name: "Default".into(),
            track_width: 0.2,
            clearance: 0.2,
            via_diameter: 0.6,
            via_drill: 0.3,
            priority: 0,
            min_width: None,
            target_width: None,
            neck_refs: Vec::new(),
            ..Default::default()
        }];
        b.nets = vec![Net {
            name: "A".into(),
            class: 0,
            pins: vec![0, 1],
            planes: LayerSet::default(),
            priority: 0,
        }];
        for (i, x) in [2.0, 8.0].into_iter().enumerate() {
            b.obstacles.push(Obstacle {
                shape: Shape::Polygon(roundrect_polygon(pt(x, 5.), 1.0, 1.0, 0.0, 0.0, 0.005)),
                layers: LayerSet::single(0),
                net: Some(0),
                kind: ObstacleKind::Pad,
                hole: None,
                clearance: 0.2,
                pin: Some(i),
                locked: true,
            });
            b.pins.push(Pin {
                name: format!("P{i}"),
                footprint: format!("P{i}"),
                net: 0,
                pos: pt(x, 5.),
                layers: LayerSet::single(0),
                obstacles: vec![i],
                size: (1., 1.),
            });
        }
        b
    }

    #[test]
    fn substrate_classifies_pads_and_outside() {
        let b = two_pad_board();
        let s = Substrate::build(&b, &SubstrateConfig::default());
        let m = &s.m[0];
        assert_eq!(m.check_orientation(), 0);
        let owned = m.status.iter().filter(|&&x| x != NONE && x != OUTSIDE).count();
        assert!(owned >= 2, "pad triangles {owned}");
        assert!(m.status.contains(&OUTSIDE));
        let sites = (0..s.shared.sites.len() as u32).filter(|&q| s.site_usable(q)).count();
        assert!(sites > 100, "sites {sites}");
    }

    /// Via vertices split the free triangles of an opening: the pieces stay closed.
    #[test]
    fn an_opening_stays_closed_when_a_triangle_splits() {
        let mut b = two_pad_board();
        b.obstacles.push(Obstacle {
            shape: Shape::Polygon(Polygon::new(vec![pt(4., 3.), pt(6., 3.), pt(6., 7.), pt(4., 7.)])),
            layers: LayerSet::single(1),
            net: Some(0),
            kind: ObstacleKind::Keepout {
                vias: true,
                tracks: true,
                mask: true,
            },
            hole: None,
            clearance: 0.0,
            pin: None,
            locked: true,
        });
        let mut s = Substrate::build(&b, &SubstrateConfig::default());
        assert!(s.m[0].opening.is_empty() && !s.m[1].opening.is_empty());
        let inside = |p: Point| p.x > 4.0 && p.x < 6.0 && p.y > 3.0 && p.y < 7.0;
        let site = (0..s.shared.sites.len() as u32)
            .find(|&q| s.site_usable(q) && inside(s.shared.sites[q as usize].p))
            .unwrap();
        let p = s.shared.sites[site as usize].p;
        let shared = s.shared.clone();
        let m = &mut s.m[1];
        let t = m.site_tri[site as usize];
        let (_, x) = m.split(t, site, p, &shared.sites);
        assert!(x.iter().all(|&t| m.closed(t, 1) && !m.closed(t, 0)));
    }

    #[test]
    fn split_keeps_the_mesh_consistent() {
        let b = two_pad_board();
        let mut s = Substrate::build(&b, &SubstrateConfig::default());
        let site = (0..s.shared.sites.len() as u32).find(|&q| s.site_usable(q)).unwrap();
        let p = s.shared.sites[site as usize].p;
        let shared = s.shared.clone();
        let m = &mut s.m[1];
        let t = m.site_tri[site as usize];
        let (v, x) = m.split(t, site, p, &shared.sites);
        assert_eq!(m.check_orientation(), 0);
        for &xt in &x {
            assert!(m.tris[xt as usize].contains(&v));
            for k in 0..3 {
                let e = m.tri_edges[xt as usize][k];
                let [a, b2] = m.edges[e as usize];
                let tv = m.tris[xt as usize];
                let (p, q) = (tv[k], tv[(k + 1) % 3]);
                assert!((a == p && b2 == q) || (a == q && b2 == p));
                assert!(m.edge_tris[e as usize].contains(&xt));
                let nb = m.tri_nbrs[xt as usize][k];
                if nb != NONE {
                    assert!(m.tri_edges[nb as usize].contains(&e));
                }
            }
        }
    }
}
