//! Routing substrate: one constrained Delaunay triangulation of the obstacle points of all
//! routing layers. There are no free Steiner points: every edge runs between real obstacles, so
//! the capacity of an edge is the room between the obstacles at its ends. Vias are inserted as
//! new vertices when they are placed (a free triangle is split in three).
//!
//! Per routing layer every triangle is free or covered by an obstacle; edge ends carry the
//! clearance demands of the obstacles that meet there.

use std::collections::HashMap;
use std::sync::Arc;

use rayon::prelude::*;
use spade::handles::FixedVertexHandle;
use spade::{ConstrainedDelaunayTriangulation, Point2, PositionInTriangulation, Triangulation};

use crate::board::{Board, LayerId, NetId, ObstacleKind};
use crate::geom::{Point, Polygon, Rect, orient, pt};
use crate::index::GridIndex;

pub const NONE: u32 = u32::MAX;
/// Obstacle id used for "outside the board".
pub const OUTSIDE: u32 = u32::MAX - 1;

#[derive(Debug, Clone)]
pub struct SubstrateConfig {
    /// Pitch of the via site grid in mm.
    pub via_pitch: f64,
}

impl Default for SubstrateConfig {
    fn default() -> Self {
        SubstrateConfig { via_pitch: 0.4 }
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
}

/// Immutable data shared by all router runs.
pub struct Shared {
    pub layers: Vec<LayerId>,
    pub obs_polys: Vec<Polygon>,
    pub obs_index: GridIndex,
    pub sites: Vec<Site>,
    pub bounds: Rect,
}

/// The mesh. Cheap to clone (the static parts are shared); every router run owns one, because
/// placing vias splits triangles.
#[derive(Clone)]
pub struct Substrate {
    pub shared: Arc<Shared>,
    pub layers: Vec<LayerId>,
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
    /// Per routing layer, per triangle: NONE (free), OUTSIDE or the covering obstacle.
    pub tri_status: Vec<Vec<u32>>,
    /// Per routing layer: triangles covered by each obstacle (fixed: vias only split free ones).
    pub obs_tris: Arc<Vec<HashMap<u32, Vec<u32>>>>,
    /// Per routing layer, per edge end (2 * edge + end): demands.
    pub demands: Vec<Vec<Vec<Demand>>>,
    /// Unused via sites inside each triangle.
    pub tri_sites: Vec<Vec<u32>>,
}

fn snap(v: f64) -> f64 {
    (v * 1e5).round() / 1e5
}

impl Substrate {
    pub fn build(board: &Board, cfg: &SubstrateConfig) -> Substrate {
        let me = board.rules.max_error;
        let layers: Vec<LayerId> = board.routing_layers().iter().collect();
        let routing = board.routing_layers();
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

        // --- vertices and constraints
        let mut cdt: ConstrainedDelaunayTriangulation<Point2<f64>> = ConstrainedDelaunayTriangulation::new();
        let mut handle_of: HashMap<(i64, i64), FixedVertexHandle> = HashMap::new();
        let mut constraints: Vec<(FixedVertexHandle, FixedVertexHandle)> = Vec::new();
        let mut add_poly = |cdt: &mut ConstrainedDelaunayTriangulation<Point2<f64>>, poly: &Polygon| {
            let mut hs = Vec::new();
            for &p in &poly.pts {
                let key = ((p.x * 1e5).round() as i64, (p.y * 1e5).round() as i64);
                let h = match handle_of.get(&key) {
                    Some(&h) => h,
                    None => {
                        let Ok(h) = cdt.insert(Point2::new(snap(p.x), snap(p.y))) else { continue };
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
            &Polygon::new(vec![frame.min, pt(frame.max.x, frame.min.y), frame.max, pt(frame.min.x, frame.max.y)]),
        );
        for o in &board.outline {
            add_poly(&mut cdt, o);
        }
        for (i, o) in board.obstacles.iter().enumerate() {
            if !o.layers.intersects(routing) {
                continue;
            }
            let p = &obs_polys[i];
            if !p.bbox().overlaps(&frame) {
                continue;
            }
            add_poly(&mut cdt, p);
        }
        for (a, b) in constraints {
            if a != b {
                cdt.add_constraint_and_split(a, b, |v| v);
            }
        }

        // --- extract
        let verts: Vec<Point> = cdt.vertices().map(|v| pt(v.position().x, v.position().y)).collect();
        let mut tris = Vec::new();
        let mut face_to_tri: HashMap<usize, u32> = HashMap::new();
        for f in cdt.inner_faces() {
            let vs = f.vertices();
            face_to_tri.insert(f.fix().index(), tris.len() as u32);
            tris.push([vs[0].fix().index() as u32, vs[1].fix().index() as u32, vs[2].fix().index() as u32]);
        }
        let mut s = Substrate {
            shared: Arc::new(Shared { layers: layers.clone(), obs_polys, obs_index, sites: Vec::new(), bounds }),
            layers: layers.clone(),
            vert_site: vec![NONE; verts.len()],
            verts,
            tris,
            tri_edges: Vec::new(),
            tri_nbrs: Vec::new(),
            edges: Vec::new(),
            edge_tris: Vec::new(),
            vert_tris: Vec::new(),
            tri_status: Vec::new(),
            obs_tris: Arc::new(Vec::new()),
            demands: Vec::new(),
            tri_sites: Vec::new(),
        };
        s.build_topology();

        // --- classify triangles per routing layer
        let shared = s.shared.clone();
        s.tri_status = layers
            .iter()
            .map(|&l| {
                s.tris
                    .par_iter()
                    .map(|t| {
                        let c = (s.verts[t[0] as usize] + s.verts[t[1] as usize] + s.verts[t[2] as usize]) * (1.0 / 3.0);
                        classify(board, &shared, l, c)
                    })
                    .collect()
            })
            .collect();
        s.obs_tris = Arc::new(
            s.tri_status
                .iter()
                .map(|st| {
                    let mut m: HashMap<u32, Vec<u32>> = HashMap::new();
                    for (t, &o) in st.iter().enumerate() {
                        if o != NONE && o != OUTSIDE {
                            m.entry(o).or_default().push(t as u32);
                        }
                    }
                    m
                })
                .collect(),
        );
        s.demands = (0..layers.len())
            .into_par_iter()
            .map(|li| (0..s.edges.len() as u32).flat_map(|e| { let [a, b] = s.edge_demands(li, e); [a, b] }).collect())
            .collect();

        // --- via sites on a staggered grid, located in free triangles
        let sites = build_sites(board, &shared, cfg.via_pitch);
        s.tri_sites = vec![Vec::new(); s.tris.len()];
        for (i, site) in sites.iter().enumerate() {
            let face = match cdt.locate(Point2::new(site.p.x, site.p.y)) {
                PositionInTriangulation::OnFace(f) => f.index(),
                _ => continue,
            };
            let Some(&t) = face_to_tri.get(&face) else { continue };
            if (0..layers.len()).all(|li| s.tri_status[li][t as usize] == NONE) {
                s.tri_sites[t as usize].push(i as u32);
            }
        }
        drop(shared);
        let mut sh = Arc::try_unwrap(s.shared).ok().expect("shared is unique while building");
        sh.sites = sites;
        s.shared = Arc::new(sh);
        s
    }

    fn build_topology(&mut self) {
        let mut edge_map: HashMap<(u32, u32), u32> = HashMap::with_capacity(self.tris.len() * 2);
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
        self.tri_nbrs = (0..self.tris.len()).map(|ti| self.tri_edges[ti].map(|e| self.other_tri(e, ti as u32))).collect();
        self.vert_tris = vec![Vec::new(); self.verts.len()];
        for (ti, t) in self.tris.iter().enumerate() {
            for &v in t {
                self.vert_tris[v as usize].push(ti as u32);
            }
        }
    }

    pub fn other_tri(&self, e: u32, t: u32) -> u32 {
        let [a, b] = self.edge_tris[e as usize];
        if a == t { b } else { a }
    }

    /// Demands at both ends of edge e on routing layer li.
    fn edge_demands(&self, li: usize, e: u32) -> [Vec<Demand>; 2] {
        let st = &self.tri_status[li];
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

    /// Demands at the `end` (0 = edges[e][0], 1 = edges[e][1]) of edge e on routing layer li.
    pub fn demands(&self, li: usize, e: u32, end: usize) -> &[Demand] {
        &self.demands[li][e as usize * 2 + end]
    }

    pub fn tri_center(&self, t: u32) -> Point {
        let v = self.tris[t as usize];
        (self.verts[v[0] as usize] + self.verts[v[1] as usize] + self.verts[v[2] as usize]) * (1.0 / 3.0)
    }

    pub fn edge_len(&self, e: u32) -> f64 {
        let [a, b] = self.edges[e as usize];
        self.verts[a as usize].dist(self.verts[b as usize])
    }

    /// Local index (0..3) of edge e in triangle t.
    pub fn local_edge(&self, t: u32, e: u32) -> usize {
        self.tri_edges[t as usize].iter().position(|&x| x == e).expect("edge not in triangle")
    }

    /// True if routing triangle status `s` lets net `net` in.
    pub fn passable(board: &Board, s: u32, net: NetId) -> bool {
        s == NONE
            || (s != OUTSIDE
                && board.obstacles[s as usize].net == Some(net)
                && board.obstacles[s as usize].kind != ObstacleKind::Hole)
    }

    pub fn check_orientation(&self) -> usize {
        self.tris
            .iter()
            .filter(|t| orient(self.verts[t[0] as usize], self.verts[t[1] as usize], self.verts[t[2] as usize]) <= 0.0)
            .count()
    }

    /// Split free triangle t at site `site`. Returns the new vertex and the three triangles
    /// X_k = (v_k, v_k+1, s), where X_0 keeps the id t. Edge k of the old triangle is local edge 0
    /// of X_k, the new edge (v_k+1, s) local edge 1 and (s, v_k) local edge 2.
    pub fn split(&mut self, t: u32, site: u32) -> (u32, [u32; 3]) {
        let p = self.shared.sites[site as usize].p;
        let s = self.verts.len() as u32;
        self.verts.push(p);
        self.vert_site.push(site);
        self.vert_tris.push(Vec::new());
        let [v0, v1, v2] = self.tris[t as usize];
        let old_edges = self.tri_edges[t as usize];
        let old_nbrs = self.tri_nbrs[t as usize];
        let x = [t, self.tris.len() as u32, self.tris.len() as u32 + 1];
        // new edges E_i = (v_i, s)
        let e_new: Vec<u32> = (0..3)
            .map(|i| {
                let v = [v0, v1, v2][i];
                self.edges.push([v.min(s), v.max(s)]);
                self.edge_tris.push([NONE, NONE]);
                (self.edges.len() - 1) as u32
            })
            .collect();
        let vs = [v0, v1, v2];
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
                for li in 0..self.layers.len() {
                    let st = self.tri_status[li][t as usize];
                    self.tri_status[li].push(st);
                }
                self.tri_sites.push(Vec::new());
            }
            // old edge k: replace t by X_k on its side
            let oe = old_edges[k] as usize;
            for side in 0..2 {
                if self.edge_tris[oe][side] == t {
                    self.edge_tris[oe][side] = x[k];
                }
            }
            // neighbour across old edge k points to X_k
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
        // vertex -> triangles
        for (k, &v) in vs.iter().enumerate() {
            let list = &mut self.vert_tris[v as usize];
            list.retain(|&q| q != t);
            list.push(x[k]);
            list.push(x[(k + 2) % 3]);
        }
        self.vert_tris[s as usize] = x.to_vec();
        // redistribute the remaining sites
        let rest: Vec<u32> = std::mem::take(&mut self.tri_sites[t as usize]).into_iter().filter(|&q| q != site).collect();
        for q in rest {
            let qp = self.shared.sites[q as usize].p;
            for &xt in &x {
                let tv = self.tris[xt as usize];
                let (a, b, c) = (self.verts[tv[0] as usize], self.verts[tv[1] as usize], self.verts[tv[2] as usize]);
                if orient(a, b, qp) > 0.0 && orient(b, c, qp) > 0.0 && orient(c, a, qp) > 0.0 {
                    self.tri_sites[xt as usize].push(q);
                    break;
                }
            }
        }
        // demands of the touched edges
        for li in 0..self.layers.len() {
            for e in old_edges.iter().chain(e_new.iter()) {
                let [a, b] = self.edge_demands(li, *e);
                let d = &mut self.demands[li];
                let i = *e as usize * 2;
                if d.len() < i + 2 {
                    d.resize(i + 2, Vec::new());
                }
                d[i] = a;
                d[i + 1] = b;
            }
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
        if let ObstacleKind::Keepout { tracks, .. } = o.kind
            && !tracks
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
    candidates
        .par_iter()
        .filter_map(|&p| {
            if board.edge_dist(p) < min_via_r + board.rules.edge_clearance {
                return None;
            }
            let mut nbrs = Vec::new();
            for id in shared.obs_index.query_point(p, 2.5) {
                let o = &board.obstacles[id as usize];
                let d = shared.obs_polys[id as usize].dist_to_point(p);
                if let ObstacleKind::Keepout { vias, .. } = o.kind {
                    if vias && d == 0.0 {
                        return None;
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
                    nbrs.push(SiteNeighbor { obs: id, dist: dd as f32 });
                }
            }
            // Too close to copper of every net: no via fits at all.
            if nbrs.iter().any(|n| (n.dist as f64) < min_via_r) {
                return None;
            }
            Some(Site { p, nbrs })
        })
        .collect()
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::board::*;
    use crate::geom::*;

    pub fn two_pad_board() -> Board {
        let mut b = Board::default();
        b.layers = vec![Layer { name: "F.Cu".into(), is_plane: false }, Layer { name: "B.Cu".into(), is_plane: false }];
        b.outline = vec![Polygon::new(vec![pt(0., 0.), pt(10., 0.), pt(10., 10.), pt(0., 10.)])];
        b.classes = vec![NetClass {
            name: "Default".into(),
            track_width: 0.2,
            clearance: 0.2,
            via_diameter: 0.6,
            via_drill: 0.3,
            priority: 0,
        }];
        b.nets = vec![Net { name: "A".into(), class: 0, pins: vec![0, 1], planes: LayerSet::default(), priority: 0 }];
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
        assert_eq!(s.check_orientation(), 0);
        let owned = s.tri_status[0].iter().filter(|&&x| x != NONE && x != OUTSIDE).count();
        assert!(owned >= 2, "pad triangles {owned}");
        assert!(s.tri_status[0].contains(&OUTSIDE));
        let sites: usize = s.tri_sites.iter().map(|v| v.len()).sum();
        assert!(sites > 100, "sites {sites}");
    }

    #[test]
    fn split_keeps_the_mesh_consistent() {
        let b = two_pad_board();
        let mut s = Substrate::build(&b, &SubstrateConfig::default());
        let t = (0..s.tris.len()).find(|&t| !s.tri_sites[t].is_empty()).unwrap() as u32;
        let site = s.tri_sites[t as usize][0];
        let (v, x) = s.split(t, site);
        assert_eq!(s.check_orientation(), 0);
        for &xt in &x {
            assert!(s.tris[xt as usize].contains(&v));
            for k in 0..3 {
                let e = s.tri_edges[xt as usize][k];
                let [a, b2] = s.edges[e as usize];
                let tv = s.tris[xt as usize];
                let (p, q) = (tv[k], tv[(k + 1) % 3]);
                assert!((a == p && b2 == q) || (a == q && b2 == p));
                assert!(s.edge_tris[e as usize].contains(&xt));
                let nb = s.tri_nbrs[xt as usize][k];
                if nb != NONE {
                    assert!(s.tri_edges[nb as usize].contains(&e));
                }
            }
        }
    }
}
