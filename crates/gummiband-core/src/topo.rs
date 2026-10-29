//! Routing substrate: one constrained Delaunay triangulation of all obstacle points of all
//! routing layers plus free via sites. Per routing layer every triangle is free or covered by an
//! obstacle; edge ends carry the clearance demands of the obstacles that meet there.

use std::collections::HashMap;

use rayon::prelude::*;
use spade::handles::FixedVertexHandle;
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};

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
        SubstrateConfig { via_pitch: 0.5 }
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

pub struct Substrate {
    pub verts: Vec<Point>,
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
    /// Routing layers (board layer ids).
    pub layers: Vec<LayerId>,
    /// Per routing layer, per triangle: NONE (free), OUTSIDE or the covering obstacle.
    pub tri_status: Vec<Vec<u32>>,
    /// Per routing layer: demands for edge end (2 * edge + end), flattened with offsets.
    demand_off: Vec<Vec<u32>>,
    demands: Vec<Vec<Demand>>,
    /// Via sites: vertices far enough from every obstacle for some via.
    pub is_site: Vec<bool>,
    pub site_nbrs: HashMap<u32, Vec<SiteNeighbor>>,
    /// Obstacle polygons (as used for the triangulation).
    pub obs_polys: Vec<Polygon>,
    pub obs_index: GridIndex,
    pub bounds: Rect,
}

fn snap(v: f64) -> f64 {
    (v * 1e5).round() / 1e5
}

impl Substrate {
    pub fn build(board: &Board, cfg: &SubstrateConfig) -> Substrate {
        let me = board.rules.max_error;
        let layers: Vec<LayerId> = board.routing_layers().iter().collect();
        let routing = board.routing_layers();
        let obs_polys: Vec<Polygon> = board
            .obstacles
            .par_iter()
            .map(|o| o.shape.to_polygon(me))
            .collect();
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
        let mut cdt: ConstrainedDelaunayTriangulation<Point2<f64>> =
            ConstrainedDelaunayTriangulation::new();
        let mut handle_of: HashMap<(i64, i64), FixedVertexHandle> = HashMap::new();
        let mut insert = |cdt: &mut ConstrainedDelaunayTriangulation<Point2<f64>>,
                          p: Point|
         -> Option<FixedVertexHandle> {
            let key = ((p.x * 1e5).round() as i64, (p.y * 1e5).round() as i64);
            if let Some(&h) = handle_of.get(&key) {
                return Some(h);
            }
            let h = cdt.insert(Point2::new(snap(p.x), snap(p.y))).ok()?;
            handle_of.insert(key, h);
            Some(h)
        };
        let mut constraints: Vec<(FixedVertexHandle, FixedVertexHandle)> = Vec::new();
        let mut add_poly = |cdt: &mut ConstrainedDelaunayTriangulation<Point2<f64>>,
                            poly: &Polygon| {
            let hs: Vec<_> = poly.pts.iter().filter_map(|&p| insert(cdt, p)).collect();
            for i in 0..hs.len() {
                let (a, b) = (hs[i], hs[(i + 1) % hs.len()]);
                if a != b {
                    constraints.push((a, b));
                }
            }
        };
        // Outline: a frame around the board so that outside triangles exist and get classified.
        let frame = bounds.grow(2.0);
        add_poly(
            &mut cdt,
            &Polygon::new(vec![
                frame.min,
                pt(frame.max.x, frame.min.y),
                frame.max,
                pt(frame.min.x, frame.max.y),
            ]),
        );
        for o in &board.outline {
            add_poly(&mut cdt, o);
        }
        for (i, o) in board.obstacles.iter().enumerate() {
            if !o.layers.intersects(routing) {
                continue;
            }
            // Clip huge obstacles (keepouts reaching beyond the board) to the frame region.
            let p = &obs_polys[i];
            if !p.bbox().overlaps(&frame) {
                continue;
            }
            add_poly(&mut cdt, p);
        }

        // Via sites on a staggered grid, only where a smallest via fits.
        let min_via_r = board.rules.min_via_diameter / 2.0;
        let pitch = cfg.via_pitch;
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
        let site_info: Vec<Option<(Point, Vec<SiteNeighbor>)>> = candidates
            .par_iter()
            .map(|&p| {
                if board.edge_dist(p) < min_via_r + board.rules.edge_clearance {
                    return None;
                }
                let mut nbrs = Vec::new();
                for id in obs_index.query_point(p, 2.0) {
                    let o = &board.obstacles[id as usize];
                    let d = obs_polys[id as usize].dist_to_point(p);
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
                    if dd < 2.0 {
                        nbrs.push(SiteNeighbor {
                            obs: id,
                            dist: dd as f32,
                        });
                    }
                }
                // Too close to copper of every net: no via fits at all.
                if nbrs.iter().any(|n| (n.dist as f64) < min_via_r) {
                    return None;
                }
                Some((p, nbrs))
            })
            .collect();
        let mut site_pts: HashMap<u32, Vec<SiteNeighbor>> = HashMap::new();
        for (p, nbrs) in site_info.into_iter().flatten() {
            if let Some(h) = insert(&mut cdt, p) {
                site_pts.insert(h.index() as u32, nbrs);
            }
        }
        for (a, b) in constraints {
            if a != b {
                cdt.add_constraint_and_split(a, b, |v| v);
            }
        }

        // --- extract
        let verts: Vec<Point> = cdt
            .vertices()
            .map(|v| pt(v.position().x, v.position().y))
            .collect();
        let mut tris = Vec::new();
        for f in cdt.inner_faces() {
            let vs = f.vertices();
            tris.push([
                vs[0].fix().index() as u32,
                vs[1].fix().index() as u32,
                vs[2].fix().index() as u32,
            ]);
        }
        // spade faces are counter-clockwise in its (y up) frame, which is what we use.
        let mut edge_map: HashMap<(u32, u32), u32> = HashMap::with_capacity(tris.len() * 2);
        let mut edges: Vec<[u32; 2]> = Vec::new();
        let mut edge_tris: Vec<[u32; 2]> = Vec::new();
        let mut tri_edges = vec![[0u32; 3]; tris.len()];
        for (ti, t) in tris.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                let key = (a.min(b), a.max(b));
                let e = *edge_map.entry(key).or_insert_with(|| {
                    edges.push([key.0, key.1]);
                    edge_tris.push([NONE, NONE]);
                    (edges.len() - 1) as u32
                });
                let slot = if edge_tris[e as usize][0] == NONE {
                    0
                } else {
                    1
                };
                edge_tris[e as usize][slot] = ti as u32;
                tri_edges[ti][k] = e;
            }
        }
        let tri_nbrs: Vec<[u32; 3]> = tri_edges
            .iter()
            .enumerate()
            .map(|(ti, es)| {
                es.map(|e| {
                    let [a, b] = edge_tris[e as usize];
                    if a == ti as u32 { b } else { a }
                })
            })
            .collect();
        let mut vert_tris = vec![Vec::new(); verts.len()];
        for (ti, t) in tris.iter().enumerate() {
            for &v in t {
                vert_tris[v as usize].push(ti as u32);
            }
        }

        // --- classify triangles per routing layer
        let tri_status: Vec<Vec<u32>> = layers
            .iter()
            .map(|&l| {
                tris.par_iter()
                    .map(|t| {
                        let c =
                            (verts[t[0] as usize] + verts[t[1] as usize] + verts[t[2] as usize])
                                * (1.0 / 3.0);
                        if !board.inside(c) {
                            return OUTSIDE;
                        }
                        let mut found = NONE;
                        for id in obs_index.query_point(c, 0.0) {
                            let o = &board.obstacles[id as usize];
                            if !o.layers.has(l) {
                                continue;
                            }
                            if let ObstacleKind::Keepout { tracks, .. } = o.kind
                                && !tracks
                            {
                                continue;
                            }
                            if obs_polys[id as usize].contains(c) {
                                // an obstacle without net wins (it blocks everyone)
                                if found == NONE || o.net.is_none() {
                                    found = id;
                                }
                            }
                        }
                        found
                    })
                    .collect()
            })
            .collect();

        let mut s = Substrate {
            verts,
            tris,
            tri_edges,
            tri_nbrs,
            edges,
            edge_tris,
            vert_tris,
            layers,
            tri_status,
            demand_off: Vec::new(),
            demands: Vec::new(),
            is_site: Vec::new(),
            site_nbrs: site_pts,
            obs_polys,
            obs_index,
            bounds,
        };
        s.is_site = (0..s.verts.len() as u32)
            .map(|v| s.site_nbrs.contains_key(&v))
            .collect();
        // A site that ended up on an obstacle triangle corner (split constraint) is no site.
        for v in 0..s.verts.len() {
            if s.is_site[v]
                && s.vert_tris[v]
                    .iter()
                    .any(|&t| (0..s.layers.len()).any(|l| s.tri_status[l][t as usize] != NONE))
            {
                s.is_site[v] = false;
            }
        }
        s.build_demands();
        s
    }

    /// Clearance demands at both ends of every edge, per routing layer.
    fn build_demands(&mut self) {
        let per_layer: Vec<(Vec<u32>, Vec<Demand>)> = (0..self.layers.len())
            .into_par_iter()
            .map(|li| {
                let st = &self.tri_status[li];
                // obstacles touching each vertex
                let vert_obs: Vec<Vec<u32>> = self
                    .vert_tris
                    .iter()
                    .map(|ts| {
                        let mut v: Vec<u32> = ts
                            .iter()
                            .map(|&t| st[t as usize])
                            .filter(|&s| s != NONE)
                            .collect();
                        v.sort_unstable();
                        v.dedup();
                        v
                    })
                    .collect();
                let mut off = Vec::with_capacity(self.edges.len() * 2 + 1);
                let mut out = Vec::new();
                for (e, &[a, b]) in self.edges.iter().enumerate() {
                    for (end, (u, v)) in [(a, b), (b, a)].into_iter().enumerate() {
                        let _ = end;
                        off.push(out.len() as u32);
                        let start = out.len();
                        for &o in &vert_obs[u as usize] {
                            out.push(Demand {
                                obs: o,
                                factor: 1.0,
                            });
                        }
                        let pu = self.verts[u as usize];
                        let dir = (self.verts[v as usize] - pu).norm();
                        for &t in &self.edge_tris[e] {
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
                            let ob = if beyond == NONE {
                                OUTSIDE
                            } else {
                                st[beyond as usize]
                            };
                            if ob == NONE {
                                continue;
                            }
                            let side = (self.verts[w as usize] - pu).norm();
                            let sin = dir.cross(side).abs();
                            let cos = dir.dot(side);
                            if cos <= 0.0 {
                                continue; // obtuse: the vertex demand covers it
                            }
                            let factor = (1.0 / sin.max(0.25)) as f32;
                            if let Some(d) = out[start..].iter_mut().find(|d| d.obs == ob) {
                                d.factor = d.factor.max(factor);
                            } else {
                                out.push(Demand { obs: ob, factor });
                            }
                        }
                    }
                }
                off.push(out.len() as u32);
                (off, out)
            })
            .collect();
        for (off, d) in per_layer {
            self.demand_off.push(off);
            self.demands.push(d);
        }
    }

    /// Demands at the `end` (0 = edges[e][0], 1 = edges[e][1]) of edge e on routing layer li.
    pub fn demands(&self, li: usize, e: u32, end: usize) -> &[Demand] {
        let i = e as usize * 2 + end;
        let off = &self.demand_off[li];
        &self.demands[li][off[i] as usize..off[i + 1] as usize]
    }

    pub fn tri_center(&self, t: u32) -> Point {
        let v = self.tris[t as usize];
        (self.verts[v[0] as usize] + self.verts[v[1] as usize] + self.verts[v[2] as usize])
            * (1.0 / 3.0)
    }

    pub fn edge_len(&self, e: u32) -> f64 {
        let [a, b] = self.edges[e as usize];
        self.verts[a as usize].dist(self.verts[b as usize])
    }

    /// Local index (0..3) of edge e in triangle t.
    pub fn local_edge(&self, t: u32, e: u32) -> usize {
        self.tri_edges[t as usize]
            .iter()
            .position(|&x| x == e)
            .expect("edge not in triangle")
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
            .filter(|t| {
                orient(
                    self.verts[t[0] as usize],
                    self.verts[t[1] as usize],
                    self.verts[t[2] as usize],
                ) <= 0.0
            })
            .count()
    }
}

#[cfg(test)]
mod tests {
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
        b.outline = vec![Polygon::new(vec![
            pt(0., 0.),
            pt(10., 0.),
            pt(10., 10.),
            pt(0., 10.),
        ])];
        b.classes = vec![NetClass {
            name: "Default".into(),
            track_width: 0.2,
            clearance: 0.2,
            via_diameter: 0.6,
            via_drill: 0.3,
            priority: 0,
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
        let owned = s.tri_status[0]
            .iter()
            .filter(|&&x| x != NONE && x != OUTSIDE)
            .count();
        assert!(owned >= 4, "pad triangles {owned}");
        assert!(s.tri_status[0].contains(&OUTSIDE));
        assert!(s.is_site.iter().filter(|&&x| x).count() > 100);
        // every interior edge has two triangles
        assert!(s.edge_tris.iter().filter(|t| t[1] != NONE).count() > s.edges.len() / 2);
    }
}
