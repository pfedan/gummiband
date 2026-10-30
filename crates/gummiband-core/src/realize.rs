//! Geometric realization: turn topological segments into taut rubber bands. Each crossing gets a
//! portal on its edge (room for the neighbours and the clearance to the edge ends); the funnel
//! algorithm finds the shortest path through the portals. Every bend of that path wraps around
//! the edge end vertex of its portal, so the bends are replaced with arcs around those pivots and
//! straight tangents between them: neighbours around one pivot become concentric arcs.

use rayon::prelude::*;

use crate::board::{LayerId, NetId, ObstacleKind};
use crate::geom::{Point, arc_polyline, orient, pt};
use crate::route::{End, Item, Router};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Elem {
    Line(Point, Point),
    Arc(Point, Point, Point),
}

#[derive(Debug, Clone)]
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
}

impl Track {
    pub fn width_at(&self, p: Point) -> f64 {
        if self.necks.iter().any(|r| r.contains(p)) {
            self.neck_w
        } else {
            self.width
        }
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

#[derive(Debug, Clone)]
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
}

#[derive(Debug, Clone, Default)]
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
}

/// Circle the band wraps around: `side` +1 = pivot on the left of travel (left turn).
#[derive(Debug, Clone, Copy)]
struct Wrap {
    c: Point,
    r: f64,
    side: f64,
    id: u32,
}

impl Router<'_> {
    fn end_point(&self, e: End) -> Point {
        match e {
            End::Pin(p) => self.board.pins[p].pos,
            End::Via(site) => self.site_point(site),
            End::Point(p) => p,
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
        let slo = Side {
            p: pu + dir * lo,
            pivot: pu,
            pivot_id: u,
        };
        let shi = Side {
            p: pu + dir * hi,
            pivot: pv,
            pivot_id: v,
        };
        // Leaving triangle `from` (counter-clockwise) through edge a->b: b is on the left.
        let t = m.tris[c.from as usize];
        let a = t[m.local_edge(c.from, c.e)];
        if a == u { (shi, slo) } else { (slo, shi) }
    }

    /// Rubber band of one segment: lines and arcs. Starts with the bends of the funnel, then
    /// adds every edge end vertex the band comes too close to as a further wrap, until the band
    /// keeps its distance from all of them.
    pub fn seg_path(&self, sid: u32) -> Vec<Elem> {
        let s = self.segs[sid as usize].as_ref().unwrap();
        let start = self.end_point(s.start);
        let end = self.end_point(s.end);
        let ends = |p: Point| Side {
            p,
            pivot: p,
            pivot_id: u32::MAX,
        };
        let mut portals = Vec::with_capacity(s.crossings.len() + 2);
        portals.push((ends(start), ends(start)));
        for k in 0..s.crossings.len() {
            portals.push(self.portal(sid, k));
        }
        portals.push((ends(end), ends(end)));
        let pts: Vec<(Point, Point)> = portals.iter().map(|(l, r)| (l.p, r.p)).collect();
        let bends = funnel_bends(&pts);
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
        if std::env::var("GB_NO_ARCS").is_ok() || self.soft_on {
            return polyline();
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
        let mut best = band(start, end, &wraps.iter().map(|w| w.1).collect::<Vec<_>>());
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
            let before = wraps.len();
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
            let _ = before;
            match band(start, end, &wraps.iter().map(|w| w.1).collect::<Vec<_>>()) {
                Some(b) => best = Some(b),
                None => break,
            }
        }
        // the band must stay inside the triangles its topology passes through; where the wraps
        // would leave them (and could cross a neighbour), the funnel polyline is the safe choice
        match best {
            Some(b) if self.in_corridor(sid, &flatten(&b)) => b,
            _ => polyline(),
        }
    }

    /// True if the polyline stays in the corridor of seg `sid` (its crossed triangles and the
    /// copper of its own net).
    fn in_corridor(&self, sid: u32, path: &[Point]) -> bool {
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
                self.board.obstacles[o as usize].net == Some(s.net) && self.sub.shared.obs_polys[o as usize].contains(p)
            })
        };
        path.windows(2).all(|w| {
            let n = ((w[0].dist(w[1]) / 0.05).ceil() as usize).clamp(1, 200);
            (0..=n).all(|k| {
                let p = w[0].lerp(w[1], k as f64 / n as f64);
                tris.iter().any(|&t| in_tri(t, p)) || own(p)
            })
        })
    }

    pub fn geometry(&self) -> Geometry {
        if self.soft_on {
            return self.soft_geometry();
        }
        let ids: Vec<u32> = (0..self.segs.len() as u32)
            .filter(|&i| self.segs[i as usize].is_some())
            .collect();
        let tracks: Vec<Track> = ids.par_iter().flat_map_iter(|&sid| self.seg_tracks(sid)).collect();
        let mut vias: Vec<Via> = self
            .vias
            .iter()
            .map(|(&v, info)| Via {
                at: self.site_point(v),
                diameter: info.r * 2.0,
                drill: info.drill,
                net: info.net,
                layer: None,
                origin: None,
                resting: false,
            })
            .collect();
        vias.sort_by(|a, b| a.at.x.total_cmp(&b.at.x).then(a.at.y.total_cmp(&b.at.y)));
        Geometry { tracks, vias }
    }

    /// The tracks of a routed segment: its band without the stretches on kept copper.
    pub(crate) fn seg_tracks(&self, sid: u32) -> Vec<Track> {
        let mut tracks = self.seg_pieces(sid);
        tracks.retain(|t| t.pts.len() >= 2);
        tracks
    }

    fn seg_pieces(&self, sid: u32) -> Vec<Track> {
        {
            {
                let s = self.segs[sid as usize].as_ref().unwrap();
                let elems = self.seg_path(sid);
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
                };
                let kept: Vec<bool> = if self.has_kept {
                    pts.iter().map(|&p| on_kept(p)).collect()
                } else {
                    Vec::new()
                };
                if !kept.iter().any(|&k| k) {
                    return vec![track(pts, elems)];
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
                            let e = run.windows(2).map(|w| Elem::Line(w[0], w[1])).collect();
                            out.push(track(std::mem::take(&mut run), e));
                        } else {
                            run.clear();
                        }
                    }
                }
                if run.len() >= 2 {
                    let e = run.windows(2).map(|w| Elem::Line(w[0], w[1])).collect();
                    out.push(track(run, e));
                }
                out
            }
        }
    }
}

/// Tangent path from `start` around the wraps to `end`, with arcs on the wraps.
fn band(start: Point, end: Point, wraps: &[Wrap]) -> Option<Vec<Elem>> {
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
    for w in circles.windows(2) {
        tangents.push(tangent(&w[0], &w[1])?);
    }
    let mut out = Vec::new();
    for (i, &(a, b)) in tangents.iter().enumerate() {
        if i > 0 {
            let (prev_b, c) = (tangents[i - 1].1, circles[i]);
            if prev_b.dist(a) > 1e-6 {
                // arc around c from prev_b to a, turning to c.side
                let a0 = (prev_b - c.c).y.atan2((prev_b - c.c).x);
                let a1 = (a - c.c).y.atan2((a - c.c).x);
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
                    return None;
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
    Some(out)
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
fn tangent(a: &Wrap, b: &Wrap) -> Option<(Point, Point)> {
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
                    self.board.obstacles[o as usize].net == Some(s.net)
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
