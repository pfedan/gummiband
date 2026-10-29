//! Geometric realization: turn topological segments into taut rubber bands. Each crossing gets a
//! portal on its edge (room for the neighbours and the clearance to the edge ends); the funnel
//! algorithm finds the shortest path through the portals. Every bend of that path wraps around
//! the edge end vertex of its portal, so the bends are replaced with arcs around those pivots and
//! straight tangents between them: neighbours around one pivot become concentric arcs.

use rayon::prelude::*;

use crate::board::{LayerId, NetId};
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
}

#[derive(Debug, Clone)]
pub struct Via {
    pub at: Point,
    pub diameter: f64,
    pub drill: f64,
    pub net: NetId,
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
        let (lo, hi) = match self.pack(li, c.e, &items, None) {
            Some(iv) => {
                // Split the common slack so that the portals of neighbours are disjoint.
                let n = iv.len() as f64;
                let slack = iv.iter().map(|(a, b)| b - a).fold(f64::INFINITY, f64::min).max(0.0);
                let (a, _) = iv[idx];
                (a + slack * idx as f64 / n, a + slack * (idx as f64 + 1.0) / n)
            }
            None => {
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

    /// Rubber band of one segment: lines and arcs.
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
        // Group bends around the same pivot into one wrap.
        let mut wraps: Vec<Wrap> = Vec::new();
        for &(i, left) in &bends {
            let side = if left { portals[i].0 } else { portals[i].1 };
            let r = side.p.dist(side.pivot);
            let sgn = if left { 1.0 } else { -1.0 };
            match wraps.last_mut() {
                Some(w) if w.id == side.pivot_id && w.side == sgn => w.r = w.r.max(r),
                _ => wraps.push(Wrap {
                    c: side.pivot,
                    r,
                    side: sgn,
                    id: side.pivot_id,
                }),
            }
        }
        let polyline = || -> Vec<Elem> {
            let mut p = vec![start];
            for &(i, left) in &bends {
                p.push(if left { portals[i].0.p } else { portals[i].1.p });
            }
            p.push(end);
            p.dedup_by(|a, b| a.dist(*b) < 1e-9);
            p.windows(2).map(|w| Elem::Line(w[0], w[1])).collect()
        };
        if std::env::var("GB_NO_ARCS").is_ok() {
            return polyline();
        }
        let mut circles = vec![Wrap {
            c: start,
            r: 0.0,
            side: 1.0,
            id: u32::MAX,
        }];
        circles.extend(wraps);
        circles.push(Wrap {
            c: end,
            r: 0.0,
            side: 1.0,
            id: u32::MAX,
        });
        // tangent points between consecutive circles
        let mut tangents = Vec::with_capacity(circles.len() - 1);
        for w in circles.windows(2) {
            match tangent(&w[0], &w[1]) {
                Some(t) => tangents.push(t),
                None => return polyline(),
            }
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
                        return polyline();
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
        out
    }

    pub fn geometry(&self) -> Geometry {
        let ids: Vec<u32> = (0..self.segs.len() as u32)
            .filter(|&i| self.segs[i as usize].is_some())
            .collect();
        let tracks: Vec<Track> = ids
            .par_iter()
            .map(|&sid| {
                let s = self.segs[sid as usize].as_ref().unwrap();
                let elems = self.seg_path(sid);
                Track {
                    pts: flatten(&elems),
                    elems,
                    width: s.width,
                    layer: self.sub.layers[s.li as usize],
                    net: s.net,
                }
            })
            .collect();
        let mut vias: Vec<Via> = self
            .vias
            .iter()
            .map(|(&v, info)| Via {
                at: self.site_point(v),
                diameter: info.r * 2.0,
                drill: info.drill,
                net: info.net,
            })
            .collect();
        vias.sort_by(|a, b| a.at.x.total_cmp(&b.at.x).then(a.at.y.total_cmp(&b.at.y)));
        Geometry { tracks, vias }
    }
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
                    .filter(|c| sb.crossings.iter().any(|d| d.e == c.e))
                    .map(|c| {
                        let l = &self.cross[sa.li as usize][c.e as usize];
                        format!("e{}[{:?}]", c.e, l)
                    })
                    .collect();
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
