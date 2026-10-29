//! Elastic optimizer: every track is a chain of points under tension; obstacles and tracks of
//! other nets push with their clearance. All tracks relax in parallel (Jacobi style: each reads
//! the previous state of the others). The topology comes from the router, the relaxation only
//! moves points a little per round, so tracks do not jump over each other; it shortens the bands,
//! spreads them and removes clearance violations.

use rayon::prelude::*;

use crate::board::{Board, ObstacleKind};
use crate::geom::{Point, Rect, point_seg_dist, pt};
use crate::index::GridIndex;
use crate::realize::{Elem, Geometry, Track};
use crate::topo::Substrate;

#[derive(Debug, Clone)]
pub struct RelaxConfig {
    pub iterations: usize,
    /// Target point spacing along a track in mm.
    pub spacing: f64,
    /// Fraction of the way to the neighbour midpoint per round.
    pub tension: f64,
    /// Extra clearance kept for numerical safety in mm.
    pub margin: f64,
    /// Largest move of a point per round in mm.
    pub max_step: f64,
}

impl Default for RelaxConfig {
    fn default() -> Self {
        RelaxConfig {
            iterations: 200,
            spacing: 0.1,
            tension: 0.25,
            margin: 0.003,
            max_step: 0.03,
        }
    }
}

struct Piece {
    a: Point,
    b: Point,
    track: u32,
}

fn resample(pts: &[Point], spacing: f64) -> Vec<Point> {
    let mut out = vec![pts[0]];
    for w in pts.windows(2) {
        let l = w[0].dist(w[1]);
        let n = (l / spacing).ceil().max(1.0) as usize;
        for i in 1..=n {
            out.push(w[0].lerp(w[1], i as f64 / n as f64));
        }
    }
    out.dedup_by(|a, b| a.dist(*b) < 1e-6);
    if out.len() < 2 {
        out.push(*pts.last().unwrap());
    }
    out
}

/// Drop points that lie on the line of their neighbours (within `tol`).
pub fn simplify(pts: &[Point], tol: f64) -> Vec<Point> {
    if pts.len() <= 2 {
        return pts.to_vec();
    }
    let mut keep = vec![false; pts.len()];
    keep[0] = true;
    keep[pts.len() - 1] = true;
    let mut stack = vec![(0usize, pts.len() - 1)];
    while let Some((a, b)) = stack.pop() {
        let mut best = (0.0, 0usize);
        for i in a + 1..b {
            let d = point_seg_dist(pts[i], pts[a], pts[b]);
            if d > best.0 {
                best = (d, i);
            }
        }
        if best.0 > tol {
            keep[best.1] = true;
            stack.push((a, best.1));
            stack.push((best.1, b));
        }
    }
    pts.iter().zip(keep).filter(|(_, k)| *k).map(|(p, _)| *p).collect()
}

pub fn relax(board: &Board, sub: &Substrate, geo: &mut Geometry, cfg: &RelaxConfig) {
    let clr = |n: u32| board.class_of(n).clearance.max(board.rules.min_clearance);
    let mut tracks: Vec<Vec<Point>> = geo.tracks.iter().map(|t| resample(&t.pts, cfg.spacing)).collect();
    let bounds = board.bbox().grow(5.0);
    // vias as static obstacles: (centre, radius, net, clearance)
    let mut vias: Vec<(Point, f64, u32, f64)> = geo
        .vias
        .iter()
        .map(|v| (v.at, v.diameter / 2.0, v.net, clr(v.net)))
        .collect();
    let edge_clr = board.rules.edge_clearance;
    let routing = board.routing_layers();
    // track ends that sit on a via of their net follow the via
    let attach = |p: Point, net: u32, vias: &[(Point, f64, u32, f64)]| {
        vias.iter().position(|v| v.2 == net && v.0.dist(p) < 1e-6)
    };
    let ends: Vec<(Option<usize>, Option<usize>)> = tracks
        .iter()
        .zip(&geo.tracks)
        .map(|(p, t)| (attach(p[0], t.net, &vias), attach(*p.last().unwrap(), t.net, &vias)))
        .collect();

    for it in 0..cfg.iterations {
        // anneal: the last quarter only resolves clearances
        let tension = if it * 4 >= cfg.iterations * 3 { 0.0 } else { cfg.tension };
        if it % 10 == 9 {
            tracks = tracks
                .par_iter()
                .map(|p| resample(&simplify(p, 0.0005), cfg.spacing))
                .collect();
        }
        let mut pieces = Vec::new();
        for (ti, p) in tracks.iter().enumerate() {
            for w in p.windows(2) {
                pieces.push(Piece {
                    a: w[0],
                    b: w[1],
                    track: ti as u32,
                });
            }
        }
        let mut idx = GridIndex::new(bounds, 0.5);
        for (i, pc) in pieces.iter().enumerate() {
            let mut r = Rect::empty();
            r.add(pc.a);
            r.add(pc.b);
            idx.insert(i as u32, &r);
        }
        let mut via_idx = GridIndex::new(bounds, 1.0);
        for (i, v) in vias.iter().enumerate() {
            let mut r = Rect::empty();
            r.add(v.0);
            via_idx.insert(i as u32, &r.grow(v.1));
        }
        // vias move as well: pushed by obstacles, other vias and tracks of other nets
        let moved: Vec<Point> = vias
            .par_iter()
            .enumerate()
            .map(|(vi, &(vc, vr, vn, vclr))| {
                let mut push = pt(0.0, 0.0);
                for oi in sub.shared.obs_index.query_point(vc, vr + 1.0) {
                    let o = &board.obstacles[oi as usize];
                    if !o.layers.intersects(routing) && o.hole.is_none() {
                        continue;
                    }
                    if o.net == Some(vn) && o.kind != ObstacleKind::Hole {
                        continue;
                    }
                    let need = match o.kind {
                        ObstacleKind::Keepout { vias: no_vias, .. } => {
                            if !no_vias {
                                continue;
                            }
                            0.0
                        }
                        ObstacleKind::Hole => board.rules.hole_clearance,
                        _ => o.clearance.max(vclr),
                    } + vr
                        + cfg.margin;
                    let poly = &sub.shared.obs_polys[oi as usize];
                    let inside = poly.contains(vc);
                    let (d, q) = nearest_on_poly(poly, vc);
                    let d = if inside { -d } else { d };
                    if d < need {
                        let dir = if inside { (q - vc).norm() } else { (vc - q).norm() };
                        push = push + dir * (need - d);
                    }
                }
                for (vj, &(c2, r2, n2, clr2)) in vias.iter().enumerate() {
                    if vj == vi || n2 == vn {
                        continue;
                    }
                    let need = vr + r2 + vclr.max(clr2) + cfg.margin;
                    let d = vc.dist(c2);
                    if d < need && d > 1e-9 {
                        push = push + (vc - c2).norm() * ((need - d) * 0.5);
                    }
                }
                for pi in idx.query_point(vc, vr + 1.5) {
                    let pc = &pieces[pi as usize];
                    let o = &geo.tracks[pc.track as usize];
                    if o.net == vn {
                        continue;
                    }
                    let need = vr + o.piece_width(pc.a, pc.b) / 2.0 + vclr.max(clr(o.net)) + cfg.margin;
                    let ab = pc.b - pc.a;
                    let l2 = ab.len2();
                    let tt = if l2 > 0.0 {
                        ((vc - pc.a).dot(ab) / l2).clamp(0.0, 1.0)
                    } else {
                        0.0
                    };
                    let q = pc.a + ab * tt;
                    let d = vc.dist(q);
                    if d < need {
                        let dir = if d > 1e-9 { (vc - q).norm() } else { ab.perp().norm() };
                        push = push + dir * ((need - d) * 0.5);
                    }
                }
                let l = push.len();
                let mv = if l > cfg.max_step {
                    push * (cfg.max_step / l)
                } else {
                    push
                };
                vc + mv
            })
            .collect();
        let snapshot = &tracks;
        tracks = snapshot
            .par_iter()
            .enumerate()
            .map(|(ti, pts)| {
                let t = &geo.tracks[ti];
                let (c, net, layer) = (clr(t.net), t.net, t.layer);
                let mut out = pts.clone();
                let n = pts.len();
                for i in 1..n.saturating_sub(1) {
                    let p = pts[i];
                    let w = t.width_at(p);
                    // tension
                    let mid = (pts[i - 1] + pts[i + 1]) * 0.5;
                    let mut mv = (mid - p) * tension;
                    let mut push = pt(0.0, 0.0);
                    let reach = w / 2.0 + 2.0;
                    // static obstacles
                    for oi in sub.shared.obs_index.query_point(p, reach) {
                        let o = &board.obstacles[oi as usize];
                        if !o.layers.has(layer) || (o.net == Some(net) && o.kind != ObstacleKind::Hole) {
                            continue;
                        }
                        let need = match o.kind {
                            ObstacleKind::Keepout { tracks, .. } => {
                                if !tracks {
                                    continue;
                                }
                                0.0
                            }
                            ObstacleKind::Hole => board.rules.hole_clearance,
                            _ => o.clearance.max(c),
                        } + w / 2.0
                            + cfg.margin;
                        let poly = &sub.shared.obs_polys[oi as usize];
                        let inside = poly.contains(p);
                        let (d, q) = nearest_on_poly(poly, p);
                        let d = if inside { -d } else { d };
                        if d < need {
                            let dir = if inside { (q - p).norm() } else { (p - q).norm() };
                            push = push + dir * (need - d);
                        }
                    }
                    // board edge
                    for o in &board.outline {
                        let (d, q) = nearest_on_poly(o, p);
                        let need = edge_clr + w / 2.0 + cfg.margin;
                        if d < need {
                            let inside_board = board.inside(p);
                            let dir = if inside_board { (p - q).norm() } else { (q - p).norm() };
                            let d = if inside_board { d } else { -d };
                            push = push + dir * (need - d);
                        }
                    }
                    // vias of other nets
                    for vi in via_idx.query_point(p, reach) {
                        let (vc, vr, vn, vclr) = vias[vi as usize];
                        if vn == net {
                            continue;
                        }
                        let need = vr + vclr.max(c) + w / 2.0 + cfg.margin;
                        let d = p.dist(vc);
                        if d < need {
                            push = push + (p - vc).norm() * ((need - d) * 0.5);
                        }
                    }
                    // other tracks (they push back from their side, so take half)
                    for pi in idx.query_point(p, reach) {
                        let pc = &pieces[pi as usize];
                        let o = &geo.tracks[pc.track as usize];
                        if pc.track as usize == ti || o.net == net || o.layer != layer {
                            continue;
                        }
                        let need =
                            w / 2.0 + o.piece_width(pc.a, pc.b) / 2.0 + board.track_clearance(net, o.net) + cfg.margin;
                        let ab = pc.b - pc.a;
                        let l2 = ab.len2();
                        let tt = if l2 > 0.0 {
                            ((p - pc.a).dot(ab) / l2).clamp(0.0, 1.0)
                        } else {
                            0.0
                        };
                        let q = pc.a + ab * tt;
                        let d = p.dist(q);
                        if d < need {
                            let dir = if d > 1e-9 { (p - q).norm() } else { ab.perp().norm() };
                            push = push + dir * ((need - d) * 0.5);
                        }
                    }
                    // constraints win over tension: drop the tension part that points into the push
                    if push.len2() > 0.0 {
                        let pn = push.norm();
                        let into = mv.dot(pn);
                        if into < 0.0 {
                            mv = mv - pn * into;
                        }
                        mv = mv + push;
                    }
                    let l = mv.len();
                    if l > cfg.max_step {
                        mv = mv * (cfg.max_step / l);
                    }
                    out[i] = p + mv;
                }
                out
            })
            .collect();
        for (v, p) in vias.iter_mut().zip(moved) {
            v.0 = p;
        }
        for (pts, (a, b)) in tracks.iter_mut().zip(&ends) {
            if let Some(a) = a {
                pts[0] = vias[*a].0;
            }
            if let Some(b) = b {
                let n = pts.len();
                pts[n - 1] = vias[*b].0;
            }
        }
    }
    for (v, (p, ..)) in geo.vias.iter_mut().zip(&vias) {
        v.at = *p;
    }
    for (t, pts) in geo.tracks.iter_mut().zip(tracks) {
        let s = simplify(&pts, 0.001);
        *t = Track {
            elems: s.windows(2).map(|w| Elem::Line(w[0], w[1])).collect(),
            pts: s,
            width: t.width,
            layer: t.layer,
            net: t.net,
            necks: std::mem::take(&mut t.necks),
            neck_w: t.neck_w,
        };
    }
}

/// Distance from p to the polygon boundary and the nearest boundary point.
fn nearest_on_poly(poly: &crate::geom::Polygon, p: Point) -> (f64, Point) {
    let mut best = (f64::INFINITY, p);
    for (a, b) in poly.edges() {
        let ab = b - a;
        let l2 = ab.len2();
        let t = if l2 > 0.0 {
            ((p - a).dot(ab) / l2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let q = a + ab * t;
        let d = p.dist(q);
        if d < best.0 {
            best = (d, q);
        }
    }
    best
}
