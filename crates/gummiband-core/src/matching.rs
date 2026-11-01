//! Lengths of nets and the coupling of differential pairs, measured as KiCad's DRC measures them,
//! and the check of the length, skew and uncoupled length rules of the board.
//!
//! - Length (`LENGTH_DELAY_CALCULATION`): the tracks (KiCad clips only lines that end exactly in
//!   a pad's centre and keeps their length when they enter straight), plus every via over the
//!   layers its tracks really use (the stackup distance between them, a via with tracks on one
//!   layer counts nothing), plus the pad-to-die lengths.
//! - Coupling (`DRC_TEST_PROVIDER_DIFF_PAIR_COUPLING`): every line of the P net against every
//!   line of the N net on the same layer that is exactly parallel to it on the nanometre grid of
//!   the file and overlaps it in projection, and arcs against arcs about nearly the same centre
//!   with radii less than half apart; the overlap counts if nothing else lies between the two and
//!   the gap is inside the `diff_pair_gap` rule (if it has a min or max). Uncoupled is the longer
//!   net's track length minus what counts. Lines a hair off parallel, as any-angle offsets
//!   rounded to the grid are, do not count at all.

use std::collections::HashMap;

use crate::board::{Board, LayerId, MatchKind, MinOptMax, NetId, ObstacleKind};
use crate::geom::{self, Point};
use crate::realize::{Elem, Geometry, Track};

/// Tolerance of KiCad's DRC (its epsilon) for lengths and gaps, in mm.
const EPS: f64 = 0.0005;

#[derive(Debug, Clone, Copy, Default, PartialEq, serde::Serialize)]
pub struct NetLength {
    pub tracks: f64,
    pub vias: f64,
    pub die: f64,
}

impl NetLength {
    pub fn total(&self) -> f64 {
        self.tracks + self.vias + self.die
    }
}

/// Length of an element.
pub fn elem_len(e: &Elem) -> f64 {
    match *e {
        Elem::Line(a, b) => a.dist(b),
        Elem::Arc(a, m, b) => match circle(a, m, b) {
            Some((c, r)) => r * sweep(a, m, b, c).abs(),
            None => a.dist(b),
        },
    }
}

/// Raw track length, as KiCad's `GetLength` of its items.
pub fn track_len(t: &Track) -> f64 {
    t.elems.iter().map(elem_len).sum()
}

/// Circle through three points.
pub fn circle(a: Point, m: Point, b: Point) -> Option<(Point, f64)> {
    let d = 2.0 * (a.x * (m.y - b.y) + m.x * (b.y - a.y) + b.x * (a.y - m.y));
    if d.abs() < 1e-12 {
        return None;
    }
    let sq = |p: Point| p.x * p.x + p.y * p.y;
    let ux = (sq(a) * (m.y - b.y) + sq(m) * (b.y - a.y) + sq(b) * (a.y - m.y)) / d;
    let uy = (sq(a) * (b.x - m.x) + sq(m) * (a.x - b.x) + sq(b) * (m.x - a.x)) / d;
    let c = Point { x: ux, y: uy };
    Some((c, c.dist(a)))
}

fn ang(p: Point, c: Point) -> f64 {
    (p.y - c.y).atan2(p.x - c.x)
}

fn norm_pi(mut a: f64) -> f64 {
    while a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    }
    while a <= -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}

/// Signed sweep of the arc a-m-b about c (counter-clockwise positive, in the maths sense).
pub fn sweep(a: Point, m: Point, b: Point, c: Point) -> f64 {
    norm_pi(ang(m, c) - ang(a, c)) + norm_pi(ang(b, c) - ang(m, c))
}

/// Length of a net as KiCad's DRC counts it for `length` and `skew` rules.
pub fn net_length(board: &Board, geo: &Geometry, net: NetId) -> NetLength {
    let tracks: Vec<&Track> = geo.tracks.iter().filter(|t| t.net == net && t.pts.len() >= 2).collect();
    let mut len = NetLength::default();
    for t in &tracks {
        len.tracks += track_len(t);
    }
    for v in geo.vias.iter().filter(|v| v.net == net && v.drill > 0.0) {
        let r = v.diameter / 2.0 + 1e-6;
        let layers: Vec<LayerId> = tracks
            .iter()
            .filter(|t| t.pts[0].dist(v.at) < r || t.pts.last().unwrap().dist(v.at) < r)
            .map(|t| t.layer)
            .collect();
        if let (Some(lo), Some(hi)) = (layers.iter().min(), layers.iter().max()) {
            len.vias += board.layer_distance(*lo, *hi);
        }
    }
    len.die = board.nets[net as usize]
        .pins
        .iter()
        .filter_map(|p| board.die_length.get(p))
        .sum();
    len
}

/// Coupled and total length of a differential pair: (coupled, longer net's track length).
pub fn coupling(board: &Board, geo: &Geometry, p: NetId, n: NetId, gap: MinOptMax) -> (f64, f64) {
    let tp: Vec<&Track> = geo.tracks.iter().filter(|t| t.net == p).collect();
    let tn: Vec<&Track> = geo.tracks.iter().filter(|t| t.net == n).collect();
    let total = tp
        .iter()
        .map(|t| track_len(t))
        .sum::<f64>()
        .max(tn.iter().map(|t| track_len(t)).sum());
    let others: Vec<&Track> = geo.tracks.iter().filter(|t| t.net != p && t.net != n).collect();
    let foreign =
        |a: Point, b: Point, layer: LayerId| -> bool {
            others.iter().any(|t| {
                t.layer == layer
                    && t.pts
                        .windows(2)
                        .any(|w| geom::seg_seg_dist(a, b, w[0], w[1]) < t.width / 2.0)
            }) || geo.vias.iter().any(|v| {
                v.net != p && v.net != n && v.drill > 0.0 && geom::point_seg_dist(v.at, a, b) < v.diameter / 2.0
            }) || board.obstacles.iter().any(|o| {
                o.kind == ObstacleKind::Pad
                    && o.net != Some(p)
                    && o.net != Some(n)
                    && o.layers.has(layer)
                    && o.shape.dist_to_seg(a, b) <= 0.0
            })
        };
    let fits = |g: f64| gap.min.is_none_or(|m| g >= m - EPS) && gap.max.is_none_or(|m| g <= m + EPS);
    let mut coupled = 0.0;
    for a in &tp {
        for b in tn.iter().filter(|b| b.layer == a.layer) {
            let half = (a.width + b.width) / 2.0;
            for ea in &a.elems {
                for eb in &b.elems {
                    match (*ea, *eb) {
                        (Elem::Line(p0, p1), Elem::Line(n0, n1)) => {
                            if p0.dist(p1) < 2e-6 || n0.dist(n1) < 2e-6 || !grid_parallel(p0, p1, n0, n1) {
                                continue;
                            }
                            let Some((pa, pb, na, nb)) = parallel_projection(p0, p1, n0, n1) else {
                                continue;
                            };
                            let (np, nn, d) = nearest(pa, pb, na, nb);
                            if fits(d - half) && !foreign(np, nn, a.layer) {
                                coupled += na.dist(nb);
                            }
                        }
                        (Elem::Arc(p0, pm, p1), Elem::Arc(n0, nm, n1)) => {
                            let (Some((cp, rp)), Some((cn, rn))) = (circle(p0, pm, p1), circle(n0, nm, n1)) else {
                                continue;
                            };
                            if cp.dist(cn) >= a.width
                                || elem_len(ea) <= 2e-6
                                || elem_len(eb) <= 2e-6
                                || (rp - rn).abs() / rp.max(rn) > 0.5
                                || cp.dist(cn) > rp.max(rn) * 0.5
                            {
                                continue;
                            }
                            let Some((from, span)) = arc_overlap(p0, pm, p1, cp, n0, nm, n1, cn) else {
                                continue;
                            };
                            let mid = from + span / 2.0;
                            let at = |c: Point, r: f64| Point {
                                x: c.x + r * mid.cos(),
                                y: c.y + r * mid.sin(),
                            };
                            let d = (rp - rn).abs();
                            if fits(d - half) && !foreign(at(cp, rp), at(cn, rn), a.layer) {
                                coupled += rn * span.abs();
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
    }
    (coupled, total)
}

/// The two lines count as parallel for KiCad: its `SEG::Intersect` of the infinite lines on the
/// nanometre grid of the file finds no crossing. That is the case if they are exactly parallel
/// (but not on one line), or if they cross so far away that the point does not fit into KiCad's
/// 32 bit coordinates (about 2.1 m): lines a hair more off parallel do not count at all.
pub fn grid_parallel(p0: Point, p1: Point, n0: Point, n1: Point) -> bool {
    let g = |v: f64| (v * 1e6).round() as i128;
    let (e, f) = ((g(p1.x) - g(p0.x)), (g(p1.y) - g(p0.y)));
    let (h, k) = ((g(n1.x) - g(n0.x)), (g(n1.y) - g(n0.y)));
    let (ox, oy) = ((g(n0.x) - g(p0.x)), (g(n0.y) - g(p0.y)));
    // determinant f x e as KiCad has it (dir2.Cross(dir1))
    let det = h * f - k * e;
    if det == 0 {
        return e * oy - f * ox != 0;
    }
    // the crossing: n0 + (q / det) * (h, k) with q = dir1 x offset
    let q = e * oy - f * ox;
    let x = g(n0.x) as f64 + q as f64 * h as f64 / det as f64;
    let y = g(n0.y) as f64 + q as f64 * k as f64 / det as f64;
    let lim = i32::MAX as f64;
    x.abs() > lim || y.abs() > lim
}

/// KiCad's `commonParallelProjection` of two lines: the piece of p that n overlaps when projected
/// on p's line, and the projection of that piece back on n's line.
fn parallel_projection(p0: Point, p1: Point, n0: Point, n1: Point) -> Option<(Point, Point, Point, Point)> {
    let d = p1 - p0;
    let ll = d.x * d.x + d.y * d.y;
    let t = |q: Point| ((q - p0).x * d.x + (q - p0).y * d.y) / ll;
    let (ta, tb) = (0.0, 1.0);
    let (mut qa, mut qb) = (t(n0), t(n1));
    if qb < qa {
        std::mem::swap(&mut qa, &mut qb);
    }
    if tb <= qa || ta >= qb {
        return None;
    }
    let lo = ta.max(qa);
    let hi = tb.min(qb);
    let pa = p0 + d * lo;
    let pb = p0 + d * hi;
    let proj = |q: Point| {
        let e = n1 - n0;
        let s = ((q - n0).x * e.x + (q - n0).y * e.y) / (e.x * e.x + e.y * e.y);
        n0 + e * s
    };
    Some((pa, pb, proj(pa), proj(pb)))
}

/// Nearest points of two segments and their distance.
fn nearest(a0: Point, a1: Point, b0: Point, b1: Point) -> (Point, Point, f64) {
    let mut best = (a0, b0, f64::INFINITY);
    for (p, s0, s1, first) in [
        (a0, b0, b1, true),
        (a1, b0, b1, true),
        (b0, a0, a1, false),
        (b1, a0, a1, false),
    ] {
        let e = s1 - s0;
        let ll = e.x * e.x + e.y * e.y;
        let t = if ll > 0.0 {
            (((p - s0).x * e.x + (p - s0).y * e.y) / ll).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let q = s0 + e * t;
        let d = p.dist(q);
        if d < best.2 {
            best = if first { (p, q, d) } else { (q, p, d) };
        }
    }
    best
}

/// Overlap of two arcs in angle about the first centre: (start angle, sweep).
#[allow(clippy::too_many_arguments)]
fn arc_overlap(
    p0: Point,
    pm: Point,
    p1: Point,
    cp: Point,
    n0: Point,
    nm: Point,
    n1: Point,
    cn: Point,
) -> Option<(f64, f64)> {
    let interval = |a: Point, m: Point, b: Point, c: Point| {
        let s = sweep(a, m, b, c);
        let a0 = ang(a, c);
        if s >= 0.0 { (a0, s) } else { (a0 + s, -s) }
    };
    let (ps, pl) = interval(p0, pm, p1, cp);
    let (ns, nl) = interval(n0, nm, n1, cn);
    // n's interval relative to p's start
    let off = (ns - ps).rem_euclid(2.0 * std::f64::consts::PI);
    let mut best: Option<(f64, f64)> = None;
    for o in [off, off - 2.0 * std::f64::consts::PI] {
        let lo = o.max(0.0);
        let hi = (o + nl).min(pl);
        if hi - lo > 1e-9 && best.is_none_or(|b| hi - lo > b.1) {
            best = Some((ps + lo, hi - lo));
        }
    }
    best
}

/// Result of one net under a rule.
#[derive(Debug, Clone, serde::Serialize)]
pub struct NetMatch {
    pub net: String,
    /// The length measured (for `uncoupled`: the uncoupled length of the pair).
    pub length: f64,
    /// For skew: the length difference to the target (negative if shorter).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub skew: Option<f64>,
    /// The length this net should have to meet the rule, if it does not.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target: Option<f64>,
    pub ok: bool,
}

/// Result of one rule.
#[derive(Debug, Clone, serde::Serialize)]
pub struct RuleMatch {
    pub rule: String,
    pub kind: MatchKind,
    pub value: MinOptMax,
    pub within_pairs: bool,
    pub nets: Vec<NetMatch>,
}

impl RuleMatch {
    pub fn failed(&self) -> usize {
        self.nets.iter().filter(|n| !n.ok).count()
    }
}

/// The `diff_pair_gap` constraint that holds for the pair of `net` (none: every gap counts).
pub fn gap_rule(board: &Board, net: NetId) -> MinOptMax {
    board
        .match_rules
        .iter()
        .find(|r| r.kind == MatchKind::Gap && r.nets.contains(&net))
        .map(|r| r.value)
        .unwrap_or_default()
}

/// The partner of a net in its differential pair.
pub fn partner(board: &Board, net: NetId) -> Option<NetId> {
    board.diff_pairs().into_iter().find_map(|(p, n)| {
        if p == net {
            Some(n)
        } else if n == net {
            Some(p)
        } else {
            None
        }
    })
}

/// Check every length, skew and uncoupled rule of the board on the geometry.
pub fn check(board: &Board, geo: &Geometry) -> Vec<RuleMatch> {
    let mut lengths: HashMap<NetId, f64> = HashMap::new();
    let mut length = |net: NetId| {
        *lengths
            .entry(net)
            .or_insert_with(|| net_length(board, geo, net).total())
    };
    let name = |net: NetId| board.nets[net as usize].name.clone();
    let mut out = Vec::new();
    for r in board.match_rules.iter().filter(|r| r.kind != MatchKind::Gap) {
        let mut nets = Vec::new();
        match r.kind {
            MatchKind::Length => {
                for &x in &r.nets {
                    let l = length(x);
                    let short = r.value.min.is_some_and(|m| l < m - EPS);
                    let long = r.value.max.is_some_and(|m| l > m + EPS);
                    let target = if short {
                        r.value.opt.or(r.value.min)
                    } else if long {
                        r.value.opt.or(r.value.max)
                    } else {
                        None
                    };
                    nets.push(NetMatch {
                        net: name(x),
                        length: l,
                        skew: None,
                        target,
                        ok: !short && !long,
                    });
                }
            }
            MatchKind::Skew => {
                // groups: the whole rule, or each pair in it
                let mut groups: Vec<Vec<NetId>> = Vec::new();
                if r.within_pairs {
                    for &x in &r.nets {
                        if let Some(y) = partner(board, x).filter(|y| r.nets.contains(y))
                            && x < y
                        {
                            groups.push(vec![x, y]);
                        }
                    }
                } else {
                    groups.push(r.nets.clone());
                }
                let max = r.value.max.unwrap_or(f64::INFINITY);
                for g in groups {
                    let longest = g.iter().map(|&x| length(x)).fold(0.0, f64::max);
                    for &x in &g {
                        let l = length(x);
                        let skew = l - longest;
                        let ok = skew.abs() <= max + EPS && r.value.min.is_none_or(|m| skew.abs() >= m - EPS);
                        nets.push(NetMatch {
                            net: name(x),
                            length: l,
                            skew: Some(skew),
                            target: (!ok).then_some(longest),
                            ok,
                        });
                    }
                }
            }
            MatchKind::Uncoupled => {
                for &x in &r.nets {
                    let Some(y) = partner(board, x) else { continue };
                    let (p, n) = board
                        .diff_pairs()
                        .into_iter()
                        .find(|&(p, n)| p == x || n == x)
                        .unwrap_or((x, y));
                    if x != p {
                        continue;
                    }
                    let (c, t) = coupling(board, geo, p, n, gap_rule(board, p));
                    let unc = (t - c).max(0.0);
                    let ok = r.value.max.is_none_or(|m| unc <= m + EPS);
                    nets.push(NetMatch {
                        net: format!("{} / {}", name(p), name(n)),
                        length: unc,
                        skew: None,
                        target: None,
                        ok,
                    });
                }
            }
            MatchKind::Gap => {}
        }
        out.push(RuleMatch {
            rule: r.name.clone(),
            kind: r.kind,
            value: r.value,
            within_pairs: r.within_pairs,
            nets,
        });
    }
    out
}

/// The tracks and vias a board file has as kept copper, as geometry (for measuring a routed
/// file).
pub fn kept_geometry(board: &Board) -> Geometry {
    use crate::board::CopperKind;
    let mut geo = Geometry::default();
    for c in &board.copper {
        let Some(net) = c.net else { continue };
        let (elems, width, layer) = match c.kind {
            CopperKind::Line { a, b, width, layer } => (vec![Elem::Line(a, b)], width, layer),
            CopperKind::Arc {
                a,
                mid,
                b,
                width,
                layer,
            } => (vec![Elem::Arc(a, mid, b)], width, layer),
            CopperKind::Via {
                at, diameter, drill, ..
            } => {
                geo.vias.push(crate::realize::Via {
                    at,
                    diameter,
                    drill,
                    net,
                    layer: None,
                    origin: None,
                    resting: true,
                });
                continue;
            }
        };
        geo.tracks.push(Track {
            pts: crate::realize::flatten(&elems),
            elems,
            width,
            layer,
            net,
            necks: Vec::new(),
            neck_w: width,
            origin: None,
            resting: true,
            fitted: Vec::new(),
        });
    }
    geo
}

/// Rules of the board that a geometry breaks, in short for a line of output.
pub fn summary(results: &[RuleMatch]) -> String {
    let failed: usize = results.iter().map(|r| r.failed()).sum();
    let total: usize = results.iter().map(|r| r.nets.len()).sum();
    format!(
        "{} rules, {} of {} nets or pairs out of range",
        results.len(),
        failed,
        total
    )
}
