//! The open end of a search: a finished result keeps improving until the user takes it. Moving a
//! connection in the routing order would route the whole board again, so a mutant here works on
//! the result itself: it takes a few connections around one spot out, lays them again against
//! everything else and relaxes what that touches, the rest sleeps (see [`relax::relax`]). What
//! is better replaces the result; now and then the whole board gets tightened. An open connection
//! gets moves of its own: what really stands in its way goes, it is laid first, the others after
//! it. When nothing helps any more, side branches start from the result with a bold change that
//! may make it worse and repair it on their own for a while. A mutant with one more connection
//! that brings too many shorts for now starts such a branch as well. At the end the copper gets
//! some air.

use rayon::prelude::*;

use crate::board::Board;
use crate::geom::{Point, Rect};
use crate::index::FastMap;
use crate::realize::{Geometry, Track, Via};
use crate::route::{Router, seg_param_intersection};
use crate::topo::Substrate;
use crate::{Candidate, Genome, Rng, Search, SearchConfig, candidate, check, relax};

/// The result so far: its topology and its relaxed geometry.
#[derive(Clone)]
struct State<'a> {
    router: Router<'a>,
    cand: Candidate,
    /// Per track of the geometry its segment, per via its site.
    segs: Vec<u32>,
    sites: Vec<u32>,
    /// Per connection where it runs.
    places: Vec<Vec<Point>>,
}

impl State<'_> {
    fn locate(&mut self) {
        let r = &self.router;
        self.places = (0..r.conns.len()).map(|ci| r.conn_points(ci)).collect();
    }
}

/// One mutation: these connections, in this order, at these prices.
struct Move {
    conns: Vec<usize>,
    via_cost: f64,
    hot: Option<(Point, f64)>,
    aim: Option<Aim>,
}

/// A move that gets an open connection through: what lies on its way goes, the connection is
/// laid first, then the others.
struct Aim {
    ci: usize,
    /// The price of running into something on the way, in mm of track.
    crossing: f64,
    /// The via cost of the connection itself.
    via_cost: f64,
    /// What lies at most this far from the way goes too, up to `extra` connections.
    reach: f64,
    extra: usize,
    /// How many connections that lost their way may in turn clear theirs.
    chain: usize,
    /// For the choice of the others.
    seed: u64,
}

fn env(key: &str, default: f64) -> f64 {
    std::env::var(key).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

fn shuffle<T>(v: &mut [T], rng: &mut Rng) {
    for i in (1..v.len()).rev() {
        v.swap(i, (rng.next() % (i as u64 + 1)) as usize);
    }
}

/// The spot of a mutant: a clearance problem or an expensive connection.
fn spot(s: &State, rng: &mut Rng) -> (Point, bool) {
    let r = &s.router;
    let hot = &s.cand.hot;
    let shorts = s.cand.stats.shorts.unwrap_or(0).min(hot.len());
    let roll = rng.unit();
    if shorts > 0 && roll < 0.4 {
        return (hot[(rng.next() % shorts as u64) as usize], true);
    }
    if !hot.is_empty() && roll < 0.8 {
        return (hot[(rng.next() % hot.len() as u64) as usize], true);
    }
    // a connection with vias or a long way counts for more
    let weight = |ci: usize| 1.0 + r.conn_vias[ci].len() as f64 * 8.0 + s.places[ci].len() as f64 * 0.1;
    let total: f64 = (0..r.conns.len()).map(weight).sum();
    let mut at = rng.unit() * total;
    let mut ci = 0;
    for c in 0..r.conns.len() {
        at -= weight(c);
        ci = c;
        if at <= 0.0 {
            break;
        }
    }
    let p = &s.places[ci];
    (p[(rng.next() % p.len() as u64) as usize], false)
}

/// A move for an open connection of `s`. The bolder, the more goes around its way, the less
/// running into something costs and the longer the chain of connections that clear their way.
fn aim(s: &State, bold: usize, rng: &mut Rng) -> Move {
    let r = &s.router;
    let open = r.unrouted();
    let ci = open[(rng.next() % open.len() as u64) as usize];
    let boldness = bold as f64;
    let factor = [0.5, 0.7, 1.0, 1.0, 1.4][(rng.next() % 5) as usize];
    Move {
        conns: Vec::new(),
        via_cost: r.cfg.via_cost * factor,
        hot: None,
        aim: Some(Aim {
            ci,
            crossing: [0.5, 1.5, 4.0][(rng.next() % 3) as usize] / (1.0 + 0.25 * boldness),
            via_cost: r.cfg.via_cost * [0.1, 0.25, 0.5, 1.0][(rng.next() % 4) as usize],
            reach: 0.3 + rng.unit() * (0.7 + 0.5 * boldness).min(5.0),
            extra: (rng.next() % (2 + 2 * bold as u64).min(24)) as usize,
            chain: 1 + bold.min(5),
            seed: rng.next(),
        }),
    }
}

/// A move for `s`. The longer nothing helped, the more it takes out.
fn pick(s: &State, stale: usize, boldness: usize, rng: &mut Rng) -> Move {
    if !s.router.unrouted().is_empty() && rng.unit() < 0.5 {
        return aim(s, boldness, rng);
    }
    let (at, hot) = spot(s, rng);
    let r = &s.router;
    let reach = 0.4 + rng.unit() * (1.0 + stale as f64 * 0.05).min(5.0);
    let most = 2 + (stale / 8).min(10) as u64;
    let want = 1 + (rng.next() % most) as usize;
    let mut near: Vec<usize> = (0..r.conns.len())
        .filter(|&ci| s.places[ci].iter().any(|p| p.dist(at) < reach))
        .collect();
    shuffle(&mut near, rng);
    near.truncate(want);
    // what is open goes first
    near.sort_by_key(|&ci| !r.conn_segs[ci].is_empty());
    let factor = [0.5, 0.7, 1.0, 1.0, 1.4, 2.0, 4.0][(rng.next() % 7) as usize];
    Move {
        conns: near,
        via_cost: r.cfg.via_cost * factor,
        hot: hot.then(|| (at, 0.05 + 0.25 * rng.unit())),
        aim: None,
    }
}

/// A bold move for a side branch: a way for an open connection, while there are any, or a whole
/// area, more of it with every branch that failed.
fn bold(s: &State, level: usize, rng: &mut Rng) -> Move {
    if !s.router.unrouted().is_empty() && rng.unit() < 0.8 {
        return aim(s, 1 + level, rng);
    }
    let (at, _) = spot(s, rng);
    let r = &s.router;
    let reach = (2.0 + level as f64 + rng.unit() * 2.0).min(15.0);
    let want = (6 + 4 * level + (rng.next() % 6) as usize).min(r.conns.len());
    let mut near: Vec<usize> = (0..r.conns.len())
        .filter(|&ci| s.places[ci].iter().any(|p| p.dist(at) < reach))
        .collect();
    shuffle(&mut near, rng);
    near.truncate(want);
    near.sort_by_key(|&ci| !r.conn_segs[ci].is_empty());
    let factor = [0.5, 0.7, 1.0, 1.4, 2.0][(rng.next() % 5) as usize];
    Move {
        conns: near,
        via_cost: r.cfg.via_cost * factor,
        hot: None,
        aim: None,
    }
}

/// Add to `conns` from `from` on what shares a via with them: a connection can end on the via of
/// another one, and that one must not walk away.
fn with_shared_vias(r: &Router, conns: &mut Vec<usize>, from: usize) {
    let mut i = from;
    while i < conns.len() {
        for &v in &r.conn_vias[conns[i]] {
            for cj in 0..r.conns.len() {
                if r.conn_vias[cj].contains(&v) && !conns.contains(&cj) {
                    conns.push(cj);
                }
            }
        }
        i += 1;
    }
}

/// Where segment `sid` of `s` crosses the line from `a` to `b`, as a fraction of it, or where it
/// comes closest to it.
fn place_on(s: &State, sid: u32, a: Point, b: Point) -> Option<f64> {
    let first = s.segs.partition_point(|&x| x < sid);
    let tracks = s.cand.geo.tracks[first..].iter().zip(&s.segs[first..]);
    let tracks = || tracks.clone().take_while(|t| *t.1 == sid).map(|t| t.0);
    for t in tracks() {
        for w in t.pts.windows(2) {
            if let Some((x, _)) = seg_param_intersection(a, b, w[0], w[1]) {
                return Some(x);
            }
        }
    }
    let line = b - a;
    let near = tracks().flat_map(|t| t.pts.iter()).map(|&p| {
        let x = ((p - a).dot(line) / line.len2().max(1e-12)).clamp(0.0, 1.0);
        (p.dist(a + line * x), x)
    });
    near.min_by(|p, q| p.0.total_cmp(&q.0)).map(|p| p.1)
}

/// `geo` of a mutant of `s`: what is left of `s` as it lies, asleep, and the new tracks and vias
/// awake. A new track starts between its neighbours where those really are, with its ends on the
/// vias where those are now: a track that starts on the wrong side of one of them stays there,
/// and that is a short.
fn warm_start(s: &State, r: &Router, fresh: Geometry, segs: &[u32], sites: &[u32]) -> (Geometry, Vec<u32>, Vec<u32>) {
    let old_vias: FastMap<u32, usize> = s.sites.iter().enumerate().map(|(i, &v)| (v, i)).collect();
    let new_seg = |sid: u32| sid as usize >= s.router.segs.len();
    let vertex = |li: usize, v: u32| {
        let m = &r.sub.m[li];
        let site = m.vert_site[v as usize];
        old_vias.get(&site).map_or(m.verts[v as usize], |&i| s.cand.geo.vias[i].at)
    };
    let place = |sid: u32, a: Point, b: Point| {
        if new_seg(sid) {
            return None;
        }
        place_on(s, sid, a, b)
    };
    let thread = !r.has_kept && !r.soft_on;
    let mut tracks = Vec::with_capacity(fresh.tracks.len());
    let mut tags = Vec::with_capacity(fresh.tracks.len());
    for (t, &sid) in s.cand.geo.tracks.iter().zip(&s.segs) {
        if r.segs[sid as usize].is_some() {
            let mut t = t.clone();
            t.resting = true;
            tracks.push(t);
            tags.push(sid);
        }
    }
    let mut moved: FastMap<(i64, i64), Point> = FastMap::default();
    let grid = |p: Point| ((p.x * 1e6).round() as i64, (p.y * 1e6).round() as i64);
    let mut vias = Vec::with_capacity(fresh.vias.len());
    for (v, &site) in fresh.vias.iter().zip(sites) {
        // a site can have a via of another net now
        match old_vias.get(&site).filter(|&&i| s.cand.geo.vias[i].net == v.net) {
            Some(&i) => {
                let mut o = s.cand.geo.vias[i].clone();
                moved.insert(grid(v.at), o.at);
                o.resting = true;
                vias.push(o);
            }
            None => vias.push(v.clone()),
        }
    }
    for (mut t, &sid) in fresh.tracks.into_iter().zip(segs) {
        if !new_seg(sid) {
            continue;
        }
        if thread {
            t.pts = r.threaded(sid, &vertex, &place);
        }
        let n = t.pts.len();
        for i in [0, n - 1] {
            if let Some(&p) = moved.get(&grid(t.pts[i])) {
                t.pts[i] = p;
            }
        }
        t.elems.clear();
        tracks.push(t);
        tags.push(sid);
    }
    if thread {
        place_new_vias(&mut tracks, &mut vias);
    }
    (crate::realize::Geometry { tracks, vias }, tags, sites.to_vec())
}

/// The sleeping tracks that `way` of the track `t` crosses an odd number of times (with `whole`
/// false only those in its first half): its start lies on their wrong side. Their number, and
/// how far along the way the last of them is.
fn wrong_sides(way: &[Point], t: &Track, tracks: &[Track], boxes: &[Rect], whole: bool) -> (usize, f64) {
    let mut bbox = Rect::empty();
    way.iter().for_each(|&p| bbox.add(p));
    let lens: Vec<f64> = way.windows(2).map(|w| w[0].dist(w[1])).collect();
    let total: f64 = lens.iter().sum();
    let (mut count, mut past) = (0, 0.0_f64);
    for (o, b) in tracks.iter().zip(boxes) {
        if !o.resting || o.layer != t.layer || o.net == t.net || !b.overlaps(&bbox) {
            continue;
        }
        let (mut hits, mut last, mut run) = (0, 0.0, 0.0);
        for (w, len) in way.windows(2).zip(&lens) {
            for q in o.pts.windows(2) {
                if let Some((x, _)) = seg_param_intersection(w[0], w[1], q[0], q[1]) {
                    hits += 1;
                    last = run + x * len;
                }
            }
            run += len;
        }
        if hits % 2 == 1 && (whole || last < total * 0.5) {
            count += 1;
            past = past.max(last);
        }
    }
    (count, past)
}

/// The point at arc length `at` of `way`.
fn along(way: &[Point], at: f64) -> Point {
    let mut left = at;
    for w in way.windows(2) {
        let len = w[0].dist(w[1]);
        if left <= len {
            return w[0] + (w[1] - w[0]) * (left / len.max(1e-9));
        }
        left -= len;
    }
    *way.last().unwrap()
}

/// `way` from `p` on: what lies before it goes if it is on the way.
fn from_point(way: &[Point], p: Point) -> Vec<Point> {
    let cut = (0..way.len() - 1).find(|&i| crate::geom::point_seg_dist(p, way[i], way[i + 1]) < 1e-9);
    let mut out = vec![p];
    out.extend_from_slice(&way[cut.map_or(1, |i| i + 1)..]);
    out
}

/// The sleeping copper has moved since the mesh was made, so the site of a new via can lie on the
/// wrong side of a track that runs past it. Its track then crosses that track once, and no push
/// gets it back. The way of the track is right, so the via goes along it, past the crossing, if
/// that leaves fewer such crossings on all its tracks.
fn place_new_vias(tracks: &mut [Track], vias: &mut [Via]) {
    let boxes: Vec<Rect> = tracks
        .iter()
        .map(|t| {
            let mut b = Rect::empty();
            t.pts.iter().for_each(|&p| b.add(p));
            b
        })
        .collect();
    let fresh: Vec<Point> = vias.iter().filter(|v| !v.resting).map(|v| v.at).collect();
    for v in vias.iter_mut().filter(|v| !v.resting) {
        // the tracks that end here, each as a way from here, and is its far end fixed?
        let mut ends: Vec<(usize, bool, Vec<Point>, bool)> = Vec::new();
        for (ti, t) in tracks.iter().enumerate().filter(|(_, t)| !t.resting && t.net == v.net) {
            for last in [false, true] {
                let mut way = t.pts.clone();
                if last {
                    way.reverse();
                }
                if way[0].dist(v.at) < 1e-6 {
                    let far = *way.last().unwrap();
                    ends.push((ti, last, way, !fresh.iter().any(|f| f.dist(far) < 1e-6)));
                }
            }
        }
        let wrong = |p: Option<Point>| {
            let ways = ends.iter().map(|e| (e, p.map_or(e.2.clone(), |p| from_point(&e.2, p))));
            ways.map(|(e, way)| wrong_sides(&way, &tracks[e.0], tracks, &boxes, e.3).0).sum::<usize>()
        };
        let mut least = wrong(None);
        let mut to = None;
        for e in &ends {
            let (count, past) = wrong_sides(&e.2, &tracks[e.0], tracks, &boxes, e.3);
            if count == 0 || least == 0 {
                continue;
            }
            let p = along(&e.2, past + v.diameter / 2.0 + 0.2);
            let left = wrong(Some(p));
            if left < least {
                (least, to) = (left, Some(p));
            }
        }
        let Some(p) = to else { continue };
        for (ti, last, way, _) in &ends {
            let mut way = from_point(way, p);
            if *last {
                way.reverse();
            }
            tracks[*ti].pts = way;
        }
        v.at = p;
    }
}

/// Same connections, same way: nothing to look at.
fn same_routes(a: &Router, b: &Router, conns: &[usize]) -> bool {
    let path = |r: &Router, ci: usize| -> Vec<(u8, Vec<u32>)> {
        r.conn_segs[ci]
            .iter()
            .map(|&s| {
                let s = r.segs[s as usize].as_ref().unwrap();
                (s.li, s.crossings.iter().map(|c| c.e).collect())
            })
            .collect()
    };
    conns
        .iter()
        .all(|&ci| a.conn_vias[ci] == b.conn_vias[ci] && path(a, ci) == path(b, ci))
}

/// Judge the geometry of a candidate. Here a violation is a spot and not a piece of track, and
/// a spot weighs by how much is missing there, so that a search does not chase the number of
/// pieces in a curve, and a spot that gets better counts before it is gone.
fn judge(board: &Board, c: &mut Candidate) {
    let v = check::check(board, &c.geo);
    let spots = check::spots(&v);
    let mut nets: Vec<_> = v.iter().flat_map(|x| [Some(x.nets.0), x.nets.1]).flatten().collect();
    nets.sort_unstable();
    nets.dedup();
    c.stats.violations = Some(spots.len());
    c.stats.shorts = Some(spots.iter().filter(|s| s.short).count());
    c.bad_nets = nets;
    // the shorts first
    c.hot = spots.iter().filter(|s| s.short).chain(spots.iter().filter(|s| !s.short)).map(|s| s.at).collect();
    c.flaw = spots.iter().map(|s| 300.0 + 700.0 * (s.missing / 0.03).min(1.0)).sum();
}

/// The cost of the open end.
fn open_cost(c: &Candidate) -> f64 {
    c.stats.cost() + c.flaw + c.stats.shorts.unwrap_or(0) as f64 * 5000.0
}

/// What has to go for `aim`: what its connection runs into, and at random what lies around its way.
/// A pad in a pocket has no way out but a via in the pad, the last resort of a run as well.
fn around_way(r: &mut Router, s: &State, aim: &Aim) -> Option<Vec<usize>> {
    let (mut hit, way) = r.way_through(aim.ci, aim.crossing).or_else(|| {
        r.cfg.via_in_pad = true;
        let found = r.way_through(aim.ci, aim.crossing);
        r.cfg.via_in_pad = false;
        found
    })?;
    let mut near: Vec<usize> = (0..r.conns.len())
        .filter(|&cj| cj != aim.ci && !hit.contains(&cj))
        .filter(|&cj| s.places[cj].iter().any(|p| way.iter().any(|q| q.dist(*p) < aim.reach)))
        .collect();
    shuffle(&mut near, &mut Rng(aim.seed));
    near.truncate(aim.extra);
    hit.extend(near);
    Some(hit)
}

/// The open connection itself, at its own prices.
fn lay_open(r: &mut Router, aim: &Aim) -> bool {
    let via_cost = r.cfg.via_cost;
    r.cfg.via_cost = aim.via_cost;
    let mut ok = r.route_one(aim.ci);
    if !ok {
        r.cfg.via_in_pad = true;
        ok = r.route_one(aim.ci);
        r.cfg.via_in_pad = false;
    }
    r.cfg.via_cost = via_cost;
    ok
}

/// A connection that lost its way clears its own: what stands in it goes and comes back after it,
/// as long as `aim` allows and never at the expense of the connections that already cleared theirs.
fn clear_ways(r: &mut Router, aim: &Aim, conns: &mut Vec<usize>, lost: &mut Vec<usize>) {
    let mut kept = vec![aim.ci];
    for _ in 0..aim.chain {
        let Some(ci) = lost.pop() else { return };
        let Some((mut hit, _)) = r.way_through(ci, aim.crossing) else {
            lost.insert(0, ci);
            continue;
        };
        hit.retain(|cj| !r.conn_segs[*cj].is_empty());
        let mut todo = vec![ci];
        todo.extend(hit);
        with_shared_vias(r, &mut todo, 1);
        if todo.iter().any(|cj| kept.contains(cj)) {
            lost.insert(0, ci);
            continue;
        }
        for &cj in &todo {
            r.rip_up(cj);
            if !conns.contains(&cj) {
                conns.push(cj);
            }
        }
        if r.route_one(ci) {
            kept.push(ci);
        } else {
            lost.insert(0, ci);
        }
        lost.extend(todo[1..].iter().copied().filter(|&cj| !r.route_one(cj)));
    }
}

fn mutant<'a>(
    board: &'a Board,
    sub: &Substrate,
    s: &State<'a>,
    mv: &Move,
    relax_cfg: &relax::RelaxConfig,
    generation: usize,
    floor: usize,
) -> Option<State<'a>> {
    if mv.conns.is_empty() && mv.aim.is_none() {
        return None;
    }
    let mut r = s.router.clone();
    let normal = (r.cfg.via_cost, r.cfg.hotspots.clone());
    let mut conns = mv.conns.clone();
    if let Some(aim) = &mv.aim {
        conns = vec![aim.ci];
        conns.extend(around_way(&mut r, s, aim)?);
    }
    with_shared_vias(&r, &mut conns, 0);
    for &ci in &conns {
        r.rip_up(ci);
    }
    r.cfg.via_cost = mv.via_cost;
    if let Some(h) = mv.hot {
        r.cfg.hotspots = std::sync::Arc::new(vec![h]);
    }
    let rest = match &mv.aim {
        Some(aim) => {
            if !lay_open(&mut r, aim) {
                return None;
            }
            1
        }
        None => 0,
    };
    let mut lost: Vec<usize> = conns[rest..].iter().copied().filter(|&ci| !r.route_one(ci)).collect();
    (r.cfg.via_cost, r.cfg.hotspots) = normal;
    lost.retain(|&ci| !r.route_one(ci));
    if let Some(aim) = &mv.aim {
        clear_ways(&mut r, aim, &mut conns, &mut lost);
    }
    let routed = r.conn_segs.iter().filter(|c| !c.is_empty()).count();
    if routed < floor || (routed == s.cand.stats.routed && same_routes(&s.router, &r, &conns)) {
        return None;
    }
    if conns.iter().any(|&ci| {
        let net = r.conns[ci].net;
        r.net_splits(net) > s.router.net_splits(net)
    }) {
        return None;
    }
    let (fresh, segs, sites) = r.tagged_from(s.router.segs.len() as u32);
    let (mut geo, segs, sites) = warm_start(s, &r, fresh, &segs, &sites);
    relax::relax(board, sub, &mut geo, relax_cfg);
    let mut cand = candidate(board, &r, geo, routed, s.cand.cfg.clone(), generation);
    judge(board, &mut cand);
    let mut next = State {
        router: r,
        cand,
        segs,
        sites,
        places: Vec::new(),
    };
    next.locate();
    Some(next)
}

/// A connection is never given up. One more counts only if it does not bring two shorts: a
/// connection that shorts other nets is not worth more than an open one.
fn better(a: &Candidate, b: &Candidate) -> bool {
    a.stats.routed >= b.stats.routed && open_cost(a) < open_cost(b)
}

/// A pass that pushes may cost length, but no clearance.
fn better_or_same(a: &Candidate, b: &Candidate) -> bool {
    a.stats.routed >= b.stats.routed && a.flaw <= b.flaw && a.stats.shorts <= b.stats.shorts
}

/// The level of a result, to tell a noticeable gain from a small one.
#[derive(Clone, Copy)]
struct Mark {
    routed: usize,
    flaws: usize,
    flaw: f64,
    cost: f64,
}

impl Mark {
    fn of(c: &Candidate) -> Mark {
        Mark {
            routed: c.stats.routed,
            flaws: c.stats.violations.unwrap_or(0) + c.stats.shorts.unwrap_or(0),
            flaw: c.flaw,
            cost: c.stats.cost(),
        }
    }

    /// Fewer flaws or more connections count at once, vias and length from 0.2 % on: small
    /// gains go on for long after the search has found its level.
    fn beaten_by(&self, n: &Mark) -> bool {
        n.routed > self.routed
            || (n.routed == self.routed && (n.flaws < self.flaws || n.flaw < self.flaw * 0.95))
            || (n.routed == self.routed && n.flaws == self.flaws && n.cost < self.cost * 0.998)
    }
}

/// Board with `air` more clearance everywhere, for measuring how tight a geometry is.
fn roomier(board: &Board, air: f64) -> Board {
    let mut b = board.clone();
    b.rules.min_clearance += air;
    b.classes.iter_mut().for_each(|c| c.clearance += air);
    b.obstacles.iter_mut().for_each(|o| o.clearance += air);
    b.pair_clearance.values_mut().for_each(|c| *c += air);
    b
}

fn measure_length(c: &mut Candidate) {
    c.stats.length = c
        .geo
        .tracks
        .iter()
        .map(|t| t.pts.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>())
        .sum();
}

/// The whole board once more, awake, with this tension and reserve.
fn relax_all(board: &Board, sub: &Substrate, c: &mut Candidate, cfg: &relax::RelaxConfig) {
    c.geo.tracks.iter_mut().for_each(|t| t.resting = false);
    c.geo.vias.iter_mut().for_each(|v| v.resting = false);
    relax::relax(board, sub, &mut c.geo, cfg);
    measure_length(c);
    judge(board, c);
}

/// Where a pass that pushes had no room, the nets in trouble go on with the normal reserve, the
/// rest stays as it is.
fn settle(board: &Board, sub: &Substrate, c: &mut Candidate, best: &Candidate, cfg: &relax::RelaxConfig) {
    let cfg = relax::RelaxConfig {
        iterations: env("GB_AIR_IT", 120.0) as usize,
        margin: relax::RelaxConfig::default().margin,
        ..cfg.clone()
    };
    for _ in 0..4 {
        if c.flaw <= best.flaw {
            break;
        }
        let bad = c.bad_nets.clone();
        c.geo.tracks.iter_mut().for_each(|t| t.resting = !bad.contains(&t.net));
        c.geo.vias.iter_mut().for_each(|v| v.resting = !bad.contains(&v.net));
        relax::relax(board, sub, &mut c.geo, &cfg);
        measure_length(c);
        judge(board, c);
    }
}

/// Work on the spots themselves. At fine pitch most of them are tracks that take their full width
/// back before the bundle has spread: there they stay at their neck width for a while longer, where
/// the rules allow it. Only the nets there move, and what they touch wakes up.
fn clean_spots(board: &Board, sub: &Substrate, best: &Candidate, log: bool) -> Option<Candidate> {
    if best.flaw == 0.0 {
        return None;
    }
    let cfg = relax::RelaxConfig {
        iterations: env("GB_AIR_IT", 120.0) as usize,
        tension: 0.0,
        via_edge: true,
        ..Default::default()
    };
    let mut c = best.clone();
    for _ in 0..3 {
        for s in check::spots(&check::check(board, &c.geo)) {
            let mut zone = crate::geom::Rect::empty();
            zone.add(s.at);
            let zone = zone.grow(1.0);
            let nets = [Some(s.nets.0), s.nets.1];
            for t in c.geo.tracks.iter_mut().filter(|t| {
                nets.contains(&Some(t.net)) && t.neck_w < t.width && board.class_of(t.net).min_width.is_none()
            }) {
                t.necks.push(zone);
            }
        }
        let bad = c.bad_nets.clone();
        c.geo.tracks.iter_mut().for_each(|t| t.resting = !bad.contains(&t.net));
        c.geo.vias.iter_mut().for_each(|v| v.resting = !bad.contains(&v.net));
        relax::relax(board, sub, &mut c.geo, &cfg);
        measure_length(&mut c);
        judge(board, &mut c);
        if log {
            eprintln!(
                "clean: {} spots flaw {:.0}, from {} spots flaw {:.0}",
                c.stats.violations.unwrap_or(0),
                c.flaw,
                best.stats.violations.unwrap_or(0),
                best.flaw
            );
        }
        if c.flaw == 0.0 {
            break;
        }
    }
    better(&c, best).then_some(c)
}

/// Make room for wider tracks: the tracks push as if they were as wide as `target` of their net
/// (0: the net does not grow), and with
/// them everything they touch, in chains as far as that goes. No tension, so nothing pulls back.
/// A spot that has no room for it stays at the old width, the rest of the track does not. Measured
/// at the old widths nothing may get worse; the wide pieces themselves come later.
pub(crate) fn make_room(
    board: &Board,
    sub: &Substrate,
    best: &Candidate,
    target: impl Fn(u32) -> f64,
    reserve: f64,
) -> Option<Candidate> {
    let mut before = best.clone();
    judge(board, &mut before);
    let mut c = before.clone();
    let plain: Vec<(f64, Vec<crate::geom::Rect>)> =
        c.geo.tracks.iter().map(|t| (t.width, t.necks.clone())).collect();
    c.geo.tracks.iter_mut().for_each(|t| t.width = t.width.max(target(t.net)));
    let base = relax::RelaxConfig::default();
    let cfg = relax::RelaxConfig {
        iterations: env("GB_AIR_IT", 120.0) as usize,
        tension: 0.0,
        margin: base.margin + reserve,
        via_edge: true,
        taut: false,
        ..base.clone()
    };
    for _ in 0..env("GB_ROOM_ROUNDS", 6.0) as usize {
        relax_all(board, sub, &mut c, &cfg);
        let left = check::check(board, &c.geo);
        if left.is_empty() {
            break;
        }
        for v in &left {
            let mut zone = crate::geom::Rect::empty();
            zone.add(v.at);
            let zone = zone.grow(0.6);
            let nets = [Some(v.nets.0), v.nets.1];
            for t in c.geo.tracks.iter_mut().filter(|t| nets.contains(&Some(t.net))) {
                t.necks.push(zone);
            }
        }
    }
    // the shortcuts at the widths reached; taut keeps half the margin
    let cfg = relax::RelaxConfig {
        iterations: 0,
        margin: 2.0 * (base.margin + reserve),
        taut: true,
        ..cfg
    };
    relax_all(board, sub, &mut c, &cfg);
    for (t, (w, necks)) in c.geo.tracks.iter_mut().zip(plain) {
        t.width = w;
        t.necks = necks;
    }
    judge(board, &mut c);
    settle(board, sub, &mut c, &before, &cfg);
    if std::env::var("GB_OPEN_LOG").is_ok() {
        eprintln!(
            "room: {} violations, {} shorts, {:.0} mm",
            c.stats.violations.unwrap_or(0),
            c.stats.shorts.unwrap_or(0),
            c.stats.length
        );
    }
    better_or_same(&c, &before).then_some(c)
}

/// A rubber band lies at the limit: tension pulls it up to the clearance of whatever it passes,
/// also with free room next to it. A round of pushing with a larger reserve and no tension
/// moves the copper away from the limit where there is room, and evens out the gaps where there
/// is none. It stays if it costs nothing.
pub(crate) fn give_air(board: &Board, sub: &Substrate, best: &Candidate, air: f64) -> Option<Candidate> {
    let mut before = best.clone();
    judge(board, &mut before);
    let mut c = before.clone();
    let base = relax::RelaxConfig::default();
    let cfg = relax::RelaxConfig {
        iterations: env("GB_AIR_IT", 120.0) as usize,
        tension: 0.0,
        margin: base.margin + air,
        via_edge: true,
        ..base
    };
    relax_all(board, sub, &mut c, &cfg);
    settle(board, sub, &mut c, &before, &cfg);
    if std::env::var("GB_OPEN_LOG").is_ok() {
        let probe = roomier(board, air / 2.0);
        eprintln!(
            "air {air}: {} violations, {} shorts, {:.0} mm; spots under {:.3} mm spare {} before, {} after",
            c.stats.violations.unwrap_or(0),
            c.stats.shorts.unwrap_or(0),
            c.stats.length,
            air / 2.0,
            check::check(&probe, &best.geo).len(),
            check::check(&probe, &c.geo).len()
        );
    }
    better_or_same(&c, &before).then_some(c)
}

/// The router state of `c`, whose run is repeated, and its geometry tagged. None if the run does
/// not come out the same way.
fn rebuild<'a>(board: &'a Board, sub: &Substrate, c: &Candidate) -> Option<State<'a>> {
    let mut r = Router::new(board, sub, c.cfg.clone());
    r.plan();
    let genome: Genome = c.order.iter().enumerate().map(|(i, &k)| (k, i as f64)).collect();
    r.reorder(|x| genome.get(&(x.net, x.from)).copied().unwrap_or(f64::MAX));
    r.run();
    r.cfg.hotspots = Default::default();
    let (fresh, segs, sites) = r.tagged_geometry();
    let same = fresh.tracks.len() == c.geo.tracks.len()
        && fresh.vias.len() == c.geo.vias.len()
        && fresh.tracks.iter().zip(&c.geo.tracks).all(|(a, b)| a.net == b.net && a.layer == b.layer);
    if !same {
        return None;
    }
    let mut cand = candidate(board, &r, c.geo.clone(), c.stats.routed, c.cfg.clone(), c.stats.generation);
    judge(board, &mut cand);
    let mut s = State {
        router: r,
        cand,
        segs,
        sites,
        places: Vec::new(),
    };
    s.locate();
    Some(s)
}

/// Mutate `start` until the search is stopped and return the best result.
pub(crate) fn run(
    board: &Board,
    sub: &Substrate,
    search: &SearchConfig,
    start: Candidate,
    first: usize,
    rng: &mut Rng,
    on_better: &mut dyn FnMut(&Candidate, usize),
) -> Candidate {
    let Some(mut best) = rebuild(board, sub, &start) else {
        eprintln!("open end: the run does not repeat, nothing to improve on");
        return start;
    };
    let pop = search.population.max(1);
    let local = relax::RelaxConfig {
        iterations: env("GB_LOCAL_IT", 160.0) as usize,
        via_edge: true,
        ..Default::default()
    };
    let full = relax::RelaxConfig {
        via_edge: true,
        ..Default::default()
    };
    let tighten_every = env("GB_TIGHTEN", 24.0) as usize;
    let log = std::env::var("GB_OPEN_LOG").is_ok();
    let t0 = std::time::Instant::now();
    // every connection had its turn a few times, and side branches did not help either
    let settled = (3 * best.cand.stats.conns / pop).max(100);
    let mut mark = Mark::of(&best.cand);
    let mut stale = 0;
    let mut generation = first;
    // a side branch and the generations it has left; every one that fails makes the next bolder
    let branch_after = env("GB_BRANCH_AFTER", 24.0) as usize;
    let branch_life = env("GB_BRANCH_LIFE", 32.0) as usize;
    let mut side: Option<(State, usize)> = None;
    let mut level = 0;
    while !search.stopped() {
        generation += 1;
        if side.is_none() && stale >= branch_after {
            let moves: Vec<Move> = (0..pop).map(|_| bold(&best, level, rng)).collect();
            let floor = best.cand.stats.routed.saturating_sub(2);
            side = moves
                .par_iter()
                .map(|m| mutant(board, sub, &best, m, &local, generation, floor))
                .collect::<Vec<_>>()
                .into_iter()
                .flatten()
                .min_by(|a, b| open_cost(&a.cand).total_cmp(&open_cost(&b.cand)))
                .map(|s| (s, branch_life));
        }
        let shared = if side.is_some() { pop / 2 } else { 0 };
        let moves: Vec<(bool, Move)> = (0..pop)
            .map(|i| match &side {
                Some((s, _)) if i < shared => (true, pick(s, 0, level, rng)),
                _ => (false, pick(&best, stale, stale / 16, rng)),
            })
            .collect();
        let next: Vec<(bool, Option<State>)> = moves
            .par_iter()
            .map(|(on_side, m)| {
                let (from, floor) = match (&side, on_side) {
                    (Some((s, _)), true) => (s, s.cand.stats.routed),
                    _ => (&best, best.cand.stats.routed),
                };
                (*on_side, mutant(board, sub, from, m, &local, generation, floor))
            })
            .collect();
        let tried = next.iter().filter(|n| n.1.is_some()).count();
        let aimed: Vec<_> = moves.iter().zip(&next).filter(|(m, _)| m.1.aim.is_some()).collect();
        let laid = aimed.iter().filter(|(_, n)| n.1.is_some()).count();
        let more = aimed.iter().filter(|(m, n)| {
            let parent = if m.0 { side.as_ref().map(|s| &s.0) } else { None }.unwrap_or(&best);
            n.1.as_ref().is_some_and(|c| c.cand.stats.routed > parent.cand.stats.routed)
        });
        let (aimed, more) = (aimed.len(), more.count());
        let mut wins = 0;
        // one more connection, at the price of shorts for now
        let mut promising: Option<State> = None;
        for (on_side, c) in next {
            let Some(c) = c else { continue };
            if on_side {
                if let Some(side) = side.as_mut().filter(|s| better(&c.cand, &s.0.cand)) {
                    if Mark::of(&side.0.cand).beaten_by(&Mark::of(&c.cand)) {
                        side.1 = branch_life;
                    }
                    side.0 = c;
                }
            } else if better(&c.cand, &best.cand) {
                best = c;
                wins += 1;
            } else if c.cand.stats.routed > best.cand.stats.routed
                && promising.as_ref().is_none_or(|p| open_cost(&c.cand) < open_cost(&p.cand))
            {
                promising = Some(c);
            }
        }
        if let Some(p) = promising.filter(|p| p.cand.stats.routed > best.cand.stats.routed)
            && side.as_ref().is_none_or(|s| better(&p.cand, &s.0.cand))
        {
            side = Some((p, branch_life));
        }
        if let Some((s, left)) = side.take() {
            if better(&s.cand, &best.cand) {
                if log {
                    eprintln!("open {generation}: a side branch of level {level} wins");
                }
                best = s;
                wins += 1;
                level = 0;
            } else if left == 0 {
                level += 1;
            } else {
                side = Some((s, left - 1));
            }
        }
        // now and then the whole board, also what has been asleep all the time
        if generation % tighten_every == 0 {
            if let Some(c) = clean_spots(board, sub, &best.cand, log) {
                best.cand = c;
                wins += 1;
            }
            for tension in [full.tension, 0.0] {
                let mut c = best.cand.clone();
                relax_all(board, sub, &mut c, &relax::RelaxConfig { tension, ..full.clone() });
                if better(&c, &best.cand) {
                    best.cand = c;
                    wins += 1;
                }
            }
        }
        if mark.beaten_by(&Mark::of(&best.cand)) {
            mark = Mark::of(&best.cand);
            stale = 0;
            level = 0;
            on_better(&best.cand, generation);
        } else {
            stale += 1;
        }
        if log {
            let st = &best.cand.stats;
            eprintln!(
                "open {generation} {:.1}s tried {tried} aimed {aimed} laid {laid} more {more} wins {wins} stale {stale} level {level}{}: {}/{} {} vias {:.0} mm {} viol {} shorts flaw {:.0}",
                t0.elapsed().as_secs_f64(),
                side.as_ref().map_or(String::new(), |(s, left)| format!(
                    " side {} left {} cost {:.0} of {:.0}",
                    s.cand.stats.vias,
                    left,
                    open_cost(&s.cand),
                    open_cost(&best.cand)
                )),
                st.routed,
                st.conns,
                st.vias,
                st.length,
                st.violations.unwrap_or(0),
                st.shorts.unwrap_or(0),
                best.cand.flaw
            );
        }
        search.report_at(
            &format!("generation {generation}"),
            &best.cand.stats,
            Search {
                generation,
                stale,
                level,
                converged: stale >= settled,
            },
        );
    }
    best.cand
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Layer, LayerSet, Net, NetClass, Obstacle, ObstacleKind, Pin};
    use crate::geom::{Polygon, Shape, pt};
    use crate::realize::Elem;
    use crate::route::RouterConfig;
    use crate::topo::SubstrateConfig;
    use crate::{ProgressFn, route_board};
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};

    /// Debugging: track ends that sit on nothing of their net, and vias with no track.
    fn loose_ends(board: &Board, geo: &Geometry) -> Vec<String> {
        let mut out = Vec::new();
        for (ti, t) in geo.tracks.iter().enumerate() {
            for p in [t.pts[0], *t.pts.last().unwrap()] {
                let on_via = geo.vias.iter().any(|v| v.net == t.net && v.at.dist(p) < 1e-4);
                let on_pad = board.pins.iter().any(|pin| {
                    pin.net == t.net
                        && pin.obstacles.iter().any(|&o| board.obstacles[o].shape.dist_to_point(p) < 0.01)
                });
                let on_track = geo.tracks.iter().enumerate().any(|(tj, o)| {
                    tj != ti && o.net == t.net && o.pts.iter().any(|q| q.dist(p) < 1e-4)
                });
                if !on_via && !on_pad && !on_track {
                    out.push(format!("track {ti} net {} ends free at {:.3} {:.3}", board.nets[t.net as usize].name, p.x, p.y));
                }
            }
        }
        for v in &geo.vias {
            let ends = geo.tracks.iter().filter(|t| t.net == v.net && (t.pts[0].dist(v.at) < 1e-4 || t.pts.last().unwrap().dist(v.at) < 1e-4)).count();
            let plane = board.planes.iter().any(|p| p.net == v.net);
            if (ends == 0 || (ends == 1 && !plane)) && v.drill > 0.0 {
                out.push(format!("via net {} at {:.3} {:.3} has no track", board.nets[v.net as usize].name, v.at.x, v.at.y));
            }
        }
        out
    }

    /// Nets that all have to cross each other: the pins of one side come in reverse order.
    fn twisted_board() -> Board {
        let mut b = Board::default();
        b.rules.min_clearance = 0.1;
        b.layers = ["F.Cu", "B.Cu"]
            .iter()
            .map(|n| Layer {
                name: n.to_string(),
                is_plane: false,
            })
            .collect();
        b.outline = vec![Polygon::new(vec![pt(0., 0.), pt(30., 0.), pt(30., 20.), pt(0., 20.)])];
        b.classes = vec![NetClass {
            name: "Default".into(),
            track_width: 0.2,
            clearance: 0.1,
            via_diameter: 0.6,
            via_drill: 0.3,
            ..Default::default()
        }];
        let n = 6;
        for i in 0..n {
            let mut net = Net {
                name: format!("N{i}"),
                class: 0,
                pins: Vec::new(),
                planes: LayerSet::default(),
                priority: 0,
            };
            for (x, row) in [(4.0, i), (26.0, n - 1 - i)] {
                let (id, o) = (b.pins.len(), b.obstacles.len());
                let at = pt(x, 4.0 + 2.0 * row as f64);
                let (lo, hi) = (at + pt(-0.6, -0.3), at + pt(0.6, 0.3));
                b.obstacles.push(Obstacle {
                    shape: Shape::Polygon(Polygon::new(vec![lo, pt(hi.x, lo.y), hi, pt(lo.x, hi.y)])),
                    layers: LayerSet::single(0),
                    net: Some(i as u32),
                    kind: ObstacleKind::Pad,
                    hole: None,
                    clearance: 0.1,
                    pin: Some(id),
                    locked: false,
                });
                b.pins.push(Pin {
                    name: format!("J{x}.{row}"),
                    footprint: format!("J{x}"),
                    net: i as u32,
                    pos: at,
                    layers: LayerSet::single(0),
                    obstacles: vec![o],
                    size: (0.6, 1.2),
                });
                net.pins.push(id);
            }
            b.nets.push(net);
        }
        b
    }

    /// The pieces of a net of the twisted board: one when routed, two without the connection.
    fn rebuild_pieces(b: &Board) -> usize {
        let sub = Substrate::build(b, &SubstrateConfig::default());
        let mut r = Router::new(b, &sub, RouterConfig::default());
        r.plan();
        assert_eq!(r.net_pieces(0), 2);
        r.run();
        r.net_pieces(0)
    }

    /// Route with an open end and stop at this generation.
    fn run_to(b: &Board, stop_at: usize) -> crate::RouteResult {
        let flag = Arc::new(AtomicBool::new(false));
        let set = flag.clone();
        let search = SearchConfig {
            population: 4,
            generations: 2,
            geometry_generations: 1,
            frames: false,
            progress: Some(ProgressFn(Arc::new(move |p: &crate::Progress| {
                if p.search.generation >= stop_at {
                    set.store(true, Ordering::Relaxed);
                }
            }))),
            seed: None,
            open_end: Some(flag),
            air: 0.0,
            prefer_width: 0.0,
        };
        route_board(b, &SubstrateConfig::default(), &RouterConfig::default(), &search).1
    }

    #[test]
    fn the_open_end_improves_and_repeats() {
        let b = twisted_board();
        let start = run_to(&b, 0);
        let res = run_to(&b, 60);
        assert_eq!(res.stats.routed, res.stats.conns, "{:?}", res.stats.unrouted);
        let cost = |r: &crate::RouteResult| r.stats.cost() + check::check(&b, &r.geometry).len() as f64 * 1000.0;
        assert!(cost(&res) <= cost(&start), "{} > {}", cost(&res), cost(&start));
        assert!(loose_ends(&b, &res.geometry).is_empty(), "{:?}", loose_ends(&b, &res.geometry));
        let again_router = rebuild_pieces(&b);
        assert_eq!(again_router, 1);
        let again = run_to(&b, 60);
        assert_eq!(res.geometry.vias.len(), again.geometry.vias.len());
        let pts = |r: &crate::RouteResult| r.geometry.tracks.iter().flat_map(|t| t.pts.clone()).collect::<Vec<_>>();
        assert_eq!(pts(&res), pts(&again));
    }

    /// One layer. Net A may go through a tunnel with room for one track or around it; net B has
    /// its pins in two pockets that open into the tunnel and no other way.
    fn tunnel_board() -> Board {
        let mut b = Board::default();
        b.rules.min_clearance = 0.1;
        b.layers = vec![Layer {
            name: "F.Cu".into(),
            is_plane: false,
        }];
        b.outline = vec![Polygon::new(vec![pt(0., 0.), pt(40., 0.), pt(40., 24.), pt(0., 24.)])];
        b.classes = vec![NetClass {
            name: "Default".into(),
            track_width: 0.2,
            clearance: 0.1,
            via_diameter: 0.6,
            via_drill: 0.3,
            ..Default::default()
        }];
        let wall = |x0: f64, y0: f64, x1: f64, y1: f64| Obstacle {
            shape: Shape::Polygon(Polygon::new(vec![pt(x0, y0), pt(x1, y0), pt(x1, y1), pt(x0, y1)])),
            layers: LayerSet::single(0),
            net: None,
            kind: ObstacleKind::Keepout { vias: true, tracks: true },
            hole: None,
            clearance: 0.0,
            pin: None,
            locked: false,
        };
        // under the tunnel, with a slit into each pocket, over it, and between the pockets
        b.obstacles = vec![
            wall(-1.0, 14.0, 14.0, 14.7),
            wall(15.0, 14.0, 25.0, 14.7),
            wall(26.0, 14.0, 41.0, 14.7),
            wall(10.0, 15.1, 30.0, 16.0),
            wall(19.0, -1.0, 21.0, 14.0),
        ];
        for (net, name, pins) in [
            (0, "A", [pt(5.0, 14.9), pt(35.0, 14.9)]),
            (1, "B", [pt(14.5, 8.0), pt(25.5, 8.0)]),
        ] {
            let mut n = Net {
                name: name.into(),
                class: 0,
                pins: Vec::new(),
                planes: LayerSet::default(),
                priority: 0,
            };
            for at in pins {
                let (id, o) = (b.pins.len(), b.obstacles.len());
                let (lo, hi) = (at + pt(-0.3, -0.2), at + pt(0.3, 0.2));
                b.obstacles.push(Obstacle {
                    shape: Shape::Polygon(Polygon::new(vec![lo, pt(hi.x, lo.y), hi, pt(lo.x, hi.y)])),
                    layers: LayerSet::single(0),
                    net: Some(net),
                    kind: ObstacleKind::Pad,
                    hole: None,
                    clearance: 0.1,
                    pin: Some(id),
                    locked: false,
                });
                b.pins.push(Pin {
                    name: format!("{name}{}", id % 2),
                    footprint: name.into(),
                    net,
                    pos: at,
                    layers: LayerSet::single(0),
                    obstacles: vec![o],
                    size: (0.4, 0.6),
                });
                n.pins.push(id);
            }
            b.nets.push(n);
        }
        b
    }

    /// A state of `b`, with the connections routed in this order of their nets, without rip-up,
    /// or with nothing routed.
    fn state_of<'a>(b: &'a Board, sub: &Substrate, first: Option<u32>) -> State<'a> {
        let cfg = RouterConfig {
            ripup_passes: 0,
            ..Default::default()
        };
        let mut r = Router::new(b, sub, cfg.clone());
        r.plan();
        if let Some(first) = first {
            r.reorder(|c| if c.net == first { 0.0 } else { 1.0 });
            r.run();
        }
        let (geo, segs, sites) = r.tagged_geometry();
        let routed = r.conn_segs.iter().filter(|c| !c.is_empty()).count();
        let mut cand = candidate(b, &r, geo, routed, cfg, 0);
        judge(b, &mut cand);
        let mut s = State {
            router: r,
            cand,
            segs,
            sites,
            places: Vec::new(),
        };
        s.locate();
        s
    }

    #[test]
    fn the_way_through_names_what_is_in_the_way() {
        let b = tunnel_board();
        let sub = Substrate::build(&b, &SubstrateConfig::default());
        let s = state_of(&b, &sub, Some(0));
        let open = s.router.unrouted();
        assert_eq!(open.len(), 1, "B must find the tunnel taken");
        assert_eq!(s.router.conns[open[0]].net, 1);
        let r = &s.router;
        let (hit, way) = r.way_through(open[0], 1.5).unwrap();
        assert_eq!(hit.iter().map(|&c| r.conns[c].net).collect::<Vec<_>>(), [0]);
        assert!(way.iter().any(|p| p.y > 14.6 && p.y < 15.2 && p.x > 15.0 && p.x < 25.0), "{way:?}");
    }

    #[test]
    fn an_aimed_move_gets_the_open_connection_through() {
        let b = tunnel_board();
        let sub = Substrate::build(&b, &SubstrateConfig::default());
        let s = state_of(&b, &sub, Some(0));
        let relax_cfg = relax::RelaxConfig {
            iterations: 40,
            via_edge: true,
            ..Default::default()
        };
        let mut rng = Rng(1);
        let mv = aim(&s, 0, &mut rng);
        let m = mutant(&b, &sub, &s, &mv, &relax_cfg, 1, s.cand.stats.routed).expect("a way for B");
        assert_eq!(m.cand.stats.routed, 2);
        assert!(m.router.unrouted().is_empty());
        assert!(m.router.validate().is_empty(), "{:?}", m.router.validate());
        assert!(better(&m.cand, &s.cand));
        // the victim went around the tunnel
        let a = (0..2).find(|&ci| m.router.conns[ci].net == 0).unwrap();
        assert!(m.places[a].iter().any(|p| p.y > 16.0), "{:?}", m.places[a]);
        assert!(loose_ends(&b, &m.cand.geo).is_empty(), "{:?}", loose_ends(&b, &m.cand.geo));
    }

    #[test]
    fn a_pad_in_a_pocket_leaves_through_a_via_in_the_pad() {
        let mut b = tunnel_board();
        b.layers.push(Layer {
            name: "B.Cu".into(),
            is_plane: false,
        });
        // B alone, and a wall around its pad on the top layer
        b.nets.truncate(1);
        b.pins.truncate(2);
        b.obstacles.retain(|o| o.pin.is_some_and(|p| p < 2));
        b.obstacles.iter_mut().enumerate().for_each(|(i, o)| {
            o.pin = Some(i);
        });
        let net_pins: Vec<usize> = b.nets[0].pins.clone();
        for (i, &p) in net_pins.iter().enumerate() {
            b.pins[p].obstacles = vec![i];
            b.pins[p].size = (0.8, 0.8);
        }
        let at = b.pins[0].pos;
        for (lo, hi) in [
            (at + pt(-1.0, -1.0), at + pt(1.0, -0.45)),
            (at + pt(-1.0, 0.45), at + pt(1.0, 1.0)),
            (at + pt(-1.0, -1.0), at + pt(-0.45, 1.0)),
            (at + pt(0.45, -1.0), at + pt(1.0, 1.0)),
        ] {
            b.obstacles.push(Obstacle {
                shape: Shape::Polygon(Polygon::new(vec![lo, pt(hi.x, lo.y), hi, pt(lo.x, hi.y)])),
                layers: LayerSet::single(0),
                net: None,
                kind: ObstacleKind::Keepout { vias: false, tracks: true },
                hole: None,
                clearance: 0.0,
                pin: None,
                locked: false,
            });
        }
        let sub_cfg = SubstrateConfig {
            via_pitch: 3.0,
            ..Default::default()
        };
        let sub = Substrate::build(&b, &sub_cfg);
        let s = state_of(&b, &sub, None);
        assert!(s.router.way_through(0, 1.5).is_none());
        let relax_cfg = relax::RelaxConfig {
            iterations: 40,
            via_edge: true,
            ..Default::default()
        };
        let mv = aim(&s, 0, &mut Rng(1));
        let m = mutant(&b, &sub, &s, &mv, &relax_cfg, 1, 0).expect("a way through the pad");
        assert_eq!(m.cand.stats.routed, 1);
        assert!(m.router.vias.keys().any(|&v| m.router.site_point(v).dist(at) < 1e-6));
    }

    #[test]
    fn a_new_via_goes_past_the_track_it_lies_behind() {
        let track = |net: u32, pts: Vec<Point>, resting: bool| Track {
            pts,
            elems: Vec::new(),
            width: 0.2,
            layer: 0,
            net,
            necks: Vec::new(),
            neck_w: 0.2,
            origin: None,
            resting,
            fitted: Vec::new(),
        };
        // the site lies under a sleeping track, the way of the new track leads over it
        let site = pt(5.0, 0.5);
        let mut tracks = vec![
            track(1, vec![pt(0.0, 1.0), pt(10.0, 1.0)], true),
            track(2, vec![site, pt(5.0, 5.0)], false),
        ];
        let mut vias = vec![Via {
            at: site,
            diameter: 0.6,
            drill: 0.3,
            net: 2,
            layer: None,
            origin: None,
            resting: false,
        }];
        place_new_vias(&mut tracks, &mut vias);
        assert!(vias[0].at.y > 1.0 && vias[0].at.y < 5.0, "{:?}", vias[0].at);
        assert_eq!(tracks[1].pts[0], vias[0].at);
        // a via on the right side stays where it is
        let at = vias[0].at;
        place_new_vias(&mut tracks, &mut vias);
        assert_eq!(vias[0].at, at);
    }

    #[test]
    fn an_open_connection_is_no_split() {
        let b = twisted_board();
        let sub = Substrate::build(&b, &SubstrateConfig::default());
        let mut r = Router::new(&b, &sub, RouterConfig::default());
        r.plan();
        r.run();
        let ci = (0..r.conns.len()).find(|&ci| !r.conn_segs[ci].is_empty()).unwrap();
        let net = r.conns[ci].net;
        assert_eq!(r.net_splits(net), 1);
        r.rip_up(ci);
        assert_eq!((r.net_pieces(net), r.net_splits(net)), (2, 1));
    }

    #[test]
    fn tracks_grow_where_there_is_room_and_taper() {
        let b = twisted_board();
        let search = SearchConfig {
            population: 2,
            generations: 0,
            geometry_generations: 0,
            frames: false,
            progress: None,
            seed: None,
            open_end: None,
            air: 0.0,
            prefer_width: 0.4,
        };
        let cfg = RouterConfig::default();
        let res = route_board(&b, &SubstrateConfig::default(), &cfg, &search).1;
        assert!(check::check(&b, &res.geometry).is_empty());
        let mut widest: f64 = 0.0;
        for t in &res.geometry.tracks {
            let ws: Vec<f64> = t.elems.iter().map(|e| match *e {
                Elem::Line(a, b) | Elem::Arc(a, _, b) => t.piece_width(a, b),
            }).collect();
            assert!(ws.iter().all(|&w| w >= 0.2 - 1e-9 && w <= 0.4 + 1e-9), "{ws:?}");
            assert!(ws.windows(2).all(|w| (w[0] - w[1]).abs() <= 0.025 + 1e-9), "{ws:?}");
            widest = ws.iter().fold(widest, |a, &b| a.max(b));
        }
        assert!(widest > 0.39, "{widest}");
    }

    #[test]
    fn a_class_grows_to_its_own_target() {
        let mut b = twisted_board();
        b.classes.push(NetClass {
            name: "Power".into(),
            target_width: Some(0.35),
            ..b.classes[0].clone()
        });
        for (i, n) in b.nets.iter_mut().enumerate() {
            n.class = (i % 2) as u16;
        }
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
        let res = route_board(&b, &SubstrateConfig::default(), &RouterConfig::default(), &search).1;
        assert!(check::check(&b, &res.geometry).is_empty());
        let mut widest = [0.0f64; 2];
        for t in &res.geometry.tracks {
            let class = b.nets[t.net as usize].class as usize;
            for e in &t.elems {
                let (Elem::Line(p, q) | Elem::Arc(p, _, q)) = *e;
                widest[class] = widest[class].max(t.piece_width(p, q));
            }
        }
        // the default class stays at its width without --prefer-width, Power grows to its target
        assert!((widest[0] - 0.2).abs() < 1e-9, "{widest:?}");
        assert!(widest[1] > 0.34 && widest[1] <= 0.35 + 1e-9, "{widest:?}");
    }

    #[test]
    fn the_pieces_of_a_curve_are_one_spot() {
        let v = |x: f64, net: u32, actual: f64| check::Violation {
            what: String::new(),
            nets: (net, Some(9)),
            at: pt(x, 0.0),
            actual,
            required: 0.2,
        };
        let spots = check::spots(&[v(0.0, 1, 0.19), v(0.5, 1, 0.15), v(1.0, 1, 0.18), v(9.0, 1, 0.19), v(0.2, 2, 0.0)]);
        assert_eq!(spots.len(), 3);
        assert!((spots[0].missing - 0.05).abs() < 1e-9 && spots[0].at.x == 0.5);
        assert!(spots[2].short && !spots[0].short);
    }

    #[test]
    fn air_moves_tracks_off_the_limit() {
        let b = twisted_board();
        let (sub, res) = crate::route_fixed(
            &b,
            &SubstrateConfig::default(),
            &RouterConfig::default(),
            &SearchConfig {
                population: 2,
                generations: 0,
                geometry_generations: 0,
                frames: false,
                progress: None,
                seed: None,
                open_end: None,
                air: 0.0,
                prefer_width: 0.0,
            },
        );
        let c = Candidate {
            stats: res.stats.clone(),
            geo: res.geometry.clone(),
            order: Vec::new(),
            failed: Vec::new(),
            bad_nets: Vec::new(),
            hot: Vec::new(),
            flaw: 0.0,
            cfg: RouterConfig::default(),
        };
        let probe = roomier(&b, 0.015);
        let before = check::check(&probe, &c.geo).len();
        let after = give_air(&b, &sub, &c, 0.03).map_or(before, |c| check::check(&probe, &c.geo).len());
        assert!(after <= before, "{after} > {before}");
    }
}
