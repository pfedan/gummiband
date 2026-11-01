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

use crate::board::Board;
use crate::geom::{Point, Rect};
use crate::index::FastMap;
use crate::realize::{Geometry, Track, Via};
use crate::route::{Goal, Router, seg_param_intersection};
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
    /// The via price it drew, for [`Bandit`].
    arm: Option<usize>,
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
    /// The connection is routed and the move is for its cost, see [`aim_at_cost`].
    improve: bool,
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
    if ideas().spot_fade {
        // a spot that resisted many moves gets fewer: the others go to vias and length
        let fade = spot_fade();
        let w: Vec<f64> = hot.iter().map(|&p| fade.weight(p)).collect();
        let most = |n: usize| w[..n].iter().copied().fold(0.0, f64::max);
        let draw = |n: usize, rng: &mut Rng| {
            let mut at = rng.unit() * w[..n].iter().sum::<f64>();
            (0..n)
                .find(|&i| {
                    at -= w[i];
                    at <= 0.0
                })
                .unwrap_or(n - 1)
        };
        if shorts > 0 && roll < 0.4 * most(shorts) {
            return (hot[draw(shorts, rng)], true);
        }
        if !hot.is_empty() && roll < 0.8 * most(hot.len()) {
            return (hot[draw(hot.len(), rng)], true);
        }
    } else {
        if shorts > 0 && roll < 0.4 {
            return (hot[(rng.next() % shorts as u64) as usize], true);
        }
        if !hot.is_empty() && roll < 0.8 {
            return (hot[(rng.next() % hot.len() as u64) as usize], true);
        }
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
            improve: false,
        }),
        arm: None,
    }
}

/// A move for `s`. The longer nothing helped, the more it takes out.
fn pick(s: &State, stale: usize, boldness: usize, bandit: Option<&Bandit>, rng: &mut Rng) -> Move {
    if !s.router.unrouted().is_empty() && rng.unit() < 0.5 {
        return aim(s, boldness, rng);
    }
    let ideas = ideas();
    if ideas.aim_cost > 0.0
        && rng.unit() < ideas.aim_cost
        && let Some(m) = aim_at_cost(s, boldness, rng)
    {
        return m;
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
    let (factor, arm) = if ideas.via_price || ideas.bandit {
        let k = match bandit.filter(|_| ideas.bandit) {
            Some(b) => b.draw(rng),
            None => (rng.next() % ARMS as u64) as usize,
        };
        (price_arms(r.cfg.via_cost)[k], Some(k))
    } else {
        ([0.5, 0.7, 1.0, 1.0, 1.4, 2.0, 4.0][(rng.next() % 7) as usize], None)
    };
    Move {
        conns: near,
        via_cost: r.cfg.via_cost * factor,
        hot: hot.then(|| (at, 0.05 + 0.25 * rng.unit())),
        aim: None,
        arm,
    }
}

/// A bold move for a side branch: a way for an open connection, while there are any, or a whole
/// area, more of it with every branch that failed.
fn bold(s: &State, level: usize, rng: &mut Rng) -> Move {
    if !s.router.unrouted().is_empty() && rng.unit() < ideas().bold_aim {
        return aim(s, 1 + level, rng);
    }
    let (at, _) = spot(s, rng);
    let r = &s.router;
    // from a handful up to a quarter of the board, whatever its size
    let quarter = (r.conns.len() / 4).max(6);
    let want = 6 + level.min(TOP_LEVEL) * (quarter - 6) / TOP_LEVEL + (rng.next() % 6) as usize;
    let distance = |ci: usize| s.places[ci].iter().map(|p| p.dist(at)).fold(f64::INFINITY, f64::min);
    let mut by_way: Vec<(f64, usize)> = (0..r.conns.len()).map(|ci| (distance(ci), ci)).collect();
    by_way.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    let mut near: Vec<usize> = by_way.iter().take(want * 3 / 2).map(|x| x.1).collect();
    shuffle(&mut near, rng);
    near.truncate(want);
    near.sort_by_key(|&ci| !r.conn_segs[ci].is_empty());
    let factor = [0.5, 0.7, 1.0, 1.4, 2.0][(rng.next() % 5) as usize];
    Move {
        conns: near,
        via_cost: r.cfg.via_cost * factor,
        hot: None,
        aim: None,
        arm: None,
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
        old_vias
            .get(&site)
            .map_or(m.verts[v as usize], |&i| s.cand.geo.vias[i].at)
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
        place_new_vias(&mut tracks, &mut vias, &r.board.planes, ideas().plane_hold);
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
fn place_new_vias(tracks: &mut [Track], vias: &mut [Via], planes: &[crate::board::Plane], hold: bool) {
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
            ways.map(|(e, way)| wrong_sides(&way, &tracks[e.0], tracks, &boxes, e.3).0)
                .sum::<usize>()
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
        // a via in the plane of its net must not leave it: a connection of the net may count as
        // made through that plane, also one that only shares the via (GB_PLANE_HOLD; GB_PLANE_LOG
        // says when it would happen)
        let r = v.diameter / 2.0;
        let home = planes.iter().find(|pl| pl.net == v.net && pl.outline.contains(v.at));
        if let Some(pl) = home {
            let floor = (r + 0.3).min(pl.outline.boundary_dist(v.at));
            let stays = pl.outline.contains(p) && pl.outline.boundary_dist(p) >= floor;
            if !stays {
                if ideas().plane_log {
                    eprintln!(
                        "plane via of net {} would leave its plane: ({:.3}, {:.3}) to ({:.3}, {:.3}){}",
                        v.net,
                        v.at.x,
                        v.at.y,
                        p.x,
                        p.y,
                        if hold { ", held" } else { "" }
                    );
                }
                if hold {
                    continue;
                }
            }
        }
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
    c.hot = spots
        .iter()
        .filter(|s| s.short)
        .chain(spots.iter().filter(|s| !s.short))
        .map(|s| s.at)
        .collect();
    c.flaw = spots.iter().map(|s| 300.0 + 700.0 * (s.missing / 0.03).min(1.0)).sum();
}

/// The cost of the open end.
fn open_cost(c: &Candidate) -> f64 {
    c.stats.cost() + c.flaw + c.stats.shorts.unwrap_or(0) as f64 * 5000.0
}

/// What has to go for `aim`: what its connection runs into, and at random what lies around its way.
/// A pad in a pocket has no way out but a via in the pad, the last resort of a run as well.
fn around_way(r: &mut Router, s: &State, aim: &Aim) -> Option<Vec<usize>> {
    let (mut hit, way) = if aim.improve {
        cheaper_way(r, s, aim)?
    } else {
        r.way_through(aim.ci, aim.crossing).or_else(|| {
            r.cfg.via_in_pad = true;
            let found = r.way_through(aim.ci, aim.crossing);
            r.cfg.via_in_pad = false;
            found
        })?
    };
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

/// Moves a lane of a generation may make, per mutant it needs.
const TRIES: usize = 4;

/// The mutant of a move of lane `l`, given whether it is still needed.
type Eval<'e, 'a> = dyn Fn(usize, &Move, &dyn Fn() -> bool) -> Option<State<'a>> + Sync + 'e;

/// The mutants of a generation, in lanes that need `want[l]` mutants each. The moves of a lane come
/// one by one from `make(l)` and run on all threads; a move that comes to nothing makes room for
/// the next one of its lane at once, until every lane has its mutants or has made `TRIES` moves per
/// mutant. The first ones in the order of the moves count, not those that are done first, so the
/// result does not depend on the threads or the timing. A move behind them that still runs learns
/// from `go_on` that it is not needed any more.
fn first_through<'a, const N: usize>(
    want: [usize; N],
    make: &mut (dyn FnMut(usize) -> Move + Send),
    eval: &Eval<'_, 'a>,
) -> [Vec<(Move, Option<State<'a>>)>; N] {
    struct Lane<'a> {
        want: usize,
        /// Per move, once it is done: the move and its mutant, None if it came to nothing.
        done: Vec<Option<(Move, Option<State<'a>>)>>,
        through: usize,
        running: usize,
    }
    impl Lane<'_> {
        fn open(&self) -> bool {
            self.through < self.want && self.done.len() < self.want * TRIES
        }
        /// Mutants still missing if all that run come through.
        fn need(&self) -> isize {
            self.want as isize - (self.through + self.running) as isize
        }
        /// Whether the move at `i` can still be one of the first that come through.
        fn needed(&self, i: usize) -> bool {
            self.done[..i]
                .iter()
                .filter(|d| matches!(d, Some((_, Some(_)))))
                .count()
                < self.want
        }
    }
    let state = std::sync::Mutex::new((
        make,
        want.map(|want| Lane {
            want,
            done: Vec::new(),
            through: 0,
            running: 0,
        }),
    ));
    rayon::broadcast(|_| {
        loop {
            let (l, i, m) = {
                let mut g = state.lock().unwrap();
                let (make, lanes) = &mut *g;
                // the lane that lacks the most; once all that is missing runs, the next moves
                // start ahead, in case one fails
                let Some(l) = (0..N)
                    .filter(|&l| lanes[l].open())
                    .max_by_key(|&l| (lanes[l].need(), N - l))
                else {
                    break;
                };
                let lane = &mut lanes[l];
                lane.done.push(None);
                lane.running += 1;
                (l, lane.done.len() - 1, make(l))
            };
            let go_on = || state.lock().unwrap().1[l].needed(i);
            let c = eval(l, &m, &go_on);
            let mut g = state.lock().unwrap();
            let lane = &mut g.1[l];
            lane.running -= 1;
            lane.through += c.is_some() as usize;
            lane.done[i] = Some((m, c));
        }
    });
    let (_, lanes) = state.into_inner().unwrap();
    lanes.map(|lane| {
        let mut through = 0;
        lane.done
            .into_iter()
            .map_while(|d| {
                (through < lane.want).then(|| {
                    let (m, c) = d.expect("every move is done");
                    through += c.is_some() as usize;
                    (m, c)
                })
            })
            .collect()
    })
}

/// [`mutant`], and in `tr` what became of it.
#[allow(clippy::too_many_arguments)]
fn mutant_traced<'a>(
    board: &'a Board,
    sub: &Substrate,
    s: &State<'a>,
    mv: &Move,
    relax_cfg: &relax::RelaxConfig,
    generation: usize,
    floor: usize,
    go_on: &dyn Fn() -> bool,
    tr: &mut Trace,
) -> Option<State<'a>> {
    if mv.conns.is_empty() && mv.aim.is_none() {
        tr.why = "empty";
        return None;
    }
    if !go_on() {
        tr.why = "late";
        return None;
    }
    let mut r = s.router.clone();
    let normal = (r.cfg.via_cost, r.cfg.hotspots.clone());
    let mut conns = mv.conns.clone();
    if let Some(aim) = &mv.aim {
        conns = vec![aim.ci];
        if aim.improve {
            // its own way and the vias it shares must not stand in the way it looks for
            with_shared_vias(&r, &mut conns, 0);
            for &ci in &conns {
                r.rip_up(ci);
            }
        }
        tr.why = "no way";
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
                tr.why = "not laid";
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
    if tr.on {
        tr.ripped = conns.len();
        tr.routed = routed as i64 - s.cand.stats.routed as i64;
        tr.vias = r.vias.len() as i64 - s.router.vias.len() as i64;
        tr.way = conns
            .iter()
            .map(|&ci| polyline(&r.conn_points(ci)) - polyline(&s.places[ci]))
            .sum();
        tr.near_spot = conns.iter().any(|&ci| {
            let near = |p: &Point| s.cand.hot.iter().any(|h| h.dist(*p) < 3.0);
            s.places[ci].iter().any(near) || r.conn_points(ci).iter().any(near)
        });
    }
    if routed < floor {
        tr.why = "fewer";
        return None;
    }
    if routed == s.cand.stats.routed && same_routes(&s.router, &r, &conns) {
        tr.why = "same";
        return None;
    }
    if conns.iter().any(|&ci| {
        let net = r.conns[ci].net;
        r.net_splits(net) > s.router.net_splits(net)
    }) {
        tr.why = "split";
        return None;
    }
    if ideas().prune && hopeless(s, &r, &conns, routed) {
        PRUNED.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        tr.why = "pruned";
        return None;
    }
    if !go_on() {
        tr.why = "late";
        return None;
    }
    tr.mark(1);
    let (fresh, segs, sites) = r.tagged_from(s.router.segs.len() as u32);
    let (mut geo, segs, sites) = warm_start(s, &r, fresh, &segs, &sites);
    let relaxed = if ideas().window > 0.0 {
        relax_window(board, sub, &mut geo, relax_cfg, go_on, ideas().window)
    } else {
        relax::relax_while(board, sub, &mut geo, relax_cfg, go_on)
    };
    tr.mark(2);
    if !relaxed || !go_on() {
        tr.why = "stopped";
        return None;
    }
    let mut cand = candidate(board, &r, geo, routed, s.cand.cfg.clone(), generation);
    judge(board, &mut cand);
    tr.mark(3);
    if !loose_ends(
        board,
        &cand.geo,
        &conns.iter().map(|&ci| r.conns[ci].net).collect::<Vec<_>>(),
    )
    .is_empty()
    {
        tr.why = "loose";
        return None;
    }
    if tr.on {
        tr.why = "done";
        tr.cost = open_cost(&cand) - open_cost(&s.cand);
        tr.flaw = cand.flaw - s.cand.flaw;
        tr.length = cand.stats.length - s.cand.stats.length;
        tr.won = better(&cand, &s.cand);
    }
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

/// `go_on` tells a mutant whether it is still needed: a generation may have all it needs before
/// the mutant is done, then it stops, also in the middle of its relaxation.
#[allow(clippy::too_many_arguments)]
fn mutant<'a>(
    board: &'a Board,
    sub: &Substrate,
    s: &State<'a>,
    mv: &Move,
    relax_cfg: &relax::RelaxConfig,
    generation: usize,
    floor: usize,
    go_on: &dyn Fn() -> bool,
) -> Option<State<'a>> {
    let Some(out) = mutant_log() else {
        return mutant_traced(
            board,
            sub,
            s,
            mv,
            relax_cfg,
            generation,
            floor,
            go_on,
            &mut Trace::default(),
        );
    };
    let mut tr = Trace {
        on: true,
        t0: Some(std::time::Instant::now()),
        ..Default::default()
    };
    let m = mutant_traced(board, sub, s, mv, relax_cfg, generation, floor, go_on, &mut tr);
    tr.mark(4);
    let kind = match &mv.aim {
        Some(a) if a.improve => "cost",
        Some(_) => "open",
        None if mv.hot.is_some() => "spot",
        None => "place",
    };
    let line = format!(
        "{{\"g\":{generation},\"kind\":\"{kind}\",\"n\":{},\"ripped\":{},\"price\":{:.3},\"arm\":{},\"why\":\"{}\",\"routed\":{},\"vias\":{},\"way\":{:.3},\"near_spot\":{},\"ms\":[{:.2},{:.2},{:.2},{:.2}],\"cost\":{:.4},\"flaw\":{:.2},\"length\":{:.3},\"won\":{},\"parent\":{:.3}}}\n",
        mv.conns.len(),
        tr.ripped,
        mv.via_cost / s.router.cfg.via_cost.max(1e-9),
        mv.arm.map_or(-1, |a| a as i64),
        tr.why,
        tr.routed,
        tr.vias,
        tr.way,
        tr.near_spot,
        tr.ms[0],
        tr.ms[1],
        tr.ms[2],
        tr.ms[3],
        tr.cost,
        tr.flaw,
        tr.length,
        tr.won,
        open_cost(&s.cand),
    );
    use std::io::Write;
    let _ = out.lock().unwrap().write_all(line.as_bytes());
    m
}

/// What became of one mutant, for `GB_MUTANT_LOG`: why it ended, what its routing changed before
/// the relaxation and what the result changed, and the time up to the end of its routing, its
/// relaxation, its judgement and its end (ms, counted from its start).
#[derive(Default)]
struct Trace {
    on: bool,
    t0: Option<std::time::Instant>,
    why: &'static str,
    ripped: usize,
    routed: i64,
    vias: i64,
    way: f64,
    near_spot: bool,
    ms: [f64; 4],
    cost: f64,
    flaw: f64,
    length: f64,
    won: bool,
}

impl Trace {
    /// Up to here phase `k` (1 routing, 2 relaxation, 3 judgement, 4 the end); the phases it
    /// did not reach take no time.
    fn mark(&mut self, k: usize) {
        if let Some(t0) = self.t0 {
            let now = t0.elapsed().as_secs_f64() * 1e3;
            for i in k - 1..4 {
                self.ms[i] = now;
            }
        }
    }
}

/// The file of `GB_MUTANT_LOG`, one line of JSON per mutant.
fn mutant_log() -> Option<&'static std::sync::Mutex<std::fs::File>> {
    static LOG: std::sync::OnceLock<Option<std::sync::Mutex<std::fs::File>>> = std::sync::OnceLock::new();
    LOG.get_or_init(|| {
        let path = std::env::var("GB_MUTANT_LOG").ok()?;
        std::fs::File::create(path).ok().map(std::sync::Mutex::new)
    })
    .as_ref()
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

/// Side branches that have to fail in a row, each bolder than the last, before the search counts
/// as converged. The last ones take a quarter of the connections of the board at a time.
pub(crate) const TOP_LEVEL: usize = 12;

/// The results that no other beats in both vias and length, at the level of the best one. The
/// cost weighs vias against length once and for all, the user may see it differently.
#[derive(Default)]
struct Front {
    items: Vec<Candidate>,
}

impl Front {
    /// Take `c` if it holds its own next to `best`. True if it sets a noticeable record.
    fn offer(&mut self, c: &Candidate, best: &Candidate) -> bool {
        if c.stats.routed < best.stats.routed || c.stats.shorts > best.stats.shorts || c.flaw > best.flaw {
            return false;
        }
        let beats = |a: &Candidate, b: &Candidate| {
            a.stats.routed >= b.stats.routed
                && a.flaw <= b.flaw
                && a.stats.vias <= b.stats.vias
                && a.stats.length <= b.stats.length
        };
        if self.items.iter().any(|o| beats(o, c)) {
            return false;
        }
        let fewest = self.items.iter().map(|o| o.stats.vias).min();
        let shortest = self.items.iter().map(|o| o.stats.length).fold(f64::INFINITY, f64::min);
        let record = fewest.is_none_or(|v| c.stats.vias < v) || c.stats.length < shortest * 0.998;
        self.items.retain(|o| !beats(c, o));
        self.items.push(c.clone());
        self.thin(8);
        record
    }

    /// Keep the ends and the most different ones.
    fn thin(&mut self, keep: usize) {
        self.items.sort_by_key(|c| c.stats.vias);
        while self.items.len() > keep.max(2) {
            let tightest = (1..self.items.len() - 1)
                .min_by_key(|&i| self.items[i + 1].stats.vias - self.items[i - 1].stats.vias)
                .unwrap();
            self.items.remove(tightest);
        }
    }

    /// What is on offer next to `best` at the end: as complete and as clean, and different.
    fn others(mut self, best: &Candidate) -> Vec<Candidate> {
        self.items.retain(|c| {
            c.stats.routed >= best.stats.routed
                && c.stats.shorts <= best.stats.shorts
                && c.flaw <= best.flaw
                && (c.stats.vias, c.stats.length.to_bits()) != (best.stats.vias, best.stats.length.to_bits())
        });
        self.thin(3);
        self.items
    }
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
    let plain: Vec<(f64, Vec<crate::geom::Rect>)> = c.geo.tracks.iter().map(|t| (t.width, t.necks.clone())).collect();
    c.geo
        .tracks
        .iter_mut()
        .for_each(|t| t.width = t.width.max(target(t.net)));
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
        && fresh
            .tracks
            .iter()
            .zip(&c.geo.tracks)
            .all(|(a, b)| a.net == b.net && a.layer == b.layer);
    if !same {
        return None;
    }
    let mut cand = candidate(
        board,
        &r,
        c.geo.clone(),
        c.stats.routed,
        c.cfg.clone(),
        c.stats.generation,
    );
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

/// Track ends of these nets that sit on nothing of their net, and vias with no track.
fn loose_ends(board: &Board, geo: &Geometry, nets: &[u32]) -> Vec<String> {
    let mut out = Vec::new();
    for (ti, t) in geo.tracks.iter().enumerate().filter(|(_, t)| nets.contains(&t.net)) {
        for p in [t.pts[0], *t.pts.last().unwrap()] {
            let on_via = geo.vias.iter().any(|v| v.net == t.net && v.at.dist(p) < 1e-4);
            let on_pad = board.pins.iter().any(|pin| {
                pin.net == t.net
                    && pin
                        .obstacles
                        .iter()
                        .any(|&o| board.obstacles[o].shape.dist_to_point(p) < 0.01)
            });
            let on_track = geo
                .tracks
                .iter()
                .enumerate()
                .any(|(tj, o)| tj != ti && o.net == t.net && o.pts.iter().any(|q| q.dist(p) < 1e-4));
            if !on_via && !on_pad && !on_track {
                out.push(format!(
                    "track {ti} net {} ends free at {:.3} {:.3}",
                    board.nets[t.net as usize].name, p.x, p.y
                ));
            }
        }
    }
    for v in geo.vias.iter().filter(|v| nets.contains(&v.net)) {
        let ends = geo
            .tracks
            .iter()
            .filter(|t| t.net == v.net && (t.pts[0].dist(v.at) < 1e-4 || t.pts.last().unwrap().dist(v.at) < 1e-4))
            .count();
        let plane = board.planes.iter().any(|p| p.net == v.net);
        // a via in a pad needs only the track on the other side
        let on_pad = board.pins.iter().any(|pin| {
            pin.net == v.net
                && pin
                    .obstacles
                    .iter()
                    .any(|&o| board.obstacles[o].shape.dist_to_point(v.at) < 0.01)
        });
        if (ends == 0 || (ends == 1 && !plane && !on_pad)) && v.drill > 0.0 {
            out.push(format!(
                "via net {} at {:.3} {:.3} has no track",
                board.nets[v.net as usize].name, v.at.x, v.at.y
            ));
        }
    }
    out
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
) -> (Candidate, Vec<Candidate>) {
    let Some(mut best) = rebuild(board, sub, &start) else {
        eprintln!("open end: the run does not repeat, nothing to improve on");
        return (start, Vec::new());
    };
    // GB_OPEN_POP: mutants per generation, by default the size of the portfolio
    let pop = env("GB_OPEN_POP", search.population.max(1) as f64).max(1.0) as usize;
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
    let mut front = Front::default();
    front.offer(&best.cand, &best.cand);
    let mut mark = Mark::of(&best.cand);
    let mut stale = 0;
    let mut generation = first;
    // a side branch and the generations it has left; every one that fails makes the next bolder
    let branch_after = env("GB_BRANCH_AFTER", 24.0) as usize;
    let branch_life = env("GB_BRANCH_LIFE", 32.0) as usize;
    let mut side: Option<(State, usize)> = None;
    let mut level = 0;
    let mut bandit = Bandit::new();
    let ideas = *ideas();
    // benchmarks: a fixed number of generations instead of a stop from outside
    let limit = env("GB_OPEN_GENERATIONS", f64::INFINITY);
    while !search.stopped() && ((generation - first) as f64) < limit {
        generation += 1;
        if side.is_none() && stale >= branch_after {
            let floor = best.cand.stats.routed.saturating_sub(2);
            let mut moves = Rng(rng.next());
            let [starts] = first_through([pop], &mut |_| bold(&best, level, &mut moves), &|_, m, go_on| {
                mutant(board, sub, &best, m, &local, generation, floor, go_on)
            });
            side = starts
                .into_iter()
                .filter_map(|(_, s)| s)
                .min_by(|a, b| open_cost(&a.cand).total_cmp(&open_cost(&b.cand)))
                .map(|s| (s, branch_life));
            if side.is_none() {
                level += 1;
            }
        }
        // half of the mutants for the side branch, if there is one; each lane its own moves
        let shared = if side.is_some() { pop / 2 } else { 0 };
        let mut lanes = [Rng(rng.next()), Rng(rng.next())];
        let [on_side, on_best] = first_through(
            [shared, pop - shared],
            &mut |lane| match (lane, &side) {
                (0, Some((s, _))) => pick(s, 0, level, Some(&bandit), &mut lanes[0]),
                _ => pick(&best, stale, stale / 16, Some(&bandit), &mut lanes[1]),
            },
            &|lane, m, go_on| {
                let (from, floor) = match (lane, &side) {
                    (0, Some((s, _))) => (s, s.cand.stats.routed),
                    _ => (&best, best.cand.stats.routed),
                };
                mutant(board, sub, from, m, &local, generation, floor, go_on)
            },
        );
        let next: Vec<(bool, Move, Option<State>)> = on_side
            .into_iter()
            .map(|(m, c)| (true, m, c))
            .chain(on_best.into_iter().map(|(m, c)| (false, m, c)))
            .collect();
        let tried = next.iter().filter(|n| n.2.is_some()).count();
        let aimed: Vec<_> = next.iter().filter(|n| n.1.aim.is_some()).collect();
        let laid = aimed.iter().filter(|n| n.2.is_some()).count();
        let more = aimed.iter().filter(|n| {
            let parent = if n.0 { side.as_ref().map(|s| &s.0) } else { None }.unwrap_or(&best);
            n.2.as_ref()
                .is_some_and(|c| c.cand.stats.routed > parent.cand.stats.routed)
        });
        let (aimed, more) = (aimed.len(), more.count());
        if ideas.bandit || ideas.aim_cost > 0.0 || ideas.prune || ideas.merge || ideas.window > 0.0 || ideas.spot_fade {
            learn(
                &ideas,
                &mut bandit,
                &next,
                side.as_ref().map(|s| &s.0),
                &best,
                generation,
                log,
            );
        }
        let mut wins = 0;
        // one more connection, at the price of shorts for now
        let mut promising: Option<State> = None;
        let mut record = false;
        let mut winners: Vec<(Move, State)> = Vec::new();
        for (on_side, m, c) in next {
            let Some(c) = c else { continue };
            record |= front.offer(&c.cand, &best.cand);
            if on_side {
                if let Some(side) = side.as_mut().filter(|s| better(&c.cand, &s.0.cand)) {
                    if Mark::of(&side.0.cand).beaten_by(&Mark::of(&c.cand)) {
                        side.1 = branch_life;
                    }
                    side.0 = c;
                }
            } else if ideas.merge && better(&c.cand, &best.cand) {
                winners.push((m, c));
                wins += 1;
            } else if better(&c.cand, &best.cand) {
                best = c;
                wins += 1;
            } else if c.cand.stats.routed > best.cand.stats.routed
                && promising
                    .as_ref()
                    .is_none_or(|p| open_cost(&c.cand) < open_cost(&p.cand))
            {
                promising = Some(c);
            }
        }
        if !winners.is_empty() {
            best = merge_wins(board, sub, best, winners, &local, generation, log);
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
                relax_all(
                    board,
                    sub,
                    &mut c,
                    &relax::RelaxConfig {
                        tension,
                        ..full.clone()
                    },
                );
                if better(&c, &best.cand) {
                    best.cand = c;
                    wins += 1;
                }
            }
        }
        // what the board still lacks resets the boldness, a via less only holds it: vias and length
        // can go on falling by a little for hours after the search has found its level
        let now = Mark::of(&best.cand);
        if mark.beaten_by(&now) {
            if now.routed > mark.routed || now.flaws < mark.flaws {
                level = 0;
            }
            mark = now;
            stale = 0;
            on_better(&best.cand, generation);
        } else if record {
            stale = 0;
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
                top: TOP_LEVEL,
                converged: level >= TOP_LEVEL,
            },
        );
    }
    let others = front.others(&best.cand);
    (best.cand, others)
}

// ------------------------------------------------------------------------------- experiments
//
// Ideas under test, each switched on by its own environment variable and off by default: a run
// without them is the same byte for byte. Combination A of the night series of 2026-11-01 (the
// bandit over the via price, aimed moves at a quarter, pruning; see docs/MILESTONES.md) and the
// plane hold, a fix, are on by default; `GB_BANDIT=0`, `GB_AIM_COST=0`, `GB_PRUNE=0` and
// `GB_PLANE_HOLD=0` switch them off.

/// The experiments switched on for this run.
#[derive(Clone, Copy, Debug)]
struct Ideas {
    /// `GB_VIA_PRICE`: the via price of a move reaches up to what a via is worth in the cost of
    /// the result (40 mm of track), not only four times the normal price of the router.
    via_price: bool,
    /// `GB_BANDIT`: the via price of a move is drawn by what has won on this board so far
    /// (Thompson sampling over the prices of `via_price`). On by default; `GB_BANDIT=0` switches
    /// it off.
    bandit: bool,
    /// `GB_AIM_COST=share`: that share of the moves aims at an expensive routed connection, see
    /// [`aim_at_cost`]. A quarter by default; `GB_AIM_COST=0` switches it off.
    aim_cost: f64,
    /// `GB_PRUNE`: a mutant that cannot beat its parent is dropped before its relaxation, see
    /// [`hopeless`]. On by default; `GB_PRUNE=0` switches it off.
    prune: bool,
    /// `GB_MERGE`: the mutants of a generation that beat their parent in places apart from each
    /// other all count, not only the best of them, see [`merge_wins`].
    merge: bool,
    /// `GB_WINDOW=mm`: a mutant relaxes only the copper that comes this near to its new copper,
    /// see [`relax_window`] (0: the whole board).
    window: f64,
    /// `GB_SPOT_FADE`: a clearance spot that many moves failed to clear gets fewer moves, see
    /// [`SpotFade`].
    spot_fade: bool,
    /// `GB_BOLD_AIM=share`: that share of the bold moves that start a side branch aims at an open
    /// connection while there is one; the rest take a whole area (default 0.8).
    bold_aim: f64,
    /// A new via that reaches the plane of its net stays in it when it is moved past a sleeping
    /// track (see `place_new_vias`). On by default; `GB_PLANE_HOLD=0` switches it off.
    plane_hold: bool,
    /// `GB_PLANE_LOG`: say on stderr when such a via would leave its plane.
    plane_log: bool,
}

fn ideas() -> &'static Ideas {
    static IDEAS: std::sync::OnceLock<Ideas> = std::sync::OnceLock::new();
    IDEAS.get_or_init(|| Ideas {
        via_price: std::env::var("GB_VIA_PRICE").is_ok(),
        bandit: std::env::var("GB_BANDIT").map_or(true, |v| v != "0"),
        aim_cost: env("GB_AIM_COST", 0.25),
        prune: std::env::var("GB_PRUNE").map_or(true, |v| v != "0"),
        merge: std::env::var("GB_MERGE").is_ok(),
        window: env("GB_WINDOW", 0.0),
        spot_fade: std::env::var("GB_SPOT_FADE").is_ok(),
        bold_aim: env("GB_BOLD_AIM", 0.8),
        plane_hold: std::env::var("GB_PLANE_HOLD").map_or(true, |v| v != "0"),
        plane_log: std::env::var("GB_PLANE_LOG").is_ok(),
    })
}

/// Mutants dropped by [`hopeless`], for the log.
static PRUNED: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

/// The moves at a clearance spot that came to nothing, by cells of half a millimetre, with a
/// memory that fades. Four of five moves go to a spot while there is one; a spot that no move
/// can clear (a wide track through a gap a few micrometres too narrow) would take them all for
/// hours. Picks and outcomes run in the order of the moves, so the run stays the same run.
#[derive(Default)]
struct SpotFade {
    fails: FastMap<(i64, i64), f64>,
}

impl SpotFade {
    const CELL: f64 = 0.5;
    /// Failures that halve the share of a spot.
    const HALF: f64 = 8.0;

    fn key(p: Point) -> (i64, i64) {
        ((p.x / Self::CELL).floor() as i64, (p.y / Self::CELL).floor() as i64)
    }

    /// The share of moves a spot at `p` keeps, 1 for a fresh one.
    fn weight(&self, p: Point) -> f64 {
        let (x, y) = Self::key(p);
        let mut f = 0.0;
        for dx in -1..=1 {
            for dy in -1..=1 {
                f += self.fails.get(&(x + dx, y + dy)).copied().unwrap_or(0.0);
            }
        }
        1.0 / (1.0 + f / Self::HALF)
    }

    fn learn(&mut self, p: Point, won: bool) {
        let k = Self::key(p);
        if won {
            let (x, y) = k;
            for dx in -1..=1 {
                for dy in -1..=1 {
                    self.fails.remove(&(x + dx, y + dy));
                }
            }
        } else {
            *self.fails.entry(k).or_default() += 1.0;
        }
    }

    fn fade(&mut self, by: f64) {
        self.fails.retain(|_, f| {
            *f *= by;
            *f > 0.05
        });
    }
}

fn spot_fade() -> std::sync::MutexGuard<'static, SpotFade> {
    static FADE: std::sync::OnceLock<std::sync::Mutex<SpotFade>> = std::sync::OnceLock::new();
    FADE.get_or_init(Default::default).lock().unwrap()
}

/// What a via is worth in mm of track in the cost of a result (`RunStats::cost`: 2 per via,
/// 0.05 per mm). The router lays a connection at 3 mm per via, which helps it through dense
/// spots, but a mutant is judged at 40.
const VIA_WORTH: f64 = 2.0 / 0.05;

/// Number of via prices a move can draw.
const ARMS: usize = 8;

/// The via prices of a move as factors of the normal via cost: the usual ones and on up to what
/// a via is worth.
fn price_arms(normal: f64) -> [f64; ARMS] {
    let top = (VIA_WORTH / normal.max(1e-9)).max(4.0);
    [0.5, 0.7, 1.0, 1.4, 2.0, 4.0, (4.0 * top).sqrt(), top]
}

/// Which via price wins on this board: per price the moves made and how many of them beat their
/// parent, with a memory that fades, so that the choice follows the search from phase to phase.
struct Bandit {
    tries: [f64; ARMS],
    wins: [f64; ARMS],
}

impl Bandit {
    fn new() -> Bandit {
        Bandit {
            tries: [0.0; ARMS],
            wins: [0.0; ARMS],
        }
    }

    /// A price by Thompson sampling: each one draws from what it did so far, the best draw wins.
    /// The prior is about one win in ten moves.
    fn draw(&self, rng: &mut Rng) -> usize {
        let mut best = (0, f64::NEG_INFINITY);
        for k in 0..ARMS {
            let x = beta(0.5 + self.wins[k], 4.5 + (self.tries[k] - self.wins[k]).max(0.0), rng);
            if x > best.1 {
                best = (k, x);
            }
        }
        best.0
    }

    fn learn(&mut self, arm: usize, won: bool) {
        self.tries[arm] += 1.0;
        if won {
            self.wins[arm] += 1.0;
        }
    }

    fn fade(&mut self, keep: f64) {
        self.tries
            .iter_mut()
            .chain(self.wins.iter_mut())
            .for_each(|x| *x *= keep);
    }
}

fn normal_draw(rng: &mut Rng) -> f64 {
    let u = rng.unit().max(1e-300);
    (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * rng.unit()).cos()
}

/// Gamma(a, 1) after Marsaglia and Tsang.
fn gamma_draw(a: f64, rng: &mut Rng) -> f64 {
    if a < 1.0 {
        let u = rng.unit().max(1e-300);
        return gamma_draw(a + 1.0, rng) * u.powf(1.0 / a);
    }
    let d = a - 1.0 / 3.0;
    let c = 1.0 / (9.0 * d).sqrt();
    loop {
        let x = normal_draw(rng);
        let v = (1.0 + c * x).powi(3);
        if v <= 0.0 {
            continue;
        }
        let u = rng.unit().max(1e-300);
        if u.ln() < 0.5 * x * x + d * (1.0 - v + v.ln()) {
            return d * v;
        }
    }
}

fn beta(a: f64, b: f64, rng: &mut Rng) -> f64 {
    let x = gamma_draw(a, rng);
    let y = gamma_draw(b, rng);
    x / (x + y).max(1e-300)
}

fn polyline(p: &[Point]) -> f64 {
    p.windows(2).map(|w| w[0].dist(w[1])).sum()
}

/// A move for an expensive routed connection of `s`, chosen by what it costs over its straight
/// way: its vias at what they are worth and its detour. It looks for its way again as if a via
/// cost that much (or, without vias, at about the normal price), through what is in the way at a
/// price; if that way is cheaper than its own, what it runs into goes, the connection is laid
/// first and the others after it, like an open connection. None if nothing costs extra.
fn aim_at_cost(s: &State, bold: usize, rng: &mut Rng) -> Option<Move> {
    let r = &s.router;
    let excess: Vec<f64> = (0..r.conns.len())
        .map(|ci| {
            if r.conn_segs[ci].is_empty() {
                return 0.0;
            }
            let p = &s.places[ci];
            let direct = match &r.conns[ci].goal {
                Goal::Pins(q) => q
                    .iter()
                    .map(|&q| r.board.pins[q].pos.dist(p[0]))
                    .fold(f64::INFINITY, f64::min),
                Goal::Plane => 0.0,
            };
            // through the middle of the edges even a straight track looks a little longer
            let detour = (polyline(p) - 1.25 * direct).max(0.0);
            r.conn_vias[ci].len() as f64 * VIA_WORTH + detour
        })
        .collect();
    let total: f64 = excess.iter().sum();
    if total <= 0.0 {
        return None;
    }
    let mut at = rng.unit() * total;
    let mut ci = 0;
    for (c, &x) in excess.iter().enumerate() {
        if x > 0.0 {
            ci = c;
            at -= x;
            if at <= 0.0 {
                break;
            }
        }
    }
    let arms = price_arms(r.cfg.via_cost);
    let own = if r.conn_vias[ci].is_empty() {
        arms[2 + (rng.next() % 3) as usize]
    } else {
        arms[5 + (rng.next() % 3) as usize]
    } * r.cfg.via_cost;
    let boldness = bold as f64;
    Some(Move {
        conns: Vec::new(),
        via_cost: r.cfg.via_cost * [0.7, 1.0, 1.0, 1.4, 2.0][(rng.next() % 5) as usize],
        hot: None,
        aim: Some(Aim {
            ci,
            crossing: own * [0.08, 0.15, 0.3][(rng.next() % 3) as usize] / (1.0 + 0.25 * boldness),
            via_cost: own,
            reach: 0.3 + rng.unit() * (0.7 + 0.5 * boldness).min(5.0),
            extra: (rng.next() % (2 + 2 * bold as u64).min(24)) as usize,
            chain: 1 + bold.min(5),
            seed: rng.next(),
            improve: true,
        }),
        arm: None,
    })
}

/// The way of `aim` (ripped up in `r`) at its own via price, and what it runs into, if it is
/// cheaper than the way the connection had in `s`: fewer vias, or as many and shorter.
fn cheaper_way(r: &mut Router, s: &State, aim: &Aim) -> Option<(Vec<usize>, Vec<Point>)> {
    let normal = r.cfg.via_cost;
    r.cfg.via_cost = aim.via_cost;
    let found = r.way_through_vias(aim.ci, aim.crossing);
    r.cfg.via_cost = normal;
    let (hit, way, vias) = found?;
    let (old_vias, old_len) = (s.router.conn_vias[aim.ci].len(), polyline(&s.places[aim.ci]));
    let cheaper = vias < old_vias || (vias == old_vias && polyline(&way) < old_len - 0.5);
    cheaper.then_some((hit, way))
}

/// A mutant that cannot beat its parent: no more connections, more vias, and nothing it moved
/// comes near a spot of the parent, so it cannot win by a spot it removes. A via is worth 40 mm
/// of track; what the moved connections may win in length is guessed from the edges they cross,
/// with room to spare.
fn hopeless(s: &State, r: &Router, conns: &[usize], routed: usize) -> bool {
    if routed > s.cand.stats.routed {
        return false;
    }
    let more = r.vias.len() as f64 - s.router.vias.len() as f64;
    if more <= 0.0 {
        return false;
    }
    let near_spot = |p: &Point| s.cand.hot.iter().any(|h| h.dist(*p) < 3.0);
    if conns
        .iter()
        .any(|&ci| s.places[ci].iter().any(near_spot) || r.conn_points(ci).iter().any(near_spot))
    {
        return false;
    }
    let gain: f64 = conns
        .iter()
        .map(|&ci| polyline(&s.places[ci]) - polyline(&r.conn_points(ci)))
        .sum();
    more * VIA_WORTH > gain.max(0.0) * 1.5 + 10.0
}

/// Relax `geo` as [`relax::relax_while`] does, but only with the copper that comes within
/// `margin` of what is awake: the rest sleeps anyway, and a relaxation that sees the whole board
/// pays for all of it in every round, however small the move. What it cannot see it cannot wake;
/// if the awake copper comes too close to it, the check of the whole result says so.
fn relax_window(
    board: &Board,
    sub: &Substrate,
    geo: &mut Geometry,
    cfg: &relax::RelaxConfig,
    go_on: &dyn Fn() -> bool,
    margin: f64,
) -> bool {
    use std::sync::atomic::Ordering::Relaxed;
    // the cells of side `margin` that awake copper passes, with their neighbours: what has a point
    // in one of them comes within two cells of it, what comes within one cell has one for sure
    let cell = margin.max(0.5);
    let key = |p: Point| ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64);
    let steps = |a: Point, b: Point| ((b - a).len() / (cell * 0.5)).ceil().max(1.0) as usize;
    let mut near: std::collections::HashSet<(i64, i64)> = std::collections::HashSet::new();
    let mut mark = |p: Point| {
        let (x, y) = key(p);
        for dx in -1..=1 {
            for dy in -1..=1 {
                near.insert((x + dx, y + dy));
            }
        }
    };
    for t in geo.tracks.iter().filter(|t| !t.resting) {
        t.pts.iter().for_each(|&p| mark(p));
        for w in t.pts.windows(2) {
            let n = steps(w[0], w[1]);
            (1..n).for_each(|k| mark(w[0] + (w[1] - w[0]) * (k as f64 / n as f64)));
        }
    }
    for v in geo.vias.iter().filter(|v| !v.resting) {
        mark(v.at);
    }
    WINDOW[0].fetch_add(1, Relaxed);
    WINDOW[1].fetch_add(geo.tracks.len(), Relaxed);
    if near.is_empty() {
        WINDOW[2].fetch_add(geo.tracks.len(), Relaxed);
        return relax::relax_while(board, sub, geo, cfg, go_on);
    }
    let comes_near = |pts: &[Point]| {
        pts.iter().any(|&p| near.contains(&key(p)))
            || pts.windows(2).any(|w| {
                let n = steps(w[0], w[1]);
                (1..n).any(|k| near.contains(&key(w[0] + (w[1] - w[0]) * (k as f64 / n as f64))))
            })
    };
    let keep_t: Vec<usize> = (0..geo.tracks.len())
        .filter(|&i| !geo.tracks[i].resting || comes_near(&geo.tracks[i].pts))
        .collect();
    let keep_v: Vec<usize> = (0..geo.vias.len())
        .filter(|&i| !geo.vias[i].resting || near.contains(&key(geo.vias[i].at)))
        .collect();
    // most of the board in the window: nothing to win
    if keep_t.len() * 10 > geo.tracks.len() * 8 {
        WINDOW[2].fetch_add(geo.tracks.len(), Relaxed);
        return relax::relax_while(board, sub, geo, cfg, go_on);
    }
    WINDOW[2].fetch_add(keep_t.len(), Relaxed);
    let mut part = Geometry {
        tracks: keep_t.iter().map(|&i| geo.tracks[i].clone()).collect(),
        vias: keep_v.iter().map(|&i| geo.vias[i].clone()).collect(),
    };
    let ok = relax::relax_while(board, sub, &mut part, cfg, go_on);
    for (t, &i) in part.tracks.into_iter().zip(&keep_t) {
        geo.tracks[i] = t;
    }
    for (v, &i) in part.vias.into_iter().zip(&keep_v) {
        geo.vias[i] = v;
    }
    ok
}

/// `GB_TAPER`: where a track still comes too close to something at the end and the rules of its
/// net class allow a narrower track (a minimum width below the class width), the pieces there
/// get as much narrower as the clearance needs and 2 µm more, never below the minimum. A spot
/// that misses a few micrometres (a wide power track through a gap that is just too narrow)
/// is a matter of the width, not of the way: the search would try for hours to find another.
pub(crate) fn taper_spots(board: &Board, geo: &Geometry) -> Option<Geometry> {
    let before = check::check(board, geo);
    if before.is_empty() {
        return None;
    }
    let mut g = geo.clone();
    for _ in 0..3 {
        let viol = check::check(board, &g);
        let mut changed = false;
        for v in &viol {
            let need = v.required - v.actual + 0.002;
            // a few micrometres to some ten: a taper, not a neck
            if need <= 0.0 || need > 0.05 {
                continue;
            }
            // the nearest piece of a net of the spot whose class may get narrower by that much
            let mut near: Option<(f64, usize, f64)> = None;
            for net in [Some(v.nets.0), v.nets.1].into_iter().flatten() {
                let Some(min) = board.class_of(net).min_width else {
                    continue;
                };
                for (ti, t) in g.tracks.iter().enumerate().filter(|(_, t)| t.net == net) {
                    for w in t.pts.windows(2) {
                        let width = t.piece_width(w[0], w[1]);
                        let d = crate::geom::point_seg_dist(v.at, w[0], w[1]) - width / 2.0;
                        if d < v.required + 0.01 && width - 2.0 * need >= min - 1e-9 && near.is_none_or(|n| d < n.0) {
                            near = Some((d, ti, width - 2.0 * need));
                        }
                    }
                }
            }
            if let Some((_, ti, w)) = near {
                let mut zone = Rect::empty();
                zone.add(v.at);
                g.tracks[ti].fitted.push((zone.grow(0.2), w));
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    let after = check::check(board, &g);
    (after.len() < before.len()).then(|| {
        eprintln!("taper: {} violations instead of {}", after.len(), before.len());
        g
    })
}

/// For the log: relaxations through [`relax_window`], the tracks they had and those they relaxed.
static WINDOW: [std::sync::atomic::AtomicUsize; 3] = [const { std::sync::atomic::AtomicUsize::new(0) }; 3];

/// The route of connection `ci` in `r`: its vias and the edges of its segments.
fn route_of(r: &Router, ci: usize) -> (Vec<u32>, Vec<(u8, Vec<u32>)>) {
    let path = r.conn_segs[ci]
        .iter()
        .map(|&s| {
            let s = r.segs[s as usize].as_ref().unwrap();
            (s.li, s.crossings.iter().map(|c| c.e).collect())
        })
        .collect();
    (r.conn_vias[ci].clone(), path)
}

/// Most moves a generation replays on top of its best mutant.
const MERGES: usize = 4;

/// All mutants of a generation start from the same parent, and each one that beats the best so
/// far replaces it: of four that win in four corners of the board only the best one stays, the
/// other three improvements are lost. Here the best one stays, and the move of every other
/// winner that changed other connections, away from what changed so far, is made once more on
/// top of it; the result counts if it is better still.
fn merge_wins<'a>(
    board: &'a Board,
    sub: &Substrate,
    parent: State<'a>,
    mut winners: Vec<(Move, State<'a>)>,
    cfg: &relax::RelaxConfig,
    generation: usize,
    log: bool,
) -> State<'a> {
    winners.sort_by(|a, b| open_cost(&a.1.cand).total_cmp(&open_cost(&b.1.cand)));
    let n = parent.router.conns.len();
    let changed = |w: &State| -> Vec<usize> {
        (0..n)
            .filter(|&ci| route_of(&parent.router, ci) != route_of(&w.router, ci))
            .collect()
    };
    let count = winners.len();
    let mut it = winners.into_iter();
    let (_, mut best) = it.next().expect("a winner");
    let mut applied = changed(&best);
    let mut zone: Vec<Point> = applied.iter().flat_map(|&ci| best.places[ci].iter().copied()).collect();
    let mut merged = 0;
    for (m, w) in it.take(MERGES) {
        let ch = changed(&w);
        if ch.is_empty() || ch.iter().any(|c| applied.contains(c)) {
            continue;
        }
        let pts: Vec<Point> = ch.iter().flat_map(|&ci| w.places[ci].iter().copied()).collect();
        if pts.iter().any(|p| zone.iter().any(|q| q.dist(*p) < 2.0)) {
            continue;
        }
        let Some(c) = mutant(board, sub, &best, &m, cfg, generation, best.cand.stats.routed, &|| true) else {
            continue;
        };
        if better(&c.cand, &best.cand) {
            applied.extend(&ch);
            zone.extend(pts);
            best = c;
            merged += 1;
        }
    }
    if log && count > 1 {
        eprintln!("merge {generation}: {count} winners, {merged} merged");
    }
    best
}

/// What the mutants of a generation tell: the bandit learns which via prices beat their parent,
/// and the log says how the experiments did.
fn learn(
    ideas: &Ideas,
    bandit: &mut Bandit,
    next: &[(bool, Move, Option<State>)],
    side: Option<&State>,
    best: &State,
    generation: usize,
    log: bool,
) {
    let parent = |on_side: bool| if on_side { side.unwrap_or(best) } else { best };
    let won = |on_side: bool, c: &Option<State>| c.as_ref().is_some_and(|c| better(&c.cand, &parent(on_side).cand));
    if ideas.spot_fade {
        let mut fade = spot_fade();
        fade.fade(0.99);
        for (on_side, m, c) in next {
            if let (Some((at, _)), None) = (m.hot, &m.aim) {
                fade.learn(at, won(*on_side, c));
            }
        }
    }
    if ideas.bandit {
        bandit.fade(0.98);
        for (on_side, m, c) in next {
            if let Some(arm) = m.arm {
                bandit.learn(arm, won(*on_side, c));
            }
        }
    }
    if log {
        let costly: Vec<_> = next
            .iter()
            .filter(|n| n.1.aim.as_ref().is_some_and(|a| a.improve))
            .collect();
        let laid = costly.iter().filter(|n| n.2.is_some()).count();
        let wins = costly.iter().filter(|n| won(n.0, &n.2)).count();
        let pruned = PRUNED.swap(0, std::sync::atomic::Ordering::Relaxed);
        let mut line = format!(
            "ideas {generation}: cost aims {} laid {laid} won {wins}, pruned {pruned}",
            costly.len()
        );
        if ideas.window > 0.0 && generation.is_multiple_of(24) {
            let w: Vec<usize> = WINDOW
                .iter()
                .map(|c| c.swap(0, std::sync::atomic::Ordering::Relaxed))
                .collect();
            line += &format!(
                ", window {} relaxations, {:.0} % of the tracks",
                w[0],
                100.0 * w[2] as f64 / w[1].max(1) as f64
            );
        }
        if ideas.bandit && generation.is_multiple_of(24) {
            let share: Vec<String> = (0..ARMS)
                .map(|k| format!("{:.0}/{:.0}", bandit.wins[k], bandit.tries[k]))
                .collect();
            line += &format!(", bandit {}", share.join(" "));
        }
        eprintln!("{line}");
    }
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

    fn all_nets(b: &Board) -> Vec<u32> {
        (0..b.nets.len() as u32).collect()
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
    fn the_45_degree_step_keeps_ends_and_clearances() {
        let b = twisted_board();
        let mut res = run_to(&b, 0);
        assert_eq!(res.stats.routed, res.stats.conns, "{:?}", res.stats.unrouted);
        let before = check::check(&b, &res.geometry).len();
        let ends: Vec<_> = res
            .geometry
            .tracks
            .iter()
            .map(|t| (t.pts[0], *t.pts.last().unwrap()))
            .collect();
        let mut again = res.geometry.clone();
        let st = crate::octo::octilinear(&b, &mut res.geometry);
        crate::octo::octilinear(&b, &mut again);
        assert!(check::check(&b, &res.geometry).len() <= before);
        assert!(loose_ends(&b, &res.geometry, &all_nets(&b)).is_empty());
        for (t, (a, z)) in res.geometry.tracks.iter().zip(ends) {
            assert!(t.pts[0].dist(a) < 1e-9 && t.pts.last().unwrap().dist(z) < 1e-9);
            assert!(t.elems.iter().all(|e| matches!(e, crate::realize::Elem::Line(..))));
        }
        // the crossing bundle has room: everything lies on the grid
        assert!(st.any_angle_length < 0.05 * st.length_after, "{st:?}");
        assert!(st.length_after >= st.length_before * 0.99, "{st:?}");
        // the same input gives the same output
        let pts = |g: &Geometry| g.tracks.iter().map(|t| t.pts.clone()).collect::<Vec<_>>();
        assert_eq!(pts(&res.geometry), pts(&again));
    }

    #[test]
    fn the_open_end_improves_and_repeats() {
        let b = twisted_board();
        let start = run_to(&b, 0);
        let res = run_to(&b, 60);
        assert_eq!(res.stats.routed, res.stats.conns, "{:?}", res.stats.unrouted);
        let cost = |r: &crate::RouteResult| r.stats.cost() + check::check(&b, &r.geometry).len() as f64 * 1000.0;
        assert!(cost(&res) <= cost(&start), "{} > {}", cost(&res), cost(&start));
        assert!(loose_ends(&b, &res.geometry, &all_nets(&b)).is_empty());
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
            kind: ObstacleKind::Keepout {
                vias: true,
                tracks: true,
                mask: false,
            },
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
        assert!(
            way.iter().any(|p| p.y > 14.6 && p.y < 15.2 && p.x > 15.0 && p.x < 25.0),
            "{way:?}"
        );
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
        let m = mutant(&b, &sub, &s, &mv, &relax_cfg, 1, s.cand.stats.routed, &|| true).expect("a way for B");
        assert_eq!(m.cand.stats.routed, 2);
        assert!(m.router.unrouted().is_empty());
        assert!(m.router.validate().is_empty(), "{:?}", m.router.validate());
        assert!(better(&m.cand, &s.cand));
        // the victim went around the tunnel
        let a = (0..2).find(|&ci| m.router.conns[ci].net == 0).unwrap();
        assert!(m.places[a].iter().any(|p| p.y > 16.0), "{:?}", m.places[a]);
        assert!(loose_ends(&b, &m.cand.geo, &all_nets(&b)).is_empty());
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
                kind: ObstacleKind::Keepout {
                    vias: false,
                    tracks: true,
                    mask: false,
                },
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
        let m = mutant(&b, &sub, &s, &mv, &relax_cfg, 1, 0, &|| true).expect("a way through the pad");
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
        place_new_vias(&mut tracks, &mut vias, &[], true);
        assert!(vias[0].at.y > 1.0 && vias[0].at.y < 5.0, "{:?}", vias[0].at);
        assert_eq!(tracks[1].pts[0], vias[0].at);
        // a via on the right side stays where it is
        let at = vias[0].at;
        place_new_vias(&mut tracks, &mut vias, &[], true);
        assert_eq!(vias[0].at, at);
        // the same via in the plane of its net, whose border runs just past the sleeping track:
        // the way out leaves the plane, so with the hold it stays
        let plane = crate::board::Plane {
            net: 2,
            layer: 1,
            outline: crate::geom::Polygon::new(vec![pt(-5.0, -5.0), pt(15.0, -5.0), pt(15.0, 1.2), pt(-5.0, 1.2)]),
        };
        let mut tracks = vec![
            track(1, vec![pt(0.0, 1.0), pt(10.0, 1.0)], true),
            track(2, vec![site, pt(5.0, 5.0)], false),
        ];
        vias[0].at = site;
        place_new_vias(&mut tracks, &mut vias, std::slice::from_ref(&plane), true);
        assert_eq!(vias[0].at, site);
        place_new_vias(&mut tracks, &mut vias, std::slice::from_ref(&plane), false);
        assert!(vias[0].at.y > 1.2, "{:?}", vias[0].at);
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
            let ws: Vec<f64> = t
                .elems
                .iter()
                .map(|e| match *e {
                    Elem::Line(a, b) | Elem::Arc(a, _, b) => t.piece_width(a, b),
                })
                .collect();
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
        let spots = check::spots(&[
            v(0.0, 1, 0.19),
            v(0.5, 1, 0.15),
            v(1.0, 1, 0.18),
            v(9.0, 1, 0.19),
            v(0.2, 2, 0.0),
        ]);
        assert_eq!(spots.len(), 3);
        assert!((spots[0].missing - 0.05).abs() < 1e-9 && spots[0].at.x == 0.5);
        assert!(spots[2].short && !spots[0].short);
    }

    #[test]
    fn the_front_keeps_what_differs() {
        let cand = |vias: usize, length: f64| Candidate {
            stats: crate::RunStats {
                vias,
                length,
                routed: 5,
                conns: 5,
                ..Default::default()
            },
            geo: Geometry {
                tracks: Vec::new(),
                vias: Vec::new(),
            },
            order: Vec::new(),
            failed: Vec::new(),
            bad_nets: Vec::new(),
            hot: Vec::new(),
            flaw: 0.0,
            cfg: RouterConfig::default(),
        };
        let best = cand(10, 100.0);
        let mut f = Front::default();
        assert!(f.offer(&best, &best));
        assert!(f.offer(&cand(8, 110.0), &best), "fewer vias");
        assert!(!f.offer(&cand(9, 111.0), &best), "beaten in both");
        assert!(f.offer(&cand(12, 90.0), &best), "shorter");
        let others = f.others(&best);
        let vias: Vec<usize> = others.iter().map(|c| c.stats.vias).collect();
        assert_eq!(vias, vec![8, 12]);
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
