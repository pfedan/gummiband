//! Optional finishing step: turn the any-angle tracks and arcs of a result into conventional
//! 45 degree routing (`--angles 45`).
//!
//! Every track is walked from its start along its taut polyline. From where it stands, the walk
//! reaches as far ahead as it can with one or two straight legs in the eight directions of the
//! 45 degree grid; where no such pair of legs keeps all clearances, the stretch is split in the
//! middle (a small staircase) and, if that does not fit either, the original piece stays as it
//! is. Corners that turn by 90 or 135 degrees are then chamfered with 45 degree pieces where the
//! room allows. Tracks are done one after another, each against the current copper of all the
//! others, so the step never adds a clearance violation: completion and clearance come first,
//! the angles second. Vias and track ends do not move.

use crate::board::{Board, KEEPOUT_GAP, NetId, ObstacleKind};
use crate::geom::{Point, Rect, point_seg_dist, pt, seg_seg_dist};
use crate::index::GridIndex;
use crate::realize::{Elem, Geometry, Track, Via};

/// What the 45 degree step did.
#[derive(Debug, Clone, Copy, Default, serde::Serialize)]
pub struct OctoStats {
    /// Tracks looked at (routed tracks of this run; imported copper is left alone).
    pub tracks: usize,
    /// Track length before and after in mm.
    pub length_before: f64,
    pub length_after: f64,
    /// Pieces that kept an angle off the 45 degree grid because nothing else fit, and their
    /// length in mm.
    pub any_angle: usize,
    pub any_angle_length: f64,
    /// Corners sharper than 45 degrees that could not be chamfered (90 or 135 degree turns).
    pub sharp_corners: usize,
}

/// Room kept on top of the clearances, for the rounding to KiCad's nanometre grid.
const MARGIN: f64 = 0.0005;
/// A leg shorter than this is not worth a corner: the piece counts as straight.
const MIN_LEG: f64 = 0.001;

/// The eight directions of the 45 degree grid, counter-clockwise from +x.
fn dirs() -> [Point; 8] {
    let h = std::f64::consts::FRAC_1_SQRT_2;
    [
        pt(1.0, 0.0),
        pt(h, h),
        pt(0.0, 1.0),
        pt(-h, h),
        pt(-1.0, 0.0),
        pt(-h, -h),
        pt(0.0, -1.0),
        pt(h, -h),
    ]
}

/// Index of the grid direction of `v` if it lies on the grid (within `MIN_LEG` over its length).
fn grid_dir(v: Point) -> Option<usize> {
    let l = v.len();
    if l < 1e-12 {
        return None;
    }
    dirs()
        .iter()
        .position(|d| d.dot(v) > 0.0 && d.cross(v).abs() < MIN_LEG.min(l * 1e-6).max(1e-7))
}

/// The two ways to go from `a` to `b` with one leg in each of the two grid directions that
/// enclose the direction a-b: the corner points (first leg along the lower direction, first leg
/// along the upper one) and the two directions. `None` if a-b already lies on the grid (or a
/// leg would be shorter than `MIN_LEG`).
fn two_legs(a: Point, b: Point) -> Option<([Point; 2], [usize; 2])> {
    let v = b - a;
    if v.len() < 1e-9 {
        return None;
    }
    let ang = v.y.atan2(v.x).rem_euclid(std::f64::consts::TAU);
    let k = ((ang / std::f64::consts::FRAC_PI_4).floor() as usize) % 8;
    let d = dirs();
    let (d1, d2) = (d[k], d[(k + 1) % 8]);
    let den = d1.cross(d2);
    let alpha = v.cross(d2) / den;
    let beta = d1.cross(v) / den;
    if alpha < MIN_LEG || beta < MIN_LEG {
        return None;
    }
    Some(([a + d1 * alpha, a + d2 * beta], [k, (k + 1) % 8]))
}

/// Turn between two grid directions in steps of 45 degrees (0 to 4).
fn turn(a: usize, b: usize) -> usize {
    let d = (b + 8 - a) % 8;
    d.min(8 - d)
}

/// The copper of the board and of the result, for clearance tests of new pieces.
struct Env<'a> {
    board: &'a Board,
    obs: GridIndex,
    edges: Vec<(Point, Point)>,
    edge_idx: GridIndex,
    vias: Vec<(Point, f64, NetId)>,
    via_idx: GridIndex,
    /// Track pieces (a, b, width, track); a removed piece gets track = u32::MAX.
    pieces: Vec<(Point, Point, f64, u32)>,
    piece_idx: GridIndex,
    owned: Vec<Vec<u32>>,
    reach: f64,
    /// The neck zones of every track as the router left them.
    necks: Vec<Necks>,
}

fn seg_box(a: Point, b: Point) -> Rect {
    let mut r = Rect::empty();
    r.add(a);
    r.add(b);
    r
}

impl<'a> Env<'a> {
    fn new(board: &'a Board, geo: &Geometry) -> Env<'a> {
        let bounds = board.bbox().grow(5.0);
        let mut obs = GridIndex::new(bounds, 1.0);
        for (i, o) in board.obstacles.iter().enumerate() {
            obs.insert(i as u32, &o.shape.bbox());
        }
        let mut edges = Vec::new();
        let mut edge_idx = GridIndex::new(bounds, 1.0);
        for o in &board.outline {
            for (p, q) in o.edges() {
                edge_idx.insert(edges.len() as u32, &seg_box(p, q));
                edges.push((p, q));
            }
        }
        let mut vias = Vec::new();
        let mut via_idx = GridIndex::new(bounds, 1.0);
        for v in geo.vias.iter().filter(|v| v.drill > 0.0) {
            via_idx.insert(vias.len() as u32, &seg_box(v.at, v.at).grow(v.diameter / 2.0));
            vias.push((v.at, v.diameter / 2.0, v.net));
        }
        let widest_track = geo.tracks.iter().map(|t| t.width.max(t.neck_w)).fold(0.0, f64::max);
        let widest_via = vias.iter().map(|v| v.1).fold(0.0, f64::max);
        let clr = (0..board.nets.len() as NetId)
            .map(|n| board.class_of(n).clearance)
            .chain(board.pair_clearance.values().copied())
            .chain(board.obstacles.iter().map(|o| o.clearance))
            .fold(board.rules.min_clearance.max(board.rules.edge_clearance), f64::max)
            .max(board.rules.hole_clearance);
        let reach = widest_track + widest_via + clr + 0.1;
        let mut env = Env {
            board,
            obs,
            edges,
            edge_idx,
            vias,
            via_idx,
            pieces: Vec::new(),
            piece_idx: GridIndex::new(bounds, 0.5),
            owned: vec![Vec::new(); geo.tracks.len()],
            reach,
            necks: geo.tracks.iter().map(|t| (t.net, t.necks.clone())).collect(),
        };
        for (ti, t) in geo.tracks.iter().enumerate() {
            let (start, parts) = parts(board, t, &|_| false);
            env.add(ti, start, &flat(&parts));
        }
        env
    }

    fn add(&mut self, ti: usize, start: Point, path: &[(Point, f64)]) {
        let mut a = start;
        for &(b, w) in path {
            let id = self.pieces.len() as u32;
            self.pieces.push((a, b, w, ti as u32));
            self.piece_idx.insert(id, &seg_box(a, b));
            self.owned[ti].push(id);
            a = b;
        }
    }

    fn replace(&mut self, ti: usize, start: Point, path: &[(Point, f64)]) {
        for &id in &self.owned[ti] {
            self.pieces[id as usize].3 = u32::MAX;
        }
        self.owned[ti].clear();
        self.add(ti, start, path);
    }

    fn clr(&self, n: NetId) -> f64 {
        self.board.class_of(n).clearance.max(self.board.rules.min_clearance)
    }

    /// Whether the piece a-b of track `ti` with width `w` keeps every clearance (the rules of
    /// `check::check`, with a small margin).
    fn clear(&self, geo: &Geometry, ti: usize, a: Point, b: Point, w: f64) -> bool {
        self.blocker(geo, ti, a, b, w).is_none()
    }

    /// What keeps the piece a-b from fitting, if anything (for `GB_OCTO_DEBUG`).
    fn blocker(&self, geo: &Geometry, ti: usize, a: Point, b: Point, w: f64) -> Option<String> {
        let board = self.board;
        let t = &geo.tracks[ti];
        let (net, layer) = (t.net, t.layer);
        let half = w / 2.0;
        let r = seg_box(a, b);
        let rr = r.grow(half + self.reach);
        let routing = board.routing_layers();
        for oi in self.obs.query(&rr) {
            let o = &board.obstacles[oi as usize];
            if o.net == Some(net) && o.kind != ObstacleKind::Hole {
                continue;
            }
            if !(o.layers.has(layer) && routing.has(layer)) {
                continue;
            }
            let need = match o.kind {
                ObstacleKind::Keepout { tracks, .. } => {
                    if !tracks {
                        continue;
                    }
                    KEEPOUT_GAP
                }
                ObstacleKind::Hole => board.rules.hole_clearance,
                _ => o.clearance.max(self.clr(net)),
            };
            if o.shape.dist_to_seg(a, b) - half < need + MARGIN {
                return Some(format!(
                    "{:?} of net {:?}",
                    o.kind,
                    o.net.map(|n| &board.nets[n as usize].name)
                ));
            }
        }
        let need = board.rules.edge_clearance + half + MARGIN;
        for ei in self.edge_idx.query(&r.grow(need + 1e-9)) {
            let (p, q) = self.edges[ei as usize];
            if seg_seg_dist(a, b, p, q) < need {
                return Some("edge".into());
            }
        }
        for vi in self.via_idx.query(&rr) {
            let (c, vr, vn) = self.vias[vi as usize];
            if vn == net {
                continue;
            }
            let need = vr + half + self.clr(net).max(self.clr(vn)) + MARGIN;
            if point_seg_dist(c, a, b) < need {
                return Some(format!("via of {}", board.nets[vn as usize].name));
            }
        }
        for pi in self.piece_idx.query(&rr) {
            let (p, q, ow, tj) = self.pieces[pi as usize];
            if tj == u32::MAX || tj as usize == ti {
                continue;
            }
            let o = &geo.tracks[tj as usize];
            if o.net == net || o.layer != layer {
                continue;
            }
            let need = half + ow / 2.0 + board.track_clearance(net, o.net) + MARGIN;
            if seg_seg_dist(a, b, p, q) < need {
                return Some(format!("track of {}", board.nets[o.net as usize].name));
            }
        }
        None
    }

    /// The widths for the polyline `legs` of track `ti`, if it fits: every leg takes the widest
    /// of `wide` (what the stretch it replaces had, if not narrower than the track), the track
    /// width and, where a neck is allowed, the stretch's narrowest width and the neck width, that
    /// keeps every clearance. A narrow stretch moved out of a neck zone must not stay narrow.
    fn fits(&self, geo: &Geometry, ti: usize, legs: &[Point], w: Span) -> Option<Vec<f64>> {
        let t = &geo.tracks[ti];
        legs.windows(2)
            .map(|p| {
                let (a, b) = (p[0], p[1]);
                let mut cand = vec![w.wide.max(t.width), t.width];
                if self.may_neck(ti, a, b) {
                    cand.push(w.narrow);
                    cand.push(t.neck_w);
                }
                cand.retain(|&c| c > 0.0);
                cand.sort_by(|x, y| y.total_cmp(x));
                cand.dedup_by(|x, y| (*x - *y).abs() < 1e-9);
                cand.into_iter().find(|&c| self.clear(geo, ti, a, b, c))
            })
            .collect()
    }

    /// Whether a piece a-b of track `ti` may be narrower than the track: as in
    /// `relax::widen_necks`, it touches a neck zone, or lies inside one if a minimum width rule
    /// applies to the net.
    fn may_neck(&self, ti: usize, a: Point, b: Point) -> bool {
        let (net, necks) = &self.necks[ti];
        // a minimum width rule allows a neck only where the piece touches a courtyard, and the
        // zone is just the box of it: there the router's own necks stay as they are
        self.board.class_of(*net).min_width.is_none() && necks.iter().any(|r| r.touches_seg(a, b))
    }
}

/// A stretch of a track: one the step may change, as a path, or one that stays as the router
/// drew it (with the width of every element).
#[derive(Clone)]
enum Part {
    Free(Path),
    Fixed(Vec<(Elem, f64)>),
}

impl Part {
    fn end(&self) -> Option<Point> {
        match self {
            Part::Free(p) => p.last().map(|q| q.0),
            Part::Fixed(e) => e.last().map(|&(e, _)| ends(e).1),
        }
    }
}

fn ends(e: Elem) -> (Point, Point) {
    match e {
        Elem::Line(a, b) | Elem::Arc(a, _, b) => (a, b),
    }
}

/// The points of an element after its start.
fn elem_points(e: Elem) -> Vec<Point> {
    match e {
        Elem::Line(_, b) => vec![b],
        Elem::Arc(a, m, b) => crate::geom::arc_polyline(a, m, b, 0.001).into_iter().skip(1).collect(),
    }
}

/// A track as the writer puts it into the file (an arc has the width of the piece between its
/// ends), cut into parts. Under a minimum width rule the router's necks stay as they are: the
/// rule allows them only where the element itself touches a courtyard, and a neck cut into
/// pieces or moved could leave it.
fn parts(board: &Board, t: &crate::realize::Track, hold: &dyn Fn(usize) -> bool) -> (Point, Vec<Part>) {
    let start = t.pts.first().copied().unwrap_or_default();
    if t.elems.is_empty() {
        let path = t.pts.windows(2).map(|w| (w[1], t.piece_width(w[0], w[1]))).collect();
        return (start, vec![Part::Free(path)]);
    }
    let ruled = board.class_of(t.net).min_width.is_some();
    let mut out: Vec<Part> = Vec::new();
    for (i, &e) in t.elems.iter().enumerate() {
        let (a, b) = ends(e);
        let w = t.piece_width(a, b);
        if (ruled && w < t.width - 1e-9) || hold(i) {
            match out.last_mut() {
                Some(Part::Fixed(v)) => v.push((e, w)),
                _ => out.push(Part::Fixed(vec![(e, w)])),
            }
        } else {
            let pts = elem_points(e).into_iter().map(|p| (p, w));
            match out.last_mut() {
                Some(Part::Free(v)) => v.extend(pts),
                _ => out.push(Part::Free(pts.collect())),
            }
        }
    }
    (ends(t.elems[0]).0, out)
}

/// All parts as one path.
fn flat(parts: &[Part]) -> Path {
    let mut out = Path::new();
    for part in parts {
        match part {
            Part::Free(p) => out.extend(p.iter().copied()),
            Part::Fixed(e) => {
                for &(e, w) in e {
                    out.extend(elem_points(e).into_iter().map(|p| (p, w)));
                }
            }
        }
    }
    out
}

/// The net of a track and its neck zones.
type Necks = (NetId, Vec<Rect>);

/// A polyline after its start point: every point with the width of the leg that ends there.
type Path = Vec<(Point, f64)>;

/// The widths of the stretch that a new path replaces.
#[derive(Clone, Copy)]
struct Span {
    wide: f64,
    narrow: f64,
}

impl Span {
    fn of(widths: &[f64]) -> Span {
        Span {
            wide: widths.iter().copied().fold(0.0, f64::max),
            narrow: widths.iter().copied().fold(f64::INFINITY, f64::min),
        }
    }
}

fn with_widths(pts: &[Point], widths: Vec<f64>) -> Path {
    pts.iter().copied().zip(widths).collect()
}

/// One or two legs on the grid from `a` to `b` for track `ti` (the one that turns least from
/// `incoming` first), or three in a Z. Returns the path after `a`.
fn connect(env: &Env, geo: &Geometry, ti: usize, a: Point, b: Point, incoming: Option<usize>, w: Span) -> Option<Path> {
    if grid_dir(b - a).is_some() || a.dist(b) < MIN_LEG {
        return env.fits(geo, ti, &[a, b], w).map(|ws| with_widths(&[b], ws));
    }
    let (corners, ks) = two_legs(a, b)?;
    let mut order = [0usize, 1];
    if let Some(k) = incoming
        && turn(k, ks[1]) < turn(k, ks[0])
    {
        order = [1, 0];
    }
    for i in order {
        if let Some(ws) = env.fits(geo, ti, &[a, corners[i], b], w) {
            return Some(with_widths(&[corners[i], b], ws));
        }
    }
    // three legs: the outer direction split around the inner one (a Z), which stays closer to
    // the straight line a-b
    for f in [0.5, 0.25, 0.75] {
        for i in order {
            let p1 = a + (corners[i] - a) * f;
            let p2 = p1 + (b - corners[i]);
            if let Some(ws) = env.fits(geo, ti, &[a, p1, p2, b], w) {
                return Some(with_widths(&[p1, p2, b], ws));
            }
        }
    }
    None
}

/// Direction of the last leg of a path, if it lies on the grid.
fn last_dir(start: Point, path: &[(Point, f64)]) -> Option<usize> {
    match path {
        [] => None,
        [(b, _)] => grid_dir(*b - start),
        [.., (a, _), (b, _)] => grid_dir(*b - *a),
    }
}

/// From `a` to `b` (a straight piece of the original) with a staircase of up to `depth` halvings.
#[allow(clippy::too_many_arguments)]
fn staircase(
    env: &Env,
    geo: &Geometry,
    ti: usize,
    a: Point,
    b: Point,
    incoming: Option<usize>,
    w: Span,
    depth: usize,
) -> Option<Path> {
    if let Some(p) = connect(env, geo, ti, a, b, incoming, w) {
        return Some(p);
    }
    if depth == 0 {
        return None;
    }
    let m = a.lerp(b, 0.5);
    let mut path = staircase(env, geo, ti, a, m, incoming, w, depth - 1)?;
    let inc = last_dir(a, &path);
    path.extend(staircase(env, geo, ti, m, b, inc, w, depth - 1)?);
    Some(path)
}

/// Chamfer the corners of a path that turn by 90 or 135 degrees with 45 degree pieces, where
/// they fit. Returns the path and the corners left sharp.
fn chamfer(env: &Env, geo: &Geometry, ti: usize, start: Point, path: Path) -> (Path, usize) {
    let mut out: Path = Vec::with_capacity(path.len());
    let mut sharp = 0;
    let d = dirs();
    let mut a = start;
    for i in 0..path.len() {
        let (b, w_in) = path[i];
        let Some(&(c, w_out)) = path.get(i + 1) else {
            out.push((b, w_in));
            break;
        };
        let (Some(ku), Some(kw)) = (grid_dir(b - a), grid_dir(c - b)) else {
            out.push((b, w_in));
            a = b;
            continue;
        };
        let steps = turn(ku, kw);
        if steps <= 1 || steps >= 4 {
            out.push((b, w_in));
            a = b;
            continue;
        }
        // intermediate directions, 45 degrees apart, turning the short way
        let sign: isize = if (kw + 8 - ku) % 8 <= 4 { 1 } else { -1 };
        let mids: Vec<Point> = (1..steps as isize)
            .map(|s| d[((ku as isize + sign * s).rem_euclid(8)) as usize])
            .collect();
        let (u, wv) = (d[ku], d[kw]);
        let msum = mids.iter().fold(pt(0.0, 0.0), |s, &m| s + m);
        // b - u*t + msum*s = b + w*t2: solve for s and t2 given t
        let den = msum.cross(wv);
        let (len_in, len_out) = (a.dist(b), b.dist(c));
        let span = Span::of(&[w_in, w_out]);
        let mut tt = len_in.min(len_out * 0.5);
        let mut done = false;
        while tt >= 0.01 && den.abs() > 1e-12 {
            let rhs = u * tt;
            let s = rhs.cross(wv) / den;
            let t2 = msum.cross(rhs) / den;
            if s > MIN_LEG && t2 > 0.0 && t2 <= len_out * 0.5 + 1e-9 {
                let p1 = b - u * tt;
                let mut poly = vec![p1];
                for &m in &mids {
                    poly.push(*poly.last().unwrap() + m * s);
                }
                if let Some(ws) = env.fits(geo, ti, &poly, span) {
                    if p1.dist(a) > 1e-9 {
                        out.push((p1, w_in));
                    }
                    out.extend(with_widths(&poly[1..], ws));
                    a = *poly.last().unwrap();
                    done = true;
                    break;
                }
            }
            tt *= 0.5;
        }
        if !done {
            out.push((b, w_in));
            a = b;
            sharp += 1;
        }
    }
    (out, sharp)
}

fn length(pts: &[Point]) -> f64 {
    pts.windows(2).map(|w| w[0].dist(w[1])).sum()
}

/// Drop points of a path that lie on the line between their neighbours with the same width,
/// and points that repeat.
fn tidy(start: Point, path: Path) -> Path {
    let mut out: Path = Vec::with_capacity(path.len());
    for (p, w) in path {
        let prev = out.last().map_or(start, |q| q.0);
        if p.dist(prev) < 1e-9 {
            continue;
        }
        if let Some(&(q, wq)) = out.last() {
            let before = if out.len() >= 2 { out[out.len() - 2].0 } else { start };
            if (wq - w).abs() < 1e-9 && point_seg_dist(q, before, p) < 1e-7 && (q - before).dot(p - q) > 0.0 {
                out.pop();
            }
        }
        out.push((p, w));
    }
    out
}

/// One pass over track `ti`: from its start, as far ahead as one or two legs on the grid reach,
/// then on from there. Returns the start, the new path and the corners left sharp.
fn convert(env: &Env, geo: &Geometry, ti: usize, start: Point, path: &[(Point, f64)], stairs: bool) -> (Path, usize) {
    let path = simplify_path(start, path, 0.0005);
    let src: Vec<Point> = std::iter::once(start).chain(path.iter().map(|p| p.0)).collect();
    let n = src.len();
    let widths: Vec<f64> = path.iter().map(|p| p.1).collect();
    let mut out: Path = Vec::new();
    let mut i = 0;
    while i < n - 1 {
        let cur = src[i];
        let inc = last_dir(start, &out);
        let try_to = |j: usize| connect(env, geo, ti, cur, src[j], inc, Span::of(&widths[i..j]));
        // farthest point reachable with one or two legs: exponential search, then bisection
        let mut best: Option<(usize, Path)> = None;
        if let Some(p) = try_to(i + 1) {
            best = Some((i + 1, p));
            let mut good = i + 1;
            let mut step = 1;
            let mut bad = n;
            while good + step < n {
                let k = good + step;
                match try_to(k) {
                    Some(p) => {
                        best = Some((k, p));
                        good = k;
                        step *= 2;
                    }
                    None => {
                        bad = k;
                        break;
                    }
                }
            }
            let mut hi = bad;
            while hi - good > 1 {
                let mid = (good + hi) / 2;
                match try_to(mid) {
                    Some(p) => {
                        best = Some((mid, p));
                        good = mid;
                    }
                    None => hi = mid,
                }
            }
        }
        match best {
            Some((j, p)) => {
                out.extend(p);
                i = j;
            }
            None => {
                let b = src[i + 1];
                let span = Span::of(&widths[i..i + 1]);
                match stairs.then(|| staircase(env, geo, ti, cur, b, inc, span, 1)).flatten() {
                    Some(p) => out.extend(p),
                    None => out.push((b, widths[i])),
                }
                i += 1;
            }
        }
    }
    let out = tidy(start, out);
    let (out, sharp) = chamfer(env, geo, ti, start, out);
    (tidy(start, out), sharp)
}

/// `relax::simplify` on every run of one width of a path.
fn simplify_path(start: Point, path: &[(Point, f64)], tol: f64) -> Path {
    let mut out: Path = Vec::with_capacity(path.len());
    let mut i = 0;
    let mut from = start;
    while i < path.len() {
        let w = path[i].1;
        let mut j = i;
        while j + 1 < path.len() && (path[j + 1].1 - w).abs() < 1e-12 {
            j += 1;
        }
        let run: Vec<Point> = std::iter::once(from).chain(path[i..=j].iter().map(|p| p.0)).collect();
        out.extend(crate::relax::simplify(&run, tol).into_iter().skip(1).map(|p| (p, w)));
        from = path[j].0;
        i = j + 1;
    }
    out
}

/// Pieces off the grid and their length.
fn off_grid(pts: &[Point]) -> (usize, f64) {
    let mut out = (0, 0.0);
    for w in pts.windows(2) {
        if grid_dir(w[1] - w[0]).is_none() && w[0].dist(w[1]) >= MIN_LEG {
            out.0 += 1;
            out.1 += w[0].dist(w[1]);
        }
    }
    out
}

/// The 45 degree step on the copper a board already has, without routing again: the unlocked
/// tracks become soft copper (see [`Board::soften`]), every run of track between two vias,
/// branches or pads is converted like a routed track, and [`crate::import::soft_changes`] tells
/// which file items to replace. Only the tracks of `nets` change (all nets if it is empty); the
/// others, locked tracks, vias and track ends stay where they are.
pub fn octilinear_copper(board: &mut Board, nets: &[String]) -> (Geometry, crate::import::SoftChanges, OctoStats) {
    board.soften(&[]);
    let board = &*board;
    let pick = |n: NetId| nets.is_empty() || nets.contains(&board.nets[n as usize].name);
    let mut geo = Geometry::default();
    for (i, ch) in board.soft.chains.iter().enumerate() {
        geo.tracks.push(Track {
            pts: ch.pts.clone(),
            elems: ch.elems.clone(),
            width: ch.width,
            layer: ch.layer,
            net: ch.net,
            necks: Vec::new(),
            neck_w: ch.width,
            // `octilinear` leaves what has an origin alone
            origin: (!pick(ch.net)).then_some(i as u32),
            resting: false,
            fitted: Vec::new(),
            pattern: None,
            seg: u32::MAX,
            tap: None,
        });
    }
    for (i, n) in board.soft.nodes.iter().enumerate() {
        geo.vias.push(Via {
            at: n.at,
            diameter: n.r * 2.0,
            drill: n.drill,
            net: n.net,
            layer: n.layer,
            origin: Some(i as u32),
            resting: true,
        });
    }
    let stats = octilinear(board, &mut geo);
    for (i, t) in geo.tracks.iter_mut().enumerate() {
        let old = &board.soft.chains[i].elems;
        let same = |a: Point, b: Point| a.dist(b) < 1e-6;
        t.resting = old.len() == t.elems.len()
            && old.iter().zip(&t.elems).all(|(x, y)| match (*x, *y) {
                (Elem::Line(a, b), Elem::Line(c, d)) => same(a, c) && same(b, d),
                (Elem::Arc(a, m, b), Elem::Arc(c, n, d)) => same(a, c) && same(m, n) && same(b, d),
                _ => false,
            });
        if t.resting {
            t.fitted.clear();
        }
        t.origin = Some(i as u32);
    }
    let changes = crate::import::soft_changes(board, &geo);
    (geo, changes, stats)
}

/// Passes over all tracks. A track converted early may stand in the way of its neighbours, and
/// a bundle frees its room only from the outside in, so every pass works on the result of the
/// one before: what got room since then straightens out. Staircases only in the last pass.
const PASSES: usize = 4;

/// Turn the routed tracks of `geo` into 45 degree routing, see the module documentation.
/// Imported copper (`origin` set) stays as it is.
pub fn octilinear(board: &Board, geo: &mut Geometry) -> OctoStats {
    octilinear_only(board, geo, &|_| true, &|_, _| false)
}

/// [`octilinear`] for the tracks that `take` holds (the wide tracks of the pairs before they are
/// split, and afterwards everything but the two tracks they became); the elements that `hold`
/// names (track index, element index) stay as they are.
pub fn octilinear_only(
    board: &Board,
    geo: &mut Geometry,
    take: &dyn Fn(&Track) -> bool,
    hold: &dyn Fn(usize, usize) -> bool,
) -> OctoStats {
    let mut stats = OctoStats::default();
    let todo: Vec<usize> = (0..geo.tracks.len())
        .filter(|&ti| geo.tracks[ti].origin.is_none() && geo.tracks[ti].pts.len() >= 2 && take(&geo.tracks[ti]))
        .collect();
    stats.tracks = todo.len();
    stats.length_before = todo.iter().map(|&ti| length(&geo.tracks[ti].pts)).sum();
    let mut env = Env::new(board, geo);
    let mut sharp = vec![0usize; geo.tracks.len()];
    // every track as the router left it, cut into what may change and what stays
    let mut tracks: Vec<(Point, Vec<Part>)> = geo
        .tracks
        .iter()
        .enumerate()
        .map(|(ti, t)| parts(board, t, &|ei| hold(ti, ei)))
        .collect();
    for pass in 0..PASSES {
        let last = pass + 1 == PASSES;
        let mut changed = false;
        for &ti in &todo {
            let start = tracks[ti].0;
            let mut from = start;
            let mut touched = false;
            sharp[ti] = 0;
            for k in 0..tracks[ti].1.len() {
                if let Part::Free(path) = &tracks[ti].1[k]
                    && !path.is_empty()
                {
                    let (out, s) = convert(&env, geo, ti, from, path, last);
                    sharp[ti] += s;
                    if out != *path {
                        tracks[ti].1[k] = Part::Free(out);
                        touched = true;
                    }
                }
                from = tracks[ti].1[k].end().unwrap_or(from);
            }
            if touched {
                changed = true;
                env.replace(ti, start, &flat(&tracks[ti].1));
            }
        }
        if !changed {
            break;
        }
    }
    for &ti in &todo {
        let (start, parts) = &tracks[ti];
        let mut elems: Vec<(Elem, f64)> = Vec::new();
        let mut from = *start;
        for part in parts {
            match part {
                Part::Fixed(e) => elems.extend(e.iter().copied()),
                Part::Free(path) => {
                    for &(p, w) in path {
                        elems.push((Elem::Line(from, p), w));
                        from = p;
                    }
                }
            }
            from = part.end().unwrap_or(from);
        }
        let t = &mut geo.tracks[ti];
        // every piece carries its width: a zone at the middle between its ends, which touches
        // no other piece (the neck zones stay for the check of the flattened arcs)
        t.fitted = elems
            .iter()
            .flat_map(|&(e, w)| {
                let (a, b) = ends(e);
                // the writer takes the width between the ends, the check the flattened pieces
                let pieces: Vec<Point> = match e {
                    Elem::Line(..) => Vec::new(),
                    Elem::Arc(..) => std::iter::once(a)
                        .chain(elem_points(e))
                        .collect::<Vec<_>>()
                        .windows(2)
                        .map(|q| q[0].lerp(q[1], 0.5))
                        .collect(),
                };
                std::iter::once(a.lerp(b, 0.5))
                    .chain(pieces)
                    .map(move |m| (seg_box(m, m).grow(1e-7), w))
            })
            .collect();
        t.elems = elems.into_iter().map(|(e, _)| e).collect();
        t.pts = crate::realize::flatten(&t.elems);
    }
    if std::env::var("GB_OCTO_DEBUG").is_ok() {
        for &ti in &todo {
            let t = &geo.tracks[ti];
            for w in t.pts.windows(2) {
                let (a, b) = (w[0], w[1]);
                if grid_dir(b - a).is_some() || a.dist(b) < MIN_LEG {
                    continue;
                }
                let width = t.piece_width(a, b);
                let span = Span {
                    wide: width,
                    narrow: width,
                };
                let why: Vec<String> = two_legs(a, b)
                    .map(|(c, _)| {
                        c.iter()
                            .map(|&c| {
                                let mut cand = vec![width, t.width];
                                if env.may_neck(ti, a, c) && env.may_neck(ti, c, b) {
                                    cand.push(t.neck_w);
                                }
                                let w = cand.into_iter().fold(f64::INFINITY, f64::min);
                                if env.fits(geo, ti, &[a, c, b], span).is_some() {
                                    return "fits now".to_string();
                                }
                                env.blocker(geo, ti, a, c, w)
                                    .or_else(|| env.blocker(geo, ti, c, b, w))
                                    .unwrap_or_else(|| "neck not allowed".into())
                            })
                            .collect()
                    })
                    .unwrap_or_default();
                eprintln!(
                    "off grid: {} layer {} {:.3},{:.3} -> {:.3},{:.3} ({:.2} mm): {:?}",
                    board.nets[t.net as usize].name,
                    t.layer,
                    a.x,
                    a.y,
                    b.x,
                    b.y,
                    a.dist(b),
                    why
                );
            }
        }
    }
    for &ti in &todo {
        let pts = &geo.tracks[ti].pts;
        let (k, l) = off_grid(pts);
        stats.any_angle += k;
        stats.any_angle_length += l;
        stats.length_after += length(pts);
        stats.sharp_corners += sharp[ti];
    }
    stats
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{CopperItem, CopperKind, LayerSet, Obstacle};
    use crate::geom::Shape;

    /// The two pads of net A at (2, 5) and (8, 5), joined by an unlocked track through `pts`.
    fn routed_board(pts: &[Point]) -> Board {
        let mut b = crate::topo::tests::two_pad_board();
        for (i, w) in pts.windows(2).enumerate() {
            let first = b.obstacles.len() as u32;
            b.obstacles.push(Obstacle {
                shape: Shape::Capsule {
                    a: w[0],
                    b: w[1],
                    r: 0.1,
                },
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
                    a: w[0],
                    b: w[1],
                    width: 0.2,
                    layer: 0,
                },
                net: Some(0),
                locked: false,
                source: 10 + i as u32,
                first,
                last: first + 1,
                pattern: false,
            });
        }
        b
    }

    #[test]
    fn copper_of_a_file_turns_45_degrees_in_place() {
        let mut b = routed_board(&[pt(2.0, 5.0), pt(5.0, 6.3), pt(8.0, 5.0)]);
        let (geo, changes, st) = octilinear_copper(&mut b, &[]);
        assert_eq!(st.tracks, 1);
        assert_eq!(changes.strip, vec![10, 11]);
        let t = &geo.tracks[0];
        assert!(!t.resting);
        assert!(t.pts[0].dist(pt(2.0, 5.0)) < 1e-9 && t.pts.last().unwrap().dist(pt(8.0, 5.0)) < 1e-9);
        for w in t.pts.windows(2) {
            assert!(grid_dir(w[1] - w[0]).is_some(), "{:?}", t.pts);
        }
        assert!(crate::check::check(&b, &geo).is_empty());
    }

    #[test]
    fn copper_on_the_grid_or_of_other_nets_stays() {
        let mut b = routed_board(&[pt(2.0, 5.0), pt(8.0, 5.0)]);
        let (_, changes, _) = octilinear_copper(&mut b, &[]);
        assert!(changes.strip.is_empty());
        let mut b = routed_board(&[pt(2.0, 5.0), pt(5.0, 6.3), pt(8.0, 5.0)]);
        let (geo, changes, st) = octilinear_copper(&mut b, &["B".to_string()]);
        assert_eq!(st.tracks, 0);
        assert!(changes.strip.is_empty());
        assert!(geo.tracks[0].resting);
    }

    #[test]
    fn two_legs_lie_on_the_grid() {
        let (a, b) = (pt(0.0, 0.0), pt(3.0, 1.0));
        let (c, ks) = two_legs(a, b).unwrap();
        for (i, k) in ks.iter().enumerate() {
            let k2 = ks[1 - i];
            assert_eq!(grid_dir(c[i] - a), Some(*k));
            assert_eq!(grid_dir(b - c[i]), Some(k2));
        }
        assert!(two_legs(a, pt(2.0, 2.0)).is_none());
        assert!(two_legs(a, pt(0.0, -2.0)).is_none());
    }

    #[test]
    fn turns() {
        assert_eq!(turn(0, 2), 2);
        assert_eq!(turn(7, 1), 2);
        assert_eq!(turn(1, 5), 4);
        assert_eq!(turn(3, 0), 3);
    }
}
