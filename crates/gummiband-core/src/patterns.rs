//! Local patterns of a board, found once from its pins: rows of pins, pad grids (BGA),
//! decoupling capacitors, neighbouring pins that share a plane, buses between two parts.
//!
//! Every pattern only feeds hints to the search (an order, a shared fan-out, a move of the open
//! end); none of them fixes copper or forbids a way. The search decides, so a pattern that does
//! not fit a board costs at most the run that tried it. See `docs/ARCHITECTURE.md`.

use std::collections::{BTreeMap, HashMap};

use crate::board::{Board, NetId};
use crate::geom::Point;

/// Pins of one footprint on a straight line at a regular pitch.
#[derive(Debug, Clone, serde::Serialize)]
pub struct PinRow {
    pub footprint: String,
    /// Pin indices in order along the row.
    pub pins: Vec<usize>,
    /// Unit vector along the row, from the first pin to the last.
    pub dir: Point,
    /// Unit normal pointing away from the footprint centre (the side tracks leave to).
    pub out: Point,
    pub pitch: f64,
}

/// Pins of one footprint on a grid of at least 3 x 3 (a BGA field).
#[derive(Debug, Clone, serde::Serialize)]
pub struct PadGrid {
    pub footprint: String,
    pub pins: Vec<usize>,
    pub pitch: f64,
    pub rows: usize,
    pub cols: usize,
    pub center: Point,
    /// Unit vector along one axis of the grid (the other axis is its normal).
    pub dir: Point,
}

/// A two pin part next to a pin of a larger part: one pin on a plane net (or a plane like
/// return), the other on the supply net it shares with that pin.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Decap {
    pub footprint: String,
    /// The capacitor pin on the supply net of the IC pin.
    pub supply_pin: usize,
    /// The other capacitor pin (usually GND).
    pub return_pin: usize,
    /// The IC pin it decouples.
    pub ic_pin: usize,
    pub dist: f64,
}

/// Pins of one plane net close to each other that can reach the plane through one via: the
/// leader fans out, the others connect to the leader on their layer.
#[derive(Debug, Clone, serde::Serialize)]
pub struct PlaneShare {
    pub net: NetId,
    pub leader: usize,
    pub others: Vec<usize>,
    /// "neighbours" (pins of one part) or "decap" (an IC pin and its capacitor).
    pub kind: &'static str,
}

/// Nets that each join one pin of part `a` to one pin of part `b`, pins next to each other in
/// both rows. `ordered`: the order along the rows is kept (no crossing needed).
#[derive(Debug, Clone, serde::Serialize)]
pub struct Bus {
    pub a: String,
    pub b: String,
    pub nets: Vec<NetId>,
    pub ordered: bool,
}

#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct Patterns {
    pub rows: Vec<PinRow>,
    pub grids: Vec<PadGrid>,
    pub decaps: Vec<Decap>,
    pub shares: Vec<PlaneShare>,
    pub buses: Vec<Bus>,
}

/// Pins of each footprint, by reference, in a stable order.
fn by_footprint(board: &Board) -> BTreeMap<&str, Vec<usize>> {
    let mut m: BTreeMap<&str, Vec<usize>> = BTreeMap::new();
    for (i, p) in board.pins.iter().enumerate() {
        m.entry(p.footprint.as_str()).or_default().push(i);
    }
    m
}

fn centre(board: &Board, pins: &[usize]) -> Point {
    let mut c = Point::default();
    for &p in pins {
        c = c + board.pins[p].pos;
    }
    c * (1.0 / pins.len().max(1) as f64)
}

/// The smallest distance between two pins of the set (pads that overlap count as one).
fn pitch_of(board: &Board, pins: &[usize]) -> f64 {
    let mut best = f64::INFINITY;
    for (k, &a) in pins.iter().enumerate() {
        for &b in &pins[k + 1..] {
            let d = board.pins[a].pos.dist(board.pins[b].pos);
            if d > 0.05 {
                best = best.min(d);
            }
        }
    }
    best
}

/// Rows of pins on a line, at most 1.1 times the pitch apart: chains of neighbours that keep
/// one direction.
fn rows_of(board: &Board, fp: &str, pins: &[usize], pitch: f64) -> Vec<PinRow> {
    let pos = |p: usize| board.pins[p].pos;
    let c = centre(board, pins);
    // neighbour links in a direction, folded to a half turn
    let mut used: HashMap<(usize, usize), bool> = HashMap::new();
    let mut rows = Vec::new();
    for (k, &a) in pins.iter().enumerate() {
        for &b in &pins[k + 1..] {
            let d = pos(b) - pos(a);
            let l = d.len();
            if l < 0.05 || l > pitch * 1.1 || used.contains_key(&(a, b)) {
                continue;
            }
            let dir = d * (1.0 / l);
            // grow the chain both ways along dir
            let on_line = |q: usize, from: usize, sign: f64| -> bool {
                let e = pos(q) - pos(from);
                let el = e.len();
                el > 0.05 && el <= pitch * 1.1 && (e * (1.0 / el)).dot(dir * sign) > 0.999
            };
            let mut chain = vec![a, b];
            for sign in [1.0, -1.0] {
                loop {
                    let end = if sign > 0.0 { *chain.last().unwrap() } else { chain[0] };
                    let Some(&q) = pins.iter().find(|&&q| !chain.contains(&q) && on_line(q, end, sign)) else {
                        break;
                    };
                    if sign > 0.0 {
                        chain.push(q);
                    } else {
                        chain.insert(0, q);
                    }
                }
            }
            for w in chain.windows(2) {
                let key = (w[0].min(w[1]), w[0].max(w[1]));
                used.insert(key, true);
            }
            if chain.len() < 4 {
                continue;
            }
            let (first, last) = (pos(chain[0]), pos(*chain.last().unwrap()));
            let dir = (last - first).norm();
            let mid = (first + last) * 0.5;
            let mut out = dir.perp();
            if (mid - c).dot(out) < 0.0 {
                out = -out;
            }
            rows.push(PinRow {
                footprint: fp.to_string(),
                pins: chain,
                dir,
                out,
                pitch,
            });
        }
    }
    rows
}

/// A grid if the rows run in two directions and both have at least 3 rows of at least 3 pins.
fn grid_of(board: &Board, fp: &str, pins: &[usize], pitch: f64, rows: &[PinRow]) -> Option<PadGrid> {
    if pins.len() < 9 {
        return None;
    }
    let d0 = rows.first()?.dir;
    let (along, across): (Vec<&PinRow>, Vec<&PinRow>) = rows.iter().partition(|r| r.dir.dot(d0).abs() > 0.9);
    let straight = |v: &[&PinRow]| v.iter().all(|r| r.dir.dot(d0).abs() < 0.1 || r.dir.dot(d0).abs() > 0.9);
    if along.len() < 3 || across.len() < 3 || !straight(&across) {
        return None;
    }
    // most pins must lie on rows of both directions (a QFN has no pin on two rows)
    let on = |set: &[&PinRow], p: usize| set.iter().any(|r| r.pins.contains(&p));
    let both = pins.iter().filter(|&&p| on(&along, p) && on(&across, p)).count();
    if both * 2 < pins.len() {
        return None;
    }
    // rows broken by missing balls are one line of the grid: count the lines, not the pieces
    let lines = |set: &[&PinRow]| {
        let n = set[0].dir.perp();
        let mut at: Vec<i64> = set
            .iter()
            .map(|r| (board.pins[r.pins[0]].pos.dot(n) / (pitch * 0.25)).round() as i64)
            .collect();
        at.sort();
        at.dedup_by(|a, b| (*a - *b).abs() <= 1);
        at.len()
    };
    Some(PadGrid {
        footprint: fp.to_string(),
        pins: pins.to_vec(),
        pitch,
        rows: lines(&along),
        cols: lines(&across),
        center: centre(board, pins),
        dir: d0,
    })
}

/// The ball fields of a board for via placement: per part, the SMD pads that have a neighbour
/// one pitch away along both axes of a square lattice, if they span at least 3 lines in each
/// direction and make at least half of the part's SMD pads. Unlike [`find`] this takes fields
/// of short rows too (a DDR package is two blocks of 3 columns), and leaves out the pads off
/// the lattice (support balls, a thermal pad).
pub fn grids(board: &Board) -> Vec<PadGrid> {
    let mut out = Vec::new();
    for (fp, all) in &by_footprint(board) {
        let smd = |p: usize| {
            board.pins[p]
                .obstacles
                .iter()
                .all(|&o| board.obstacles[o].hole.is_none())
        };
        let pins: Vec<usize> = all.iter().copied().filter(|&p| smd(p)).collect();
        if pins.len() < 9 {
            continue;
        }
        let pos = |p: usize| board.pins[p].pos;
        // the pitch is the usual distance to the nearest pad (the smallest one may come from a
        // pad between the balls, or from the two pins of a merged pair)
        let mut nearest: Vec<(f64, usize, usize)> = pins
            .iter()
            .filter_map(|&a| {
                pins.iter()
                    .filter(|&&b| b != a && pos(a).dist(pos(b)) > 0.05)
                    .map(|&b| (pos(a).dist(pos(b)), a, b))
                    .min_by(|x, y| x.0.total_cmp(&y.0))
            })
            .collect();
        nearest.sort_by(|x, y| x.0.total_cmp(&y.0));
        let Some(&(pitch, a, b)) = nearest.get(nearest.len() / 2) else {
            continue;
        };
        let u = (pos(b) - pos(a)).norm();
        let v = u.perp();
        let tol = 0.1 * pitch;
        let has = |a: Point, d: Point| pins.iter().any(|&q| (pos(q) - (a + d)).len() < tol);
        let balls: Vec<usize> = pins
            .iter()
            .copied()
            .filter(|&p| {
                let a = pos(p);
                (has(a, u * pitch) || has(a, u * -pitch)) && (has(a, v * pitch) || has(a, v * -pitch))
            })
            .collect();
        let lines = |n: Point| {
            let mut at: Vec<f64> = balls.iter().map(|&p| pos(p).dot(n)).collect();
            at.sort_by(f64::total_cmp);
            at.dedup_by(|a, b| (*a - *b).abs() < tol);
            at.len()
        };
        if balls.len() * 2 < pins.len() || lines(u) < 3 || lines(v) < 3 {
            continue;
        }
        out.push(PadGrid {
            footprint: fp.to_string(),
            rows: lines(v),
            cols: lines(u),
            center: centre(board, &balls),
            pins: balls,
            pitch,
            dir: u,
        });
    }
    out
}

fn has_plane(board: &Board, net: NetId) -> bool {
    board.planes.iter().any(|p| p.net == net)
}

/// A pin on a plane net whose pad reaches the plane on its own (through hole) needs no fan-out.
fn reaches_plane(board: &Board, p: usize) -> bool {
    let pin = &board.pins[p];
    board
        .planes
        .iter()
        .any(|pl| pl.net == pin.net && pin.layers.has(pl.layer) && pl.covers(pin.pos))
}

/// Find the patterns of a board.
pub fn find(board: &Board) -> Patterns {
    let fps = by_footprint(board);
    let mut out = Patterns::default();
    for (fp, pins) in &fps {
        if pins.len() < 4 {
            continue;
        }
        let pitch = pitch_of(board, pins);
        if !pitch.is_finite() {
            continue;
        }
        let rows = rows_of(board, fp, pins, pitch);
        if let Some(g) = grid_of(board, fp, pins, pitch, &rows) {
            out.grids.push(g);
        } else {
            out.rows.extend(rows);
        }
    }
    out.decaps = decaps(board, &fps);
    let grids: Vec<&str> = out.grids.iter().map(|g| g.footprint.as_str()).collect();
    out.shares = shares(board, &fps, &grids, &out.decaps);
    out.buses = buses(board, &out.rows);
    out
}

/// Two pin parts next to a pin of a part with 6 or more pins, on a net they share; the other pin
/// of the part on a plane net, or the reference starts with C.
fn decaps(board: &Board, fps: &BTreeMap<&str, Vec<usize>>) -> Vec<Decap> {
    const REACH: f64 = 4.0;
    let mut out = Vec::new();
    for (fp, pins) in fps {
        if pins.len() != 2 {
            continue;
        }
        let is_cap = fp.starts_with('C') && fp[1..].chars().next().is_some_and(|c| c.is_ascii_digit());
        for (s, r) in [(pins[0], pins[1]), (pins[1], pins[0])] {
            let (sn, rn) = (board.pins[s].net, board.pins[r].net);
            if sn == rn || !(is_cap || has_plane(board, rn)) {
                continue;
            }
            // the closest pin of a larger part on the supply net
            let best = board.nets[sn as usize]
                .pins
                .iter()
                .copied()
                .filter(|&q| fps.get(board.pins[q].footprint.as_str()).is_some_and(|v| v.len() >= 6))
                .map(|q| (board.pins[q].pos.dist(board.pins[s].pos), q))
                .filter(|&(d, _)| d <= REACH)
                .min_by(|a, b| a.0.total_cmp(&b.0));
            if let Some((dist, ic)) = best {
                // the return pin must be a plane or a net with many pins (a ground), and the
                // supply must not be the plane side of a plane pair the other way round
                let returns = has_plane(board, rn) || board.nets[rn as usize].pins.len() >= 6;
                if returns && !(has_plane(board, sn) && !has_plane(board, rn)) {
                    out.push(Decap {
                        footprint: fp.to_string(),
                        supply_pin: s,
                        return_pin: r,
                        ic_pin: ic,
                        dist,
                    });
                    break;
                }
            }
        }
    }
    // one capacitor per IC pin: the closest
    out.sort_by(|a, b| a.ic_pin.cmp(&b.ic_pin).then(a.dist.total_cmp(&b.dist)));
    out.dedup_by_key(|d| d.ic_pin);
    out.sort_by(|a, b| a.footprint.cmp(&b.footprint));
    out
}

/// Plane pins that can share one fan-out: neighbours of one part on the same plane net, and an
/// IC pin with the capacitor pin that decouples it.
fn shares(board: &Board, fps: &BTreeMap<&str, Vec<usize>>, grids: &[&str], decaps: &[Decap]) -> Vec<PlaneShare> {
    let n = board.pins.len();
    let mut parent: Vec<usize> = (0..n).collect();
    fn root(parent: &mut [usize], mut a: usize) -> usize {
        while parent[a] != a {
            parent[a] = parent[parent[a]];
            a = parent[a];
        }
        a
    }
    let fan = |p: usize| has_plane(board, board.pins[p].net) && !reaches_plane(board, p);
    let mut kind: HashMap<usize, &'static str> = HashMap::new();
    let mut leader_of: HashMap<usize, usize> = HashMap::new();
    // capacitor and IC pin: the capacitor pin leads (the via goes at the capacitor)
    for d in decaps {
        let (c, ic) = (d.supply_pin, d.ic_pin);
        if fan(c) && fan(ic) && board.pins[c].layers.intersects(board.pins[ic].layers) {
            let (rc, ri) = (root(&mut parent, c), root(&mut parent, ic));
            if rc != ri {
                parent[ri] = rc;
                kind.insert(rc, "decap");
                leader_of.insert(rc, c);
            }
        }
    }
    // neighbours of one part: at most 1.6 times the pitch apart, on a common layer; not in a
    // pad grid, where a bridge between two balls closes an escape channel
    for (fp, pins) in fps {
        if pins.len() < 3 || grids.contains(fp) {
            continue;
        }
        let pitch = pitch_of(board, pins);
        for (k, &a) in pins.iter().enumerate() {
            for &b in &pins[k + 1..] {
                let (pa, pb) = (&board.pins[a], &board.pins[b]);
                if pa.net != pb.net || !fan(a) || !fan(b) || !pa.layers.intersects(pb.layers) {
                    continue;
                }
                let d = pa.pos.dist(pb.pos);
                if d < 0.05 || d > pitch * 1.6 {
                    continue;
                }
                // nothing of another net between them on the straight line
                if board.pins.iter().any(|q| {
                    q.net != pa.net
                        && q.layers.intersects(pa.layers)
                        && crate::geom::point_seg_dist(q.pos, pa.pos, pb.pos) < q.size.0.min(q.size.1) * 0.5
                }) {
                    continue;
                }
                let (ra, rb) = (root(&mut parent, a), root(&mut parent, b));
                if ra != rb {
                    parent[rb] = ra;
                    kind.entry(ra).or_insert("neighbours");
                    if let Some(l) = leader_of.remove(&rb) {
                        leader_of.insert(ra, l);
                        kind.insert(ra, "decap");
                    }
                }
            }
        }
    }
    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for p in 0..n {
        let r = root(&mut parent, p);
        if r != p || kind.contains_key(&r) {
            groups.entry(r).or_default().push(p);
        }
    }
    let mut out = Vec::new();
    for (r, pins) in groups {
        if pins.len() < 2 {
            continue;
        }
        let leader = leader_of.get(&r).copied().unwrap_or(pins[0]);
        out.push(PlaneShare {
            net: board.pins[leader].net,
            leader,
            others: pins.into_iter().filter(|&p| p != leader).collect(),
            kind: kind.get(&r).copied().unwrap_or("neighbours"),
        });
    }
    out
}

/// Buses: runs of at least 4 neighbouring pins of a row whose nets each go to exactly one pin of
/// one other row.
fn buses(board: &Board, rows: &[PinRow]) -> Vec<Bus> {
    let mut row_of: HashMap<usize, (usize, usize)> = HashMap::new();
    for (ri, r) in rows.iter().enumerate() {
        for (k, &p) in r.pins.iter().enumerate() {
            row_of.entry(p).or_insert((ri, k));
        }
    }
    let mut out = Vec::new();
    for (ri, r) in rows.iter().enumerate() {
        // the partner of every pin: the one other pin of a two pin net, in another row
        let partner = |p: usize| -> Option<(usize, usize)> {
            let net = &board.nets[board.pins[p].net as usize];
            if net.pins.len() != 2 {
                return None;
            }
            let q = if net.pins[0] == p { net.pins[1] } else { net.pins[0] };
            row_of.get(&q).copied().filter(|&(rj, _)| rj != ri)
        };
        let mut k = 0;
        while k < r.pins.len() {
            let Some((rj, _)) = partner(r.pins[k]) else {
                k += 1;
                continue;
            };
            let mut end = k + 1;
            while end < r.pins.len() && partner(r.pins[end]).is_some_and(|(x, _)| x == rj) {
                end += 1;
            }
            if end - k >= 4 && ri < rj {
                let idx: Vec<usize> = (k..end).map(|i| partner(r.pins[i]).unwrap().1).collect();
                // the order is kept if the straight lines from pin to pin do not cross
                let lines: Vec<(Point, Point)> = (k..end)
                    .zip(&idx)
                    .map(|(i, &j)| (board.pins[r.pins[i]].pos, board.pins[rows[rj].pins[j]].pos))
                    .collect();
                let crossed = lines.iter().enumerate().any(|(x, &(a, b))| {
                    lines[x + 1..]
                        .iter()
                        .any(|&(c, d)| crate::geom::segs_intersect(a, b, c, d))
                });
                out.push(Bus {
                    a: r.footprint.clone(),
                    b: rows[rj].footprint.clone(),
                    nets: (k..end).map(|i| board.pins[r.pins[i]].net).collect(),
                    ordered: !crossed,
                });
            }
            k = end;
        }
    }
    out
}

/// The hints that `route --patterns` knows and `all` switches on.
pub const HINTS: [&str; 5] = ["shares", "decaps", "rows", "pairs", "groups"];

/// `groups` with another place for the blocks in the routing order (see
/// [`crate::route::GroupOrder`]): `groups` alone is `long`, the longest block first.
pub const GROUP_VARIANTS: [&str; 3] = ["groups-long", "groups-short", "groups-inline"];

/// Parse a comma separated list of hints: names of [`HINTS`] or [`GROUP_VARIANTS`], `buses`
/// (experimental, not part of `all` yet), `all` or `none`.
pub fn hints(arg: &str) -> Result<Vec<&'static str>, String> {
    let mut out = Vec::new();
    for w in arg.split(',').map(str::trim).filter(|w| !w.is_empty()) {
        match w {
            "none" => {}
            "all" => out.extend(HINTS),
            "buses" => out.push("buses"),
            _ => match HINTS.iter().chain(&GROUP_VARIANTS).find(|&&a| a == w) {
                Some(&a) => out.push(a),
                None => {
                    return Err(format!(
                        "unknown hint {w:?} (shares, decaps, rows, pairs, groups, buses, groups-long, \
                         groups-short, groups-inline, all, none)"
                    ));
                }
            },
        }
    }
    let mut variants: Vec<&str> = out.iter().copied().filter(|w| GROUP_VARIANTS.contains(w)).collect();
    variants.dedup();
    if variants.len() > 1 {
        return Err(format!("one group order at a time, not {}", variants.join(" and ")));
    }
    Ok(out)
}

/// The patterns with pin and net names instead of indices, for reports and agents.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Named {
    pub rows: Vec<NamedRow>,
    pub grids: Vec<NamedGrid>,
    pub decaps: Vec<NamedDecap>,
    pub shared_fanouts: Vec<NamedShare>,
    pub buses: Vec<NamedBus>,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct NamedRow {
    pub part: String,
    pub pins: Vec<String>,
    pub pitch_mm: f64,
    /// Unit normal on the side tracks leave to, away from the part.
    pub out: [f64; 2],
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct NamedGrid {
    pub part: String,
    pub rows: usize,
    pub cols: usize,
    pub pitch_mm: f64,
    pub pins: usize,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct NamedDecap {
    pub part: String,
    pub ic_pin: String,
    pub supply_pin: String,
    pub return_pin: String,
    pub dist_mm: f64,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct NamedShare {
    pub net: String,
    /// The pin that gets the via.
    pub leader: String,
    /// The pins that connect to the leader instead of a via of their own.
    pub others: Vec<String>,
    pub kind: String,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct NamedBus {
    pub a: String,
    pub b: String,
    pub nets: Vec<String>,
    /// The order of the pins is kept from one part to the other (no crossing needed).
    pub ordered: bool,
}

fn r3(x: f64) -> f64 {
    (x * 1000.0).round() / 1000.0
}

impl Patterns {
    pub fn named(&self, board: &Board) -> Named {
        let pin = |p: usize| board.pins[p].name.clone();
        let net = |n: NetId| board.nets[n as usize].name.clone();
        Named {
            rows: self
                .rows
                .iter()
                .map(|r| NamedRow {
                    part: r.footprint.clone(),
                    pins: r.pins.iter().map(|&p| pin(p)).collect(),
                    pitch_mm: r3(r.pitch),
                    out: [r3(r.out.x), r3(r.out.y)],
                })
                .collect(),
            grids: self
                .grids
                .iter()
                .map(|g| NamedGrid {
                    part: g.footprint.clone(),
                    rows: g.rows,
                    cols: g.cols,
                    pitch_mm: r3(g.pitch),
                    pins: g.pins.len(),
                })
                .collect(),
            decaps: self
                .decaps
                .iter()
                .map(|d| NamedDecap {
                    part: d.footprint.clone(),
                    ic_pin: pin(d.ic_pin),
                    supply_pin: pin(d.supply_pin),
                    return_pin: pin(d.return_pin),
                    dist_mm: r3(d.dist),
                })
                .collect(),
            shared_fanouts: self
                .shares
                .iter()
                .map(|s| NamedShare {
                    net: net(s.net),
                    leader: pin(s.leader),
                    others: s.others.iter().map(|&p| pin(p)).collect(),
                    kind: s.kind.to_string(),
                })
                .collect(),
            buses: self
                .buses
                .iter()
                .map(|b| NamedBus {
                    a: b.a.clone(),
                    b: b.b.clone(),
                    nets: b.nets.iter().map(|&n| net(n)).collect(),
                    ordered: b.ordered,
                })
                .collect(),
        }
    }

    /// One line per kind, for `gummiband info` and logs.
    pub fn summary(&self, board: &Board) -> Vec<String> {
        let mut lines = Vec::new();
        let mut per_fp: BTreeMap<&str, (usize, usize, f64)> = BTreeMap::new();
        for r in &self.rows {
            let e = per_fp.entry(r.footprint.as_str()).or_insert((0, 0, r.pitch));
            e.0 += 1;
            e.1 += r.pins.len();
        }
        let fine = per_fp.values().filter(|v| v.2 <= 0.65).count();
        lines.push(format!(
            "pin rows: {} in {} parts ({} at 0.65 mm pitch or less)",
            self.rows.len(),
            per_fp.len(),
            fine
        ));
        for g in &self.grids {
            lines.push(format!(
                "pad grid: {} {} x {} at {:.2} mm, {} pins",
                g.footprint,
                g.rows,
                g.cols,
                g.pitch,
                g.pins.len()
            ));
        }
        lines.push(format!("decoupling capacitors: {}", self.decaps.len()));
        let vias_saved: usize = self.shares.iter().map(|s| s.others.len()).sum();
        let by_kind = |k: &str| self.shares.iter().filter(|s| s.kind == k).count();
        lines.push(format!(
            "shared plane fan-outs: {} groups ({} of IC pin and capacitor, {} of neighbours), {} fan-outs fewer",
            self.shares.len(),
            by_kind("decap"),
            by_kind("neighbours"),
            vias_saved
        ));
        for b in &self.buses {
            lines.push(format!(
                "bus: {} - {}, {} nets{}",
                b.a,
                b.b,
                b.nets.len(),
                if b.ordered { "" } else { ", order changes" }
            ));
        }
        let _ = board;
        lines
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{LayerSet, Net, NetClass, Pin};
    use crate::geom::pt;
    use crate::route::{GroupOrder, RouterConfig};

    #[test]
    fn group_order_hints() {
        let order = |arg: &str| RouterConfig::default().with_hints(&hints(arg).unwrap()).groups;
        assert_eq!(order("none"), GroupOrder::Off);
        assert_eq!(order("decaps"), GroupOrder::Off);
        assert_eq!(order("groups-long"), GroupOrder::Long);
        assert_eq!(order("pairs,groups-short"), GroupOrder::Short);
        assert_eq!(order("all,groups-inline"), GroupOrder::Inline);
        assert!(hints("groups-short,groups-inline").is_err());
        assert!(hints("groups-sideways").is_err());
        // buses implies groups, long unless a variant says otherwise; not part of all yet
        assert_eq!(order("buses"), GroupOrder::Long);
        assert_eq!(order("buses,groups-short"), GroupOrder::Short);
        assert!(!hints("all").unwrap().contains(&"buses"));
        assert!(RouterConfig::default().with_hints(&hints("buses").unwrap()).bus_pull > 0.0);
    }

    fn pin(board: &mut Board, fp: &str, num: usize, net: NetId, x: f64, y: f64) {
        board.pins.push(Pin {
            name: format!("{fp}.{num}"),
            footprint: fp.to_string(),
            net,
            pos: pt(x, y),
            layers: LayerSet(1),
            obstacles: Vec::new(),
            size: (0.25, 0.8),
        });
        let i = board.pins.len() - 1;
        board.nets[net as usize].pins.push(i);
    }

    fn board(nets: usize) -> Board {
        let mut b = Board::default();
        b.classes.push(NetClass::default());
        for i in 0..nets {
            b.nets.push(Net {
                name: format!("N{i}"),
                class: 0,
                pins: Vec::new(),
                planes: LayerSet(0),
                priority: 0,
            });
        }
        b
    }

    #[test]
    fn a_qfn_has_four_rows_and_no_grid() {
        let mut b = board(16);
        for k in 0..4 {
            pin(&mut b, "U1", k, k as u32, -1.5, -0.75 + 0.5 * k as f64);
            pin(&mut b, "U1", 4 + k, 4 + k as u32, 1.5, -0.75 + 0.5 * k as f64);
            pin(&mut b, "U1", 8 + k, 8 + k as u32, -0.75 + 0.5 * k as f64, -1.5);
            pin(&mut b, "U1", 12 + k, 12 + k as u32, -0.75 + 0.5 * k as f64, 1.5);
        }
        let p = find(&b);
        assert_eq!(p.rows.len(), 4);
        assert!(p.grids.is_empty());
        let left = p.rows.iter().find(|r| b.pins[r.pins[0]].pos.x < -1.0).unwrap();
        assert!(left.out.x < -0.99, "the left row faces left: {:?}", left.out);
        assert!((left.pitch - 0.5).abs() < 1e-9);
    }

    #[test]
    fn a_memory_package_of_two_narrow_blocks_is_a_ball_field() {
        // two blocks of 3 columns and 13 rows, 1.6 mm apart, and four support balls
        let mut b = board(1);
        let mut k = 0;
        for x in [-4.0, -3.2, -2.4, 2.4, 3.2, 4.0] {
            for j in 0..13 {
                pin(&mut b, "U1", k, 0, x, -4.8 + 0.8 * j as f64);
                k += 1;
            }
        }
        for (x, y) in [(-4.8, -4.8), (4.8, -4.8), (-4.8, 4.8), (4.8, 4.8)] {
            pin(&mut b, "U1", k, 0, x, y);
            k += 1;
        }
        assert!(find(&b).grids.is_empty(), "rows of 3 are no pin rows");
        let g = grids(&b);
        assert_eq!(g.len(), 1);
        assert_eq!(g[0].pins.len(), 78, "the support balls are off the lattice");
        assert!((g[0].pitch - 0.8).abs() < 1e-9);
        assert_eq!((g[0].rows.min(g[0].cols), g[0].rows.max(g[0].cols)), (6, 13));
    }

    #[test]
    fn a_qfn_is_no_ball_field() {
        let mut b = board(1);
        for k in 0..4 {
            pin(&mut b, "U1", k, 0, -1.5, -0.75 + 0.5 * k as f64);
            pin(&mut b, "U1", 4 + k, 0, 1.5, -0.75 + 0.5 * k as f64);
            pin(&mut b, "U1", 8 + k, 0, -0.75 + 0.5 * k as f64, -1.5);
            pin(&mut b, "U1", 12 + k, 0, -0.75 + 0.5 * k as f64, 1.5);
        }
        assert!(grids(&b).is_empty());
    }

    #[test]
    fn a_ball_field_is_a_grid() {
        let mut b = board(16);
        for i in 0..4 {
            for j in 0..4 {
                pin(
                    &mut b,
                    "U1",
                    i * 4 + j,
                    (i * 4 + j) as u32,
                    i as f64 * 0.8,
                    j as f64 * 0.8,
                );
            }
        }
        let p = find(&b);
        assert_eq!(p.grids.len(), 1);
        assert_eq!((p.grids[0].rows, p.grids[0].cols), (4, 4));
        assert!(p.rows.is_empty());
    }

    #[test]
    fn two_connector_rows_make_an_ordered_bus() {
        let mut b = board(5);
        for k in 0..5 {
            pin(&mut b, "J1", k, k as u32, 0.0, k as f64 * 0.5);
            pin(&mut b, "J2", k, k as u32, 10.0, k as f64 * 0.5);
        }
        let p = find(&b);
        assert_eq!(p.buses.len(), 1);
        assert_eq!(p.buses[0].nets.len(), 5);
        assert!(p.buses[0].ordered);
    }
}
