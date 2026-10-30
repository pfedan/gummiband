//! Placement of a board from its netlist. Locked and mechanical parts (see
//! `gummiband_kicad::ReadOptions::move_parts`) stay; every other part gets a position from the
//! netlist, the outline, the planes and the fixed parts alone. Where a movable part sits in the
//! input does not matter, so a pile of freshly imported footprints works as well as a placed board.

use std::collections::HashMap;
use std::path::Path;

use anyhow::{Context, Result, bail};
use gummiband_core::board::{Board, LayerSet};
use gummiband_core::geom::{Point, Polygon, pt};
use gummiband_kicad::place_write::{self, Patch, kicad_rot};
use gummiband_kicad::sexpr::Node;
use gummiband_kicad::{KicadBoard, ReadOptions, read_board};
use rayon::prelude::*;

use crate::anneal::{Field, Placer, Problem, Raster, Rng, State};
use crate::model::{Design, R};
use crate::variant::{Quad, Unit};

/// Pins of a net with a partial plane stay this far inside it, so that a via fits.
const PLANE_INSET: f64 = 0.4;
/// Cost per mm of a pin away from the plane of its net.
const PLANE_WEIGHT: f64 = 1.5;
/// Grid of the written positions.
const GRID: f64 = 0.05;
/// Annealing seeds per placement seed, more than anyone starts.
const SEED_STEP: u64 = 1000;

/// Which parts get a new position. Locked footprints stay in every case.
#[derive(Debug, Clone, Default)]
pub enum Scope {
    /// Everything except connectors, modules, mounting holes, buttons and test points.
    #[default]
    Unlocked,
    /// Every part that is not locked, mechanical ones too.
    AllUnlocked,
    /// Parts that sit outside the board outline.
    Outside,
    /// Just these references.
    Only(Vec<String>),
}

#[derive(Debug, Clone)]
pub struct PlaceOptions {
    pub scope: Scope,
    /// References that stay, whatever the scope says.
    pub fixed: Vec<String>,
    /// References released on top of the scope (a test point, a button).
    pub free: Vec<String>,
    /// Another seed gives another set of starts, none shared with a neighbouring seed.
    pub seed: u64,
    /// Independent annealing runs, the best one is kept.
    pub starts: usize,
    /// Annealing moves of one run.
    pub iters: usize,
    /// Spacing between courtyards in mm.
    pub gap: f64,
    /// Courtyards keep this distance from the outline.
    pub margin: f64,
    /// Extra room around the courtyards of parts with five or more pads, for their escape.
    pub halo: f64,
    pub plane_layers: Vec<String>,
}

impl Default for PlaceOptions {
    fn default() -> Self {
        PlaceOptions {
            scope: Scope::Unlocked,
            fixed: Vec::new(),
            free: Vec::new(),
            seed: 1,
            starts: 4,
            iters: 3_000_000,
            gap: 0.3,
            margin: 0.5,
            halo: 0.3,
            plane_layers: Vec::new(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct PlaceReport {
    pub fixed: Vec<String>,
    pub moved: Vec<String>,
    /// Parts that stayed because kept copper ends on one of their pads.
    pub anchored: Vec<String>,
    /// New pose of every moved part: reference, x, y, rotation in degrees.
    pub poses: Vec<(String, f64, f64, f64)>,
    /// New outline of every keepout that followed its part: zone id, points.
    pub zones: Vec<(String, Vec<Point>)>,
    /// Decoupling capacitors with the part they were assigned to.
    pub decoupling: Vec<(String, String)>,
    /// Pins left outside the plane of their net: reached by routing.
    pub off_plane: Vec<String>,
    /// Tracks and vias in the input: they do not follow the parts.
    pub stale_copper: usize,
    pub seed: u64,
    /// The start that won, counted from 1.
    pub start: usize,
    /// Weighted wire length of the placement.
    pub wire: f64,
    /// Crossings between the spanning trees of the nets, a measure of the vias to expect.
    pub crossings: usize,
    /// Result of every start: its number, wire, crossings; a start without a legal placement is left out.
    pub starts: Vec<(usize, f64, usize)>,
    pub free_area: f64,
    pub movable_area: f64,
    pub min_separation: f64,
}

pub struct Placed {
    pub text: String,
    pub report: PlaceReport,
}

/// Parts that stay, as flags per part of `d`.
fn fixed_parts(
    d: &Design,
    kb: &KicadBoard,
    opts: &PlaceOptions,
    outer: &Polygon,
) -> Result<(Vec<bool>, Vec<usize>)> {
    let only: &[String] = match &opts.scope {
        Scope::Only(refs) => refs,
        _ => &[],
    };
    for r in opts.fixed.iter().chain(&opts.free).chain(only) {
        if !d.by_ref.contains_key(r) {
            bail!("no footprint {r} on the board");
        }
    }
    let outside = |p: &crate::model::Part| -> bool {
        let b = p
            .court
            .iter()
            .fold(R::empty(), |b, r| b.union(&r.transformed_bbox(p.pos, p.rot)));
        !outer.contains(b.center())
    };
    let mut fixed: Vec<bool> = d
        .parts
        .iter()
        .map(|p| {
            let in_scope = match &opts.scope {
                Scope::Unlocked => kb.board.movable.contains(&p.reference),
                Scope::AllUnlocked => true,
                Scope::Outside => outside(p),
                Scope::Only(refs) => refs.contains(&p.reference),
            };
            let chosen = in_scope || opts.free.contains(&p.reference);
            !chosen || p.locked || opts.fixed.contains(&p.reference) || p.court.is_empty()
        })
        .collect();
    // copper that ends on a pad keeps the part where it is: tracks do not follow
    let anchored = anchored_parts(d, &kb.board, &fixed);
    for &i in &anchored {
        fixed[i] = true;
    }
    // a block with its copper stays, the writer cannot turn tracks yet
    for b in &kb.board.blocks {
        if !b.uuids.is_empty() {
            for r in &b.refs {
                if let Some(&i) = d.by_ref.get(r) {
                    fixed[i] = true;
                }
            }
        }
    }
    // a block moves as a whole or not at all
    for b in &kb.board.blocks {
        let ids: Vec<usize> = b.refs.iter().filter_map(|r| d.by_ref.get(r).copied()).collect();
        if ids.iter().any(|&i| fixed[i]) {
            ids.iter().for_each(|&i| fixed[i] = true);
        }
    }
    Ok((fixed, anchored))
}

/// Parts with kept copper of their net on a pad, among those not fixed yet.
fn anchored_parts(d: &Design, board: &Board, fixed: &[bool]) -> Vec<usize> {
    let on_pad = |pin: &gummiband_core::board::Pin| -> bool {
        let reach = pin.size.0.max(pin.size.1) * 0.5;
        board.copper.iter().filter(|c| c.net == Some(pin.net)).any(|c| {
            board.obstacles[c.first as usize..c.last as usize]
                .iter()
                .any(|o| o.shape.dist_to_point(pin.pos) <= reach)
        })
    };
    (0..d.parts.len())
        .filter(|&i| !fixed[i])
        .filter(|&i| {
            board
                .pins
                .iter()
                .any(|p| p.footprint == d.parts[i].reference && on_pad(p))
        })
        .collect()
}

/// Kept copper on the layers of the moving parts, to look up by position.
struct CopperGrid<'a> {
    board: &'a Board,
    bbox: R,
    cols: usize,
    cells: Vec<Vec<usize>>,
    reach: f64,
}

impl<'a> CopperGrid<'a> {
    const CELL: f64 = 2.0;

    fn new(board: &'a Board, d: &Design, fixed: &[bool], bbox: R, reach: f64) -> CopperGrid<'a> {
        let mut layers = LayerSet::default();
        for p in &board.pins {
            if d.by_ref.get(&p.footprint).is_some_and(|&i| !fixed[i]) {
                for l in p.layers.iter() {
                    layers.insert(l);
                }
            }
        }
        let cols = (bbox.w() / Self::CELL).ceil() as usize + 1;
        let rows = (bbox.h() / Self::CELL).ceil() as usize + 1;
        let mut cells = vec![Vec::new(); cols * rows];
        let column = |x: f64| (((x - bbox.x0) / Self::CELL).max(0.0) as usize).min(cols - 1);
        let row = |y: f64| (((y - bbox.y0) / Self::CELL).max(0.0) as usize).min(rows - 1);
        for c in &board.copper {
            for o in c.first as usize..c.last as usize {
                let ob = &board.obstacles[o];
                if !ob.layers.intersects(layers) {
                    continue;
                }
                let b = ob.shape.bbox().grow(reach);
                for j in row(b.min.y)..=row(b.max.y) {
                    for i in column(b.min.x)..=column(b.max.x) {
                        cells[j * cols + i].push(o);
                    }
                }
            }
        }
        CopperGrid {
            board,
            bbox,
            cols,
            cells,
            reach,
        }
    }

    fn near(&self, p: Point) -> bool {
        if p.x < self.bbox.x0 || p.y < self.bbox.y0 || p.x > self.bbox.x1 || p.y > self.bbox.y1 {
            return false;
        }
        let i = ((p.x - self.bbox.x0) / Self::CELL) as usize;
        let j = ((p.y - self.bbox.y0) / Self::CELL) as usize;
        self.cells[j * self.cols + i]
            .iter()
            .any(|&o| self.board.obstacles[o].shape.dist_to_point(p) < self.reach)
    }
}

fn movable_units(d: &Design, kb: &KicadBoard, fixed: &[bool]) -> Vec<Unit> {
    let mut taken = fixed.to_vec();
    let mut units = Vec::new();
    for b in &kb.board.blocks {
        let ps: Vec<usize> = b.refs.iter().filter_map(|r| d.by_ref.get(r).copied()).collect();
        if !ps.is_empty() && ps.iter().all(|&p| !taken[p]) {
            for &p in &ps {
                taken[p] = true;
            }
            units.push(Unit::rigid(d, &b.name, &ps));
        }
    }
    for i in 0..d.parts.len() {
        if !taken[i] {
            let mut u = Unit::single(d, i);
            // the rotation is searched, not inherited
            u.members[0].rel_rot = 0.0;
            units.push(u);
        }
    }
    units
}

fn is_ground(name: &str) -> bool {
    let n = name.rsplit('/').next().unwrap_or(name).to_ascii_uppercase();
    n.contains("GND") || n == "VSS"
}

/// Capacitance in farad from a value such as `100n`, `4.7uF` or `22u/25V`.
fn farad(value: &str) -> Option<f64> {
    let end = value.find(|c: char| !c.is_ascii_digit() && c != '.')?;
    let v: f64 = value[..end].parse().ok()?;
    let unit = value[end..].chars().next()?;
    let scale = match unit {
        'p' | 'P' => 1e-12,
        'n' | 'N' => 1e-9,
        'u' | 'U' | 'µ' => 1e-6,
        'm' => 1e-3,
        _ => return None,
    };
    Some(v * scale)
}

/// Groups of pads of `net` on an IC: pads next to each other count as one supply pin, with the
/// number of pads it stands for.
fn supply_pins(d: &Design, net: usize) -> Vec<(usize, usize, usize)> {
    let mut out: Vec<(usize, usize, usize)> = Vec::new();
    for (i, p) in d.parts.iter().enumerate() {
        let connector = p.footprint.contains("Connector") || p.reference.starts_with('J');
        if p.pads.len() < 4 || connector {
            continue;
        }
        let first = out.len();
        for (k, pad) in p.pads.iter().enumerate() {
            if pad.net != Some(net) {
                continue;
            }
            match out[first..]
                .iter_mut()
                .find(|g| p.pads[g.1].local.dist(pad.local) <= 2.0)
            {
                Some(g) => g.2 += 1,
                None => out.push((i, k, 1)),
            }
        }
    }
    out
}

/// Pads of `net` on inductors: bulk capacitors of a converter output belong there.
fn inductor_pins(d: &Design, net: usize) -> Vec<(usize, usize, usize)> {
    let mut out = Vec::new();
    for (i, p) in d.parts.iter().enumerate() {
        if p.reference.starts_with('L')
            && p.pads.len() == 2
            && let Some(k) = p.pads.iter().position(|x| x.net == Some(net))
        {
            out.push((i, k, 1));
        }
    }
    out
}

/// Each decoupling capacitor goes to one supply pin of an IC on its net, bulk capacitors to an
/// inductor on the net if there is one. On a net the smallest capacitors are dealt out first and a
/// pin's share follows the pads it stands for, so a power stage gets more than a logic chip and
/// every pin has its fast capacitor before one gets a second.
/// The result: capacitor pad, supply pin, weight.
fn decoupling(d: &Design, fixed: &[bool]) -> Vec<((usize, usize), (usize, usize), f64)> {
    let mut by_net: Vec<Vec<(f64, usize, usize)>> = vec![Vec::new(); d.nets.len()];
    for (i, c) in d.parts.iter().enumerate() {
        if fixed[i] || !c.reference.starts_with('C') || c.pads.len() != 2 {
            continue;
        }
        let nets = [c.pads[0].net, c.pads[1].net];
        let Some(g) = nets.iter().position(|n| n.is_some_and(|n| is_ground(&d.nets[n]))) else {
            continue;
        };
        if let Some(n) = nets[1 - g]
            && !is_ground(&d.nets[n])
        {
            by_net[n].push((farad(&c.value).unwrap_or(1e-7), i, 1 - g));
        }
    }
    let mut out = Vec::new();
    for (n, caps) in by_net.iter_mut().enumerate() {
        caps.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let pins = supply_pins(d, n);
        let coils = inductor_pins(d, n);
        let mut dealt = vec![0usize; pins.len()];
        let mut coil_dealt = vec![0usize; coils.len()];
        for &(f, c, pad) in caps.iter() {
            let bulk = f > 1.1e-6;
            let (set, count) = if bulk && !coils.is_empty() {
                (&coils, &mut coil_dealt)
            } else {
                (&pins, &mut dealt)
            };
            if set.is_empty() {
                continue;
            }
            let k = (0..set.len())
                .min_by(|&a, &b| {
                    let share = |i: usize| (count[i] + 1) as f64 / set[i].2 as f64;
                    share(a).partial_cmp(&share(b)).unwrap()
                })
                .unwrap();
            count[k] += 1;
            out.push(((c, pad), (set[k].0, set[k].1), if bulk { 2.5 } else { 3.0 }));
        }
    }
    out
}

/// Distance raster of a partial plane: 0 well inside it, growing outside.
fn plane_field(polys: &[&Polygon], b: &R) -> Field {
    let h = 0.5;
    let nx = (b.w() / h).ceil() as usize + 1;
    let ny = (b.h() / h).ceil() as usize + 1;
    let v: Vec<f32> = (0..nx * ny)
        .into_par_iter()
        .map(|c| {
            let p = pt(b.x0 + ((c % nx) as f64 + 0.5) * h, b.y0 + ((c / nx) as f64 + 0.5) * h);
            let mut best = f64::INFINITY;
            for poly in polys {
                let e = poly.boundary_dist(p);
                let cost = if poly.contains(p) {
                    (PLANE_INSET - e).max(0.0)
                } else {
                    PLANE_INSET + e
                };
                best = best.min(cost);
            }
            best as f32
        })
        .collect();
    Field {
        ox: b.x0,
        oy: b.y0,
        h,
        nx,
        ny,
        v,
    }
}

struct Nets {
    /// Net weight per net of the design.
    weight: Vec<f64>,
    /// Polygons of the partial plane of a net, indexed by net of the design.
    partial: Vec<Vec<Polygon>>,
}

/// Nets with a plane over most of the board cost nothing; a partial plane carries its net itself,
/// so that its pins only have to sit over it. Wide and differential nets count double.
fn net_terms(d: &Design, board: &Board, board_area: f64) -> Nets {
    let mut weight = vec![1.0; d.nets.len()];
    let mut partial: Vec<Vec<Polygon>> = vec![Vec::new(); d.nets.len()];
    let pads_on = |n: usize| -> usize {
        d.parts
            .iter()
            .map(|p| p.pads.iter().filter(|x| x.net == Some(n)).count())
            .sum()
    };
    for (n, name) in d.nets.iter().enumerate() {
        if name.starts_with("unconnected-") {
            weight[n] = 0.0;
            continue;
        }
        let Some((id, net)) = board.nets.iter().enumerate().find(|(_, x)| &x.name == name) else {
            continue;
        };
        let polys: Vec<&Polygon> = board
            .planes
            .iter()
            .filter(|p| p.net as usize == id)
            .map(|p| &p.outline)
            .collect();
        let area: f64 = polys.iter().map(|p| p.signed_area().abs()).sum();
        if area >= 0.8 * board_area {
            weight[n] = 0.0;
        } else if !polys.is_empty() {
            weight[n] = 0.3;
            partial[n] = polys.into_iter().cloned().collect();
        } else {
            let c = &board.classes[net.class as usize];
            let coil = d
                .parts
                .iter()
                .any(|p| p.reference.starts_with('L') && p.pads.iter().any(|x| x.net == Some(n)));
            weight[n] = if coil {
                // the switch node of a converter: as short as it gets
                8.0
            } else if c.track_width >= 0.5 || c.diff_width > 0.0 {
                2.0
            } else if pads_on(n) >= 8 {
                0.6
            } else {
                1.0
            };
        }
    }
    Nets { weight, partial }
}

/// Crossings between the spanning trees of the nets (pads in their final position).
fn crossings(d: &Design, pose: &[(Point, f64)], weight: &[f64]) -> usize {
    let mut pins: Vec<Vec<Point>> = vec![Vec::new(); d.nets.len()];
    for (i, p) in d.parts.iter().enumerate() {
        for pad in &p.pads {
            if let Some(n) = pad.net {
                pins[n].push(pose[i].0 + kicad_rot(pad.local, pose[i].1));
            }
        }
    }
    let mut segs: Vec<(usize, Point, Point)> = Vec::new();
    for (n, ps) in pins.iter().enumerate() {
        if weight[n] < 0.5 || ps.len() < 2 {
            continue;
        }
        let mut done = vec![false; ps.len()];
        let mut dist: Vec<(f64, usize)> = ps.iter().map(|q| (q.dist(ps[0]), 0)).collect();
        done[0] = true;
        for _ in 1..ps.len() {
            let (next, _) = (0..ps.len())
                .filter(|&k| !done[k])
                .map(|k| (k, dist[k].0))
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                .unwrap();
            done[next] = true;
            segs.push((n, ps[dist[next].1], ps[next]));
            for k in 0..ps.len() {
                let e = ps[k].dist(ps[next]);
                if !done[k] && e < dist[k].0 {
                    dist[k] = (e, next);
                }
            }
        }
    }
    let mut count = 0;
    for (i, a) in segs.iter().enumerate() {
        for b in &segs[i + 1..] {
            if a.0 != b.0 && gummiband_core::geom::segs_intersect(a.1, a.2, b.1, b.2) {
                count += 1;
            }
        }
    }
    count
}

fn unit_poses(d: &Design, units: &[Unit], s: &State) -> Vec<(Point, f64)> {
    let mut pose: Vec<(Point, f64)> = d.parts.iter().map(|p| (p.pos, p.rot)).collect();
    for (ui, u) in units.iter().enumerate() {
        let mut t = u.clone();
        t.pos = pt((s.pos[ui].x / GRID).round() * GRID, (s.pos[ui].y / GRID).round() * GRID);
        t.rot = u.rot + 90.0 * s.k[ui] as f64;
        for m in &t.members {
            let (p, r) = t.member_pose(m);
            pose[m.part] = (p, place_write::norm_angle(r));
        }
    }
    pose
}

/// One annealing run from random positions; None if it finds no legal placement.
fn run(p: &Problem, bbox: &R, seed: u64, iters: usize) -> Option<(State, f64)> {
    let mut rng = Rng::new(seed);
    let n = p.units.len();
    let pos = (0..n)
        .map(|_| pt(bbox.x0 + rng.f() * bbox.w(), bbox.y0 + rng.f() * bbox.h()))
        .collect();
    let mut pl = Placer::new(p, State { pos, k: vec![0; n] });
    let size = bbox.w().max(bbox.h());
    pl.anneal(&mut rng, iters, 30.0, 0.05, 0.3, 400.0, size / 3.0, 0.2, false);
    if pl.legalize(0.25, 40.0) > 0 || pl.total_overlap() > 1e-6 {
        return None;
    }
    pl.anneal(&mut rng, iters / 3, 0.8, 0.01, 0.0, 0.0, 2.0, 0.1, true);
    let wire = pl.wire();
    Some((pl.s, wire))
}

fn zone_points(z: &Node) -> Vec<Point> {
    z.child("polygon")
        .and_then(|p| p.child("pts"))
        .map(|p| {
            p.children("xy")
                .filter_map(|n| {
                    let a = n.args();
                    Some(pt(a.first()?.as_f64()?, a.get(1)?.as_f64()?))
                })
                .collect()
        })
        .unwrap_or_default()
}

/// A keepout that no part may sit on.
fn bars_parts(z: &Node) -> bool {
    z.child("keepout")
        .is_some_and(|k| k.child_str("footprints") == Some("not_allowed") || k.child_str("pads") == Some("not_allowed"))
}

fn bbox_of(pts: &[Point]) -> R {
    pts.iter().fold(R::empty(), |b, p| {
        b.union(&R {
            x0: p.x,
            y0: p.y,
            x1: p.x,
            y1: p.y,
        })
    })
}

/// The movable part a root keepout belongs to: the one whose courtyard covers most of it.
fn keepout_owner(pts: &[Point], d: &Design, fixed: &[bool]) -> Option<usize> {
    let zb = bbox_of(pts);
    d.parts
        .iter()
        .enumerate()
        .filter(|(i, _)| !fixed[*i])
        .flat_map(|(i, p)| {
            p.court.iter().map(move |r| {
                let q = Quad {
                    pos: p.pos,
                    rot: p.rot,
                    r: *r,
                };
                (i, q.bbox().overlap(&zb))
            })
        })
        .filter(|x| x.1 >= 0.5 * zb.area())
        .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .map(|x| x.0)
}

/// Place the board in the file at `path`. The file text comes back with the footprints moved; the
/// zones of the moved parts follow them, filled copper is left to KiCad's next refill.
pub fn place_board(path: &Path, opts: &PlaceOptions) -> Result<Placed> {
    let kb = read_board(
        path,
        &ReadOptions {
            move_parts: true,
            fixed: opts.fixed.clone(),
            plane_layers: opts.plane_layers.clone(),
            ..Default::default()
        },
    )?;
    let mut d = Design::from_root(&kb.root);
    let outlines = {
        let mut v: Vec<&Polygon> = kb.board.outline.iter().collect();
        v.sort_by(|a, b| b.signed_area().abs().partial_cmp(&a.signed_area().abs()).unwrap());
        v
    };
    let (outer, cutouts) = outlines.split_first().context("the board has no outline")?;
    let (fixed, anchored) = fixed_parts(&d, &kb, opts, outer)?;
    for p in &mut d.parts {
        if p.pads.len() >= 5 {
            for r in &mut p.court {
                *r = r.grow(opts.halo);
            }
        }
    }
    let units = movable_units(&d, &kb, &fixed);
    if units.is_empty() {
        bail!("nothing to place: all parts are fixed");
    }

    let bb = outer.bbox();
    let bbox = R {
        x0: bb.min.x,
        y0: bb.min.y,
        x1: bb.max.x,
        y1: bb.max.y,
    };

    // ---- what the parts have to avoid
    let mut barred: Vec<Polygon> = Vec::new();
    let mut followers: Vec<(&Node, usize)> = Vec::new();
    for n in kb.root.args() {
        if n.head() == Some("zone") && n.child("keepout").is_some() {
            let pts = zone_points(n);
            if pts.len() < 3 {
                continue;
            }
            if let Some(i) = keepout_owner(&pts, &d, &fixed) {
                followers.push((n, i));
            } else if bars_parts(n) {
                barred.push(Polygon::new(pts));
            }
        }
        if matches!(n.head(), Some("footprint") | Some("module")) {
            let at = d.parts.iter().position(|p| std::ptr::eq(&kb.root.args()[p.node], n));
            if at.is_some_and(|i| fixed[i]) {
                for z in n.children("zone").filter(|z| bars_parts(z)) {
                    let pts = zone_points(z);
                    if pts.len() >= 3 {
                        barred.push(Polygon::new(pts));
                    }
                }
            }
        }
    }
    let fixed_quads: Vec<Quad> = d
        .parts
        .iter()
        .enumerate()
        .filter(|(i, _)| fixed[*i])
        .flat_map(|(_, p)| {
            p.court.iter().map(|r| Quad {
                pos: p.pos,
                rot: p.rot,
                r: *r,
            })
        })
        .collect();
    let cell = 0.1;
    let half_diag = cell * std::f64::consts::FRAC_1_SQRT_2 + 1e-6;
    let edge = opts.margin + half_diag;
    let copper = CopperGrid::new(&kb.board, &d, &fixed, bbox, opts.gap + half_diag);
    let inside = |c: Point| -> bool {
        outer.contains(c)
            && outer.boundary_dist(c) >= edge
            && !cutouts.iter().any(|h| h.contains(c) || h.boundary_dist(c) < edge)
            && !barred.iter().any(|z| z.contains(c) || z.boundary_dist(c) < half_diag)
            && !copper.near(c)
    };
    let raster = Raster::over(bbox, &inside, &fixed_quads, opts.gap, cell);
    let free_area = bbox.area() - raster.blocked(&bbox);
    let movable_area: f64 = units
        .iter()
        .flat_map(|u| &u.members)
        .map(|m| d.parts[m.part].court_area())
        .sum();

    // ---- nets
    let terms = net_terms(&d, &kb.board, outer.signed_area().abs());
    let mut fixed_pins = Vec::new();
    for (i, p) in d.parts.iter().enumerate() {
        if fixed[i] {
            for pad in &p.pads {
                if let Some(n) = pad.net {
                    fixed_pins.push((n, p.pad_world(pad)));
                }
            }
        }
    }
    let pairs = decoupling(&d, &fixed);
    let decap_pairs: Vec<_> = pairs
        .iter()
        .map(|&(c, (ic, pad), w)| {
            let target = if fixed[ic] {
                Err(d.parts[ic].pad_world(&d.parts[ic].pads[pad]))
            } else {
                Ok((ic, pad))
            };
            (c, target, w)
        })
        .collect();
    let net_weight = |n: usize| terms.weight[n];
    let (mut prob, part_pins) = Problem::new(&d, &units, &net_weight, &fixed_pins, &decap_pairs, raster, opts.gap);

    // ---- partial planes
    let mut field_of: HashMap<usize, usize> = HashMap::new();
    for (n, polys) in terms.partial.iter().enumerate() {
        if !polys.is_empty() {
            field_of.insert(n, prob.fields.len());
            prob.fields.push(plane_field(&polys.iter().collect::<Vec<_>>(), &bbox));
        }
    }
    prob.field_weight = PLANE_WEIGHT;
    for (i, p) in d.parts.iter().enumerate() {
        for (k, pad) in p.pads.iter().enumerate() {
            if let (Some(n), Some(&(u, pin))) = (pad.net, part_pins[i].get(k))
                && let Some(&f) = field_of.get(&n)
            {
                prob.units[u].pin_field[pin] = Some(f);
            }
        }
    }

    // ---- the starts, in parallel; the fewest crossings win, then the shortest wires
    let starts: Vec<usize> = (1..=opts.starts.max(1)).collect();
    let mut results: Vec<(usize, State, f64, usize)> = starts
        .par_iter()
        .filter_map(|&start| {
            let seed = opts.seed.wrapping_sub(1).wrapping_mul(SEED_STEP) + start as u64;
            let (s, wire) = run(&prob, &bbox, seed, opts.iters)?;
            let c = crossings(&d, &unit_poses(&d, &units, &s), &terms.weight);
            Some((start, s, wire, c))
        })
        .collect();
    results.sort_by(|a, b| (a.3, a.2).partial_cmp(&(b.3, b.2)).unwrap());
    let starts: Vec<(usize, f64, usize)> = results.iter().map(|r| (r.0, r.2, r.3)).collect();
    let Some((start, state, wire, crossed)) = results.into_iter().next() else {
        bail!(
            "no legal placement: movable courtyards {movable_area:.0} mm2 on {free_area:.0} mm2 free, \
             try a smaller --gap or --halo"
        );
    };
    let pose = unit_poses(&d, &units, &state);

    // ---- exact check on the grid positions
    let quads: Vec<(usize, Quad)> = d
        .parts
        .iter()
        .enumerate()
        .flat_map(|(i, p)| {
            let (pos, rot) = pose[i];
            p.court.iter().map(move |r| (i, Quad { pos, rot, r: *r }))
        })
        .collect();
    let mut min_sep = (f64::INFINITY, String::new());
    for (a, (i, q)) in quads.iter().enumerate() {
        for (j, o) in &quads[a + 1..] {
            if i == j || (fixed[*i] && fixed[*j]) || q.bbox().grow(1.0).overlap(&o.bbox()) == 0.0 {
                continue;
            }
            let s = q.separation(o);
            if s < min_sep.0 {
                min_sep = (s, format!("{}-{}", d.parts[*i].reference, d.parts[*j].reference));
            }
        }
    }
    if min_sep.0 < 0.0 {
        bail!("placement check failed: courtyards {} overlap", min_sep.1);
    }
    for (i, q) in &quads {
        if !fixed[*i] && q.corners().iter().any(|c| !outer.contains(*c)) {
            bail!("placement check failed: {} leaves the board", d.parts[*i].reference);
        }
    }

    // ---- write
    let xf = |i: usize| {
        let (p0, r0) = (d.parts[i].pos, d.parts[i].rot);
        let (p1, r1) = pose[i];
        move |q: Point| p1 + kicad_rot(q - p0, r1 - r0)
    };
    let mut patch = Patch::new(opts.seed ^ 0x706c_6163);
    for (i, p) in d.parts.iter().enumerate() {
        if !fixed[i] {
            patch.move_footprint(&kb.root.args()[p.node], pose[i].0, pose[i].1);
        }
    }
    let mut zones = Vec::new();
    for (z, i) in &followers {
        let f = xf(*i);
        patch.transform_zone(z, &f);
        if let Some(id) = z.child_str("uuid").or_else(|| z.child_str("tstamp")) {
            zones.push((id.to_string(), zone_points(z).into_iter().map(&f).collect()));
        }
    }
    for n in kb.root.args().iter().filter(|n| n.head() == Some("gr_text")) {
        if let Some(at) = n.child_f64s("at")
            && at.len() >= 2
        {
            let c = pt(at[0], at[1]);
            let owner = quads
                .iter()
                .map(|(i, q)| {
                    (
                        *i,
                        Quad {
                            pos: d.parts[*i].pos,
                            rot: d.parts[*i].rot,
                            r: q.r,
                        },
                    )
                })
                .find(|(_, q)| q.contains(c, 0.5));
            if let Some((i, _)) = owner.filter(|x| !fixed[x.0]) {
                let f = xf(i);
                patch.transform_text(n, &f, pose[i].1 - d.parts[i].rot);
            }
        }
    }

    let mut off_plane = Vec::new();
    for (i, p) in d.parts.iter().enumerate().filter(|(i, _)| !fixed[*i]) {
        for pad in &p.pads {
            if let Some(n) = pad.net
                && let Some(&f) = field_of.get(&n)
                && prob.fields[f].at(pose[i].0 + kicad_rot(pad.local, pose[i].1)) > PLANE_INSET
            {
                off_plane.push(format!("{}.{}", p.reference, pad.number));
            }
        }
    }
    off_plane.dedup();
    let names = |want: bool| -> Vec<String> {
        let mut v: Vec<String> = (0..d.parts.len())
            .filter(|&i| fixed[i] == want)
            .map(|i| d.parts[i].reference.clone())
            .collect();
        v.sort();
        v
    };
    let stale_copper = kb
        .root
        .args()
        .iter()
        .filter(|n| matches!(n.head(), Some("segment") | Some("arc") | Some("via")))
        .count();
    Ok(Placed {
        text: patch.apply(&kb.text, &kb.root),
        report: PlaceReport {
            fixed: names(true),
            moved: names(false),
            anchored: anchored.iter().map(|&i| d.parts[i].reference.clone()).collect(),
            poses: (0..d.parts.len())
                .filter(|&i| !fixed[i])
                .map(|i| (d.parts[i].reference.clone(), pose[i].0.x, pose[i].0.y, pose[i].1))
                .collect(),
            zones,
            decoupling: pairs
                .iter()
                .map(|&((c, _), (ic, _), _)| (d.parts[c].reference.clone(), d.parts[ic].reference.clone()))
                .collect(),
            off_plane,
            stale_copper,
            seed: opts.seed,
            start,
            wire,
            crossings: crossed,
            starts,
            free_area,
            movable_area,
            min_separation: min_sep.0,
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fp(lib: &str, r: &str, at: (f64, f64), locked: bool, pads: &[(&str, &str)]) -> String {
        let lock = if locked { "(locked yes)" } else { "" };
        let mut s = format!(
            "\t(footprint \"{lib}\" {lock} (layer \"F.Cu\") (at {} {})\n\
             \t\t(property \"Reference\" \"{r}\" (at 0 0 0) (layer \"F.SilkS\"))\n\
             \t\t(property \"Value\" \"100n\" (at 0 0 0) (layer \"F.Fab\"))\n\
             \t\t(fp_rect (start -1 -1) (end 1 1) (layer \"F.CrtYd\"))\n",
            at.0, at.1
        );
        for (i, (n, net)) in pads.iter().enumerate() {
            s.push_str(&format!(
                "\t\t(pad \"{n}\" smd rect (at {} 0) (size 0.4 0.4) (layers \"F.Cu\") (net \"{net}\"))\n",
                -0.6 + 1.2 * i as f64 / (pads.len() - 1).max(1) as f64
            ));
        }
        s.push_str("\t)\n");
        s
    }

    /// A 30 x 20 mm board: a locked IC, a connector, a free IC and small parts at `spot`.
    fn board(spot: impl Fn(usize) -> (f64, f64)) -> String {
        let mut s = String::from(
            "(kicad_pcb (version 20240108) (generator test)\n\
             \t(layers (0 \"F.Cu\" signal) (31 \"B.Cu\" signal) (44 \"Edge.Cuts\" user))\n\
             \t(setup (pad_to_mask_clearance 0))\n\
             \t(net 0 \"\") (net 1 \"GND\") (net 2 \"VCC\") (net 3 \"A\") (net 4 \"B\")\n\
             \t(gr_rect (start 100 100) (end 130 120) (stroke (width 0.1) (type solid)) (fill none) \
             (layer \"Edge.Cuts\"))\n",
        );
        s += &fp(
            "Lib:IC",
            "U1",
            (105.0, 105.0),
            true,
            &[("1", "A"), ("2", "VCC"), ("3", "GND"), ("4", "B")],
        );
        s += &fp("Connector:X", "J1", (125.0, 115.0), false, &[("1", "A"), ("2", "GND")]);
        s += &fp(
            "Lib:IC",
            "U2",
            spot(0),
            false,
            &[("1", "B"), ("2", "VCC"), ("3", "GND"), ("4", "A")],
        );
        s += &fp("Lib:C", "C1", spot(1), false, &[("1", "VCC"), ("2", "GND")]);
        s += &fp("Lib:C", "C2", spot(2), false, &[("1", "VCC"), ("2", "GND")]);
        s += &fp("Lib:R", "R1", spot(3), false, &[("1", "A"), ("2", "B")]);
        s.push_str(")\n");
        s
    }

    /// A locked track, inserted before the end of the board.
    fn with_track(board: &str, a: (f64, f64), b: (f64, f64), layer: &str, net: &str) -> String {
        let end = board.rfind(')').unwrap();
        format!(
            "{}\t(segment (start {} {}) (end {} {}) (width 0.3) (layer \"{layer}\") (net \"{net}\") \
             (locked yes) (uuid \"00000000-0000-0000-0000-000000000001\"))\n{}",
            &board[..end],
            a.0,
            a.1,
            b.0,
            b.1,
            &board[end..]
        )
    }

    fn place(text: &str, name: &str, opts: &PlaceOptions) -> Result<Placed> {
        let dir = std::env::temp_dir().join(format!("gb-auto-{}-{name}", std::process::id()));
        std::fs::create_dir_all(&dir)?;
        let path = dir.join("t.kicad_pcb");
        std::fs::write(&path, text)?;
        let out = place_board(&path, opts);
        std::fs::remove_dir_all(&dir)?;
        out
    }

    fn quick() -> PlaceOptions {
        PlaceOptions {
            iters: 60_000,
            starts: 2,
            ..Default::default()
        }
    }

    fn pose(r: &PlaceReport, name: &str) -> (f64, f64) {
        let p = r.poses.iter().find(|p| p.0 == name).unwrap();
        (p.1, p.2)
    }

    #[test]
    fn locked_and_mechanical_parts_stay() {
        let p = place(&board(|i| (110.0 + i as f64 * 4.0, 110.0)), "stay", &quick()).unwrap();
        assert_eq!(p.report.fixed, vec!["J1", "U1"]);
        assert_eq!(p.report.moved, vec!["C1", "C2", "R1", "U2"]);
        assert!(p.report.min_separation >= 0.0);
    }

    #[test]
    fn the_input_position_of_a_moving_part_does_not_matter() {
        let a = place(&board(|i| (110.0 + i as f64 * 4.0, 110.0)), "a", &quick()).unwrap();
        // a pile outside the board
        let b = place(&board(|_| (160.0, 160.0)), "b", &quick()).unwrap();
        assert_eq!(a.report.poses, b.report.poses);
    }

    #[test]
    fn capacitors_sit_next_to_the_supply_pin() {
        let p = place(&board(|_| (160.0, 160.0)), "caps", &quick()).unwrap();
        assert_eq!(p.report.decoupling.len(), 2);
        for c in ["C1", "C2"] {
            let (x, y) = pose(&p.report, c);
            let ic = &p.report.decoupling.iter().find(|d| d.0 == c).unwrap().1;
            let (ix, iy) = if ic == "U1" {
                (105.0, 105.0)
            } else {
                pose(&p.report, ic)
            };
            let d = ((x - ix).powi(2) + (y - iy).powi(2)).sqrt();
            assert!(d < 6.0, "{c} is {d} mm from {ic}");
        }
    }

    #[test]
    fn only_the_named_parts_move() {
        let opts = PlaceOptions {
            scope: Scope::Only(vec!["C1".into()]),
            ..quick()
        };
        let p = place(&board(|i| (110.0 + i as f64 * 4.0, 110.0)), "only", &opts).unwrap();
        assert_eq!(p.report.moved, vec!["C1"]);
    }

    #[test]
    fn outside_takes_the_parts_off_the_board() {
        let opts = PlaceOptions {
            scope: Scope::Outside,
            ..quick()
        };
        let text = board(|i| {
            if i < 2 {
                (160.0, 160.0)
            } else {
                (110.0 + i as f64 * 4.0, 110.0)
            }
        });
        let p = place(&text, "outside", &opts).unwrap();
        assert_eq!(p.report.moved, vec!["C1", "U2"]);
        let (x, y) = pose(&p.report, "C1");
        assert!((100.0..130.0).contains(&x) && (100.0..120.0).contains(&y));
    }

    #[test]
    fn a_locked_part_stays_whatever_the_scope_says() {
        let opts = PlaceOptions {
            scope: Scope::AllUnlocked,
            ..quick()
        };
        let p = place(&board(|_| (160.0, 160.0)), "all", &opts).unwrap();
        assert!(p.report.moved.contains(&"J1".to_string()));
        assert_eq!(p.report.fixed, vec!["U1"]);
    }

    #[test]
    fn an_unknown_reference_is_an_error() {
        let opts = PlaceOptions {
            fixed: vec!["X9".into()],
            ..quick()
        };
        assert!(place(&board(|_| (160.0, 160.0)), "unknown", &opts).is_err());
    }

    #[test]
    fn neighbouring_seeds_share_no_start() {
        let spot = |i: usize| (110.0 + i as f64 * 4.0, 110.0);
        let opts = |seed| PlaceOptions { seed, ..quick() };
        let one = place(&board(spot), "seed1", &opts(1)).unwrap();
        let two = place(&board(spot), "seed2", &opts(2)).unwrap();
        assert_eq!(one.report.starts.len(), 2);
        assert_ne!(one.report.poses, two.report.poses);
        assert_eq!(two.report.seed, 2);
    }

    #[test]
    fn parts_stay_off_locked_copper() {
        // a wall across the board, on the parts' side
        let text = with_track(&board(|_| (160.0, 160.0)), (112.0, 100.0), (112.0, 120.0), "F.Cu", "B");
        let p = place(&text, "wall", &quick()).unwrap();
        for (r, x, _, _) in &p.report.poses {
            assert!((x - 112.0).abs() > 1.3, "{r} sits on the track at {x}");
        }
    }

    #[test]
    fn copper_on_the_other_side_does_not_block() {
        let text = with_track(&board(|_| (160.0, 160.0)), (112.0, 100.0), (112.0, 120.0), "B.Cu", "B");
        assert!(place(&text, "other", &quick()).is_ok());
    }

    #[test]
    fn a_part_with_copper_on_its_pad_stays() {
        let text = with_track(&board(|_| (116.0, 110.0)), (115.4, 110.0), (112.0, 112.0), "F.Cu", "B");
        let p = place(&text, "anchor", &quick()).unwrap();
        assert_eq!(p.report.anchored, vec!["U2"]);
        assert!(p.report.fixed.contains(&"U2".to_string()));
    }

    #[test]
    fn capacitance_values() {
        assert_eq!(farad("100n"), Some(100.0 * 1e-9));
        assert_eq!(farad("22uF/25V"), Some(22.0 * 1e-6));
        assert_eq!(farad("10k"), None);
    }
}
