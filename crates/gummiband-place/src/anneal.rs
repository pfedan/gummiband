//! Placement of the movable units: simulated annealing on weighted half perimeter wire length,
//! decoupling distances and courtyard overlap (soft), then legalization by a spiral search and a
//! low temperature polish that only accepts overlap free moves.

use gummiband_core::geom::{Point, pt};
use gummiband_kicad::place_write::kicad_rot;

use crate::model::{Design, R};
use crate::variant::{Outline, Quad, Unit};

/// Deterministic random numbers (splitmix64).
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(seed ^ 0x2545_f491_4f6c_dd1d)
    }
    pub fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    pub fn f(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
}

/// Blocked cells (outside the board with margin, fixed courtyards with gap) and their prefix sums.
pub struct Raster {
    ox: f64,
    oy: f64,
    h: f64,
    nx: usize,
    ny: usize,
    sum: Vec<u32>,
}

impl Raster {
    pub fn new(outline: &Outline, fixed: &[Quad], margin: f64, gap: f64, h: f64) -> Raster {
        let half_diag = h * std::f64::consts::FRAC_1_SQRT_2 + 1e-6;
        let inside = |c: Point| outline.inside(c, margin + half_diag);
        Raster::over(outline.bbox(), &inside, fixed, gap, h)
    }

    /// Raster over `bbox`: a cell is free if `inside(centre)` holds and no fixed courtyard with
    /// `gap` touches it. `inside` has to account for the cell size itself.
    pub fn over(bbox: R, inside: &dyn Fn(Point) -> bool, fixed: &[Quad], gap: f64, h: f64) -> Raster {
        let b = bbox.grow(1.0);
        let nx = (b.w() / h).ceil() as usize;
        let ny = (b.h() / h).ceil() as usize;
        let half_diag = h * std::f64::consts::FRAC_1_SQRT_2 + 1e-6;
        let fb: Vec<R> = fixed.iter().map(|q| q.bbox().grow(gap + h)).collect();
        let mut sum = vec![0u32; (nx + 1) * (ny + 1)];
        for j in 0..ny {
            let mut row = 0u32;
            for i in 0..nx {
                let c = pt(b.x0 + (i as f64 + 0.5) * h, b.y0 + (j as f64 + 0.5) * h);
                let mut blocked = !inside(c);
                if !blocked {
                    for (q, bb) in fixed.iter().zip(&fb) {
                        if c.x >= bb.x0
                            && c.x <= bb.x1
                            && c.y >= bb.y0
                            && c.y <= bb.y1
                            && q.contains(c, gap + half_diag)
                        {
                            blocked = true;
                            break;
                        }
                    }
                }
                row += blocked as u32;
                sum[(j + 1) * (nx + 1) + i + 1] = sum[j * (nx + 1) + i + 1] + row;
            }
        }
        Raster {
            ox: b.x0,
            oy: b.y0,
            h,
            nx,
            ny,
            sum,
        }
    }

    /// Blocked area under `r` (cells touched by `r` count fully). Area outside the grid counts
    /// as blocked, weighted by its distance so that a part far outside is pulled back.
    pub fn blocked(&self, r: &R) -> f64 {
        let gx1 = self.ox + self.nx as f64 * self.h;
        let gy1 = self.oy + self.ny as f64 * self.h;
        let cx0 = r.x0.max(self.ox);
        let cy0 = r.y0.max(self.oy);
        let cx1 = r.x1.min(gx1);
        let cy1 = r.y1.min(gy1);
        let mut out = r.area();
        let mut blocked = 0.0;
        if cx1 > cx0 && cy1 > cy0 {
            out -= (cx1 - cx0) * (cy1 - cy0);
            let i0 = ((cx0 - self.ox) / self.h).floor() as usize;
            let j0 = ((cy0 - self.oy) / self.h).floor() as usize;
            let i1 = (((cx1 - self.ox) / self.h).ceil() as usize).min(self.nx);
            let j1 = (((cy1 - self.oy) / self.h).ceil() as usize).min(self.ny);
            let w = self.nx + 1;
            let s = self.sum[j1 * w + i1] + self.sum[j0 * w + i0] - self.sum[j0 * w + i1] - self.sum[j1 * w + i0];
            blocked = s as f64 * self.h * self.h;
        }
        if out > 0.0 {
            let c = r.center();
            let dx = (self.ox - c.x).max(c.x - gx1).max(0.0);
            let dy = (self.oy - c.y).max(c.y - gy1).max(0.0);
            out *= 1.0 + dx + dy;
        }
        blocked + out
    }
}

/// Cost per mm around the board (a distance raster): pins of a net with a partial plane want to
/// sit over that plane, so that a via reaches it.
#[derive(Clone)]
pub struct Field {
    pub ox: f64,
    pub oy: f64,
    pub h: f64,
    pub nx: usize,
    pub ny: usize,
    pub v: Vec<f32>,
}

impl Field {
    pub fn at(&self, p: Point) -> f64 {
        let i = (((p.x - self.ox) / self.h).max(0.0) as usize).min(self.nx - 1);
        let j = (((p.y - self.oy) / self.h).max(0.0) as usize).min(self.ny - 1);
        self.v[j * self.nx + i] as f64
    }
}

/// Geometry of a movable unit for each of the four rotations, relative to the unit position.
pub struct UnitGeom {
    pub rects: [Vec<R>; 4],
    pub bbox: [R; 4],
    pub pins: [Vec<Point>; 4],
    pub pin_net: Vec<usize>,
    /// Plane field of each pin, see [`Field`].
    pub pin_field: Vec<Option<usize>>,
    /// Escape field of each pin: where it is above 0 the pin has no room for its via, which
    /// counts as overlap ([`Problem::escape_overlap`]).
    pub pin_escape: Vec<Option<usize>>,
    pub area: f64,
    pub nets: Vec<usize>,
    pub decaps: Vec<usize>,
}

pub struct NetTerm {
    pub weight: f64,
    pub pins: Vec<(usize, usize)>,
    pub fixed: Vec<Point>,
    /// Spanning tree length instead of the half perimeter: for current paths, where every pin
    /// inside the box still costs copper.
    pub mst: bool,
}

/// Nets with more pins than this keep the half perimeter even with `mst`.
const MST_PINS: usize = 24;

/// Decoupling term: pin of a unit to a pin of another unit (or a fixed point).
pub struct Decap {
    pub a: (usize, usize),
    pub b: Result<(usize, usize), Point>,
    pub weight: f64,
}

pub struct Problem {
    pub units: Vec<UnitGeom>,
    pub nets: Vec<NetTerm>,
    pub decaps: Vec<Decap>,
    pub raster: Raster,
    pub half_gap: f64,
    pub swappable: Vec<usize>,
    pub fields: Vec<Field>,
    pub field_weight: f64,
    /// Overlap (mm2) a pin without room for its via adds to its unit; 0 leaves it to the cost.
    pub escape_overlap: f64,
}

impl Problem {
    /// `units`: the movable units with the base rotation of rotation index 0 in `unit.rot`.
    /// `pin_terms(unit, member, pad)` gives the net of a pad (None = ignored).
    pub fn new(
        d: &Design,
        units: &[Unit],
        net_weight: &dyn Fn(usize) -> f64,
        fixed_pins: &[(usize, Point)],
        decap_pairs: &[((usize, usize), Result<(usize, usize), Point>, f64)],
        raster: Raster,
        gap: f64,
    ) -> (Problem, Vec<Vec<(usize, usize)>>) {
        let mut geoms = Vec::new();
        // pad index map: (unit, member, pad) -> unit pin index
        let mut part_pins: Vec<Vec<(usize, usize)>> = vec![Vec::new(); d.parts.len()];
        for (ui, u) in units.iter().enumerate() {
            let mut rects: [Vec<R>; 4] = Default::default();
            let mut pins: [Vec<Point>; 4] = Default::default();
            let mut pin_net = Vec::new();
            for k in 0..4 {
                let mut t = u.clone();
                t.pos = pt(0.0, 0.0);
                t.rot = u.rot + 90.0 * k as f64;
                for m in &t.members {
                    let (pos, rot) = t.member_pose(m);
                    let p = &d.parts[m.part];
                    for r in &p.court {
                        rects[k].push(r.transformed_bbox(pos, rot));
                    }
                    for pad in &p.pads {
                        pins[k].push(pos + kicad_rot(pad.local, rot));
                        if k == 0 {
                            pin_net.push(pad.net.unwrap_or(usize::MAX));
                            part_pins[m.part].push((ui, pins[0].len() - 1));
                        }
                    }
                }
            }
            let bbox = [0, 1, 2, 3].map(|k| rects[k].iter().fold(R::empty(), |a, r| a.union(r)));
            let area = rects[0].iter().map(|r| r.area()).sum();
            let pin_field = vec![None; pin_net.len()];
            let pin_escape = vec![None; pin_net.len()];
            geoms.push(UnitGeom {
                rects,
                bbox,
                pins,
                pin_net,
                pin_field,
                pin_escape,
                area,
                nets: Vec::new(),
                decaps: Vec::new(),
            });
        }
        let mut nets: Vec<NetTerm> = (0..d.nets.len())
            .map(|n| NetTerm {
                weight: net_weight(n),
                pins: Vec::new(),
                fixed: Vec::new(),
                mst: false,
            })
            .collect();
        for (ui, g) in geoms.iter().enumerate() {
            for (pi, &n) in g.pin_net.iter().enumerate() {
                if n != usize::MAX {
                    nets[n].pins.push((ui, pi));
                }
            }
        }
        for &(n, p) in fixed_pins {
            nets[n].fixed.push(p);
        }
        for (ni, n) in nets.iter_mut().enumerate() {
            if n.pins.len() + n.fixed.len() < 2 || n.pins.is_empty() {
                n.weight = 0.0;
            }
            let _ = ni;
        }
        for (ni, n) in nets.iter().enumerate() {
            if n.weight == 0.0 {
                continue;
            }
            for &(u, _) in &n.pins {
                if !geoms[u].nets.contains(&ni) {
                    geoms[u].nets.push(ni);
                }
            }
        }
        let mut decaps = Vec::new();
        for ((pa, pada), b, w) in decap_pairs {
            let Some(a) = part_pins[*pa].get(*pada).copied() else {
                continue;
            };
            match b {
                Ok((pb, padb)) => {
                    if let Some(bb) = part_pins[*pb].get(*padb).copied()
                        && bb.0 != a.0
                    {
                        decaps.push(Decap {
                            a,
                            b: Ok(bb),
                            weight: *w,
                        });
                    }
                }
                Err(q) => decaps.push(Decap {
                    a,
                    b: Err(*q),
                    weight: *w,
                }),
            }
        }
        for (di, dc) in decaps.iter().enumerate() {
            geoms[dc.a.0].decaps.push(di);
            if let Ok(b) = dc.b {
                geoms[b.0].decaps.push(di);
            }
        }
        let swappable = (0..geoms.len()).filter(|&u| units[u].members.len() == 1).collect();
        (
            Problem {
                units: geoms,
                nets,
                decaps,
                raster,
                half_gap: gap / 2.0,
                swappable,
                fields: Vec::new(),
                field_weight: 0.0,
                escape_overlap: 0.0,
            },
            part_pins,
        )
    }
}

/// Length of the minimum spanning tree (Prim), Manhattan distances like the half perimeter.
fn mst_len(q: &[Point]) -> f64 {
    let n = q.len();
    if n < 2 {
        return 0.0;
    }
    let mut dist = [f64::INFINITY; MST_PINS];
    let mut done = [false; MST_PINS];
    let mut cur = 0;
    done[0] = true;
    let mut total = 0.0;
    for _ in 1..n {
        let mut best = (f64::INFINITY, 0);
        for k in 0..n {
            if done[k] {
                continue;
            }
            let e = (q[k].x - q[cur].x).abs() + (q[k].y - q[cur].y).abs();
            if e < dist[k] {
                dist[k] = e;
            }
            if dist[k] < best.0 {
                best = (dist[k], k);
            }
        }
        total += best.0;
        cur = best.1;
        done[cur] = true;
    }
    total
}

#[derive(Clone)]
pub struct State {
    pub pos: Vec<Point>,
    pub k: Vec<usize>,
}

pub struct Placer<'a> {
    pub p: &'a Problem,
    pub s: State,
    net_cost: Vec<f64>,
}

impl<'a> Placer<'a> {
    pub fn new(p: &'a Problem, s: State) -> Placer<'a> {
        let mut pl = Placer {
            p,
            s,
            net_cost: vec![0.0; p.nets.len()],
        };
        for n in 0..p.nets.len() {
            pl.net_cost[n] = pl.hpwl(n);
        }
        pl
    }

    fn pin(&self, u: usize, i: usize) -> Point {
        self.s.pos[u] + self.p.units[u].pins[self.s.k[u]][i]
    }

    fn hpwl(&self, n: usize) -> f64 {
        let t = &self.p.nets[n];
        if t.weight == 0.0 {
            return 0.0;
        }
        let count = t.pins.len() + t.fixed.len();
        if t.mst && count <= MST_PINS {
            let mut q = [pt(0.0, 0.0); MST_PINS];
            for (k, &(u, i)) in t.pins.iter().enumerate() {
                q[k] = self.pin(u, i);
            }
            q[t.pins.len()..count].copy_from_slice(&t.fixed);
            return t.weight * mst_len(&q[..count]);
        }
        let mut b = R::empty();
        let mut add = |q: Point| {
            b.x0 = b.x0.min(q.x);
            b.y0 = b.y0.min(q.y);
            b.x1 = b.x1.max(q.x);
            b.y1 = b.y1.max(q.y);
        };
        for &(u, i) in &t.pins {
            add(self.pin(u, i));
        }
        for &q in &t.fixed {
            add(q);
        }
        t.weight * (b.w() + b.h())
    }

    fn decap(&self, di: usize) -> f64 {
        let dc = &self.p.decaps[di];
        let a = self.pin(dc.a.0, dc.a.1);
        let b = match dc.b {
            Ok((u, i)) => self.pin(u, i),
            Err(q) => q,
        };
        dc.weight * a.dist(b)
    }

    fn field_cost(&self, u: usize) -> f64 {
        let g = &self.p.units[u];
        let mut s = 0.0;
        for (i, f) in g.pin_field.iter().enumerate() {
            if let Some(f) = f {
                s += self.p.fields[*f].at(self.pin(u, i));
            }
        }
        s * self.p.field_weight
    }

    pub fn rects(&self, u: usize) -> impl Iterator<Item = R> + '_ {
        let o = self.s.pos[u];
        self.p.units[u].rects[self.s.k[u]].iter().map(move |r| r.shift(o))
    }

    fn bbox(&self, u: usize) -> R {
        self.p.units[u].bbox[self.s.k[u]].shift(self.s.pos[u])
    }

    pub fn pair_overlap(&self, u: usize, v: usize) -> f64 {
        let g = self.p.half_gap;
        if self.bbox(u).grow(g).overlap(&self.bbox(v).grow(g)) == 0.0 {
            return 0.0;
        }
        let mut s = 0.0;
        for a in self.rects(u) {
            let a = a.grow(g);
            for b in self.rects(v) {
                s += a.overlap(&b.grow(g));
            }
        }
        s
    }

    pub fn fixed_overlap(&self, u: usize) -> f64 {
        self.rects(u).map(|r| self.p.raster.blocked(&r)).sum::<f64>() + self.escape_blocked(u)
    }

    /// Overlap for the pins of `u` that find no room for their via.
    fn escape_blocked(&self, u: usize) -> f64 {
        if self.p.escape_overlap == 0.0 {
            return 0.0;
        }
        let g = &self.p.units[u];
        let blocked = g
            .pin_escape
            .iter()
            .enumerate()
            .filter(|(i, f)| f.is_some_and(|f| self.p.fields[f].at(self.pin(u, *i)) > 0.0))
            .count();
        blocked as f64 * self.p.escape_overlap
    }

    /// Overlap of `u` with everything else.
    pub fn overlap(&self, u: usize) -> f64 {
        let mut s = self.fixed_overlap(u);
        for v in 0..self.p.units.len() {
            if v != u {
                s += self.pair_overlap(u, v);
            }
        }
        s
    }

    pub fn wire(&self) -> f64 {
        self.net_cost.iter().sum::<f64>()
            + (0..self.p.decaps.len()).map(|d| self.decap(d)).sum::<f64>()
            + (0..self.p.units.len()).map(|u| self.field_cost(u)).sum::<f64>()
    }

    pub fn total_overlap(&self) -> f64 {
        let n = self.p.units.len();
        let mut s = 0.0;
        for u in 0..n {
            s += self.fixed_overlap(u);
            for v in u + 1..n {
                s += self.pair_overlap(u, v);
            }
        }
        s
    }

    fn local_cost(&self, us: &[usize], nets: &[usize], decs: &[usize], lambda: f64) -> (f64, f64) {
        let w: f64 = nets.iter().map(|&n| self.hpwl(n)).sum::<f64>()
            + decs.iter().map(|&d| self.decap(d)).sum::<f64>()
            + us.iter().map(|&u| self.field_cost(u)).sum::<f64>();
        let mut ov = 0.0;
        if lambda > 0.0 {
            for &u in us {
                ov += self.overlap(u);
            }
            if us.len() == 2 {
                ov -= self.pair_overlap(us[0], us[1]);
            }
        }
        (w, ov)
    }

    fn affected(&self, us: &[usize]) -> (Vec<usize>, Vec<usize>) {
        let mut nets: Vec<usize> = Vec::new();
        let mut decs: Vec<usize> = Vec::new();
        for &u in us {
            for &n in &self.p.units[u].nets {
                if !nets.contains(&n) {
                    nets.push(n);
                }
            }
            for &d in &self.p.units[u].decaps {
                if !decs.contains(&d) {
                    decs.push(d);
                }
            }
        }
        (nets, decs)
    }

    /// One annealing run. `hard`: reject every move that leaves an overlap on a moved unit.
    pub fn anneal(
        &mut self,
        rng: &mut Rng,
        iters: usize,
        t0: f64,
        t1: f64,
        l0: f64,
        l1: f64,
        r0: f64,
        r1: f64,
        hard: bool,
    ) {
        let n = self.p.units.len();
        if n == 0 {
            return;
        }
        for it in 0..iters {
            let f = it as f64 / iters as f64;
            let t = t0 * (t1 / t0).powf(f);
            let lambda = if hard { 0.0 } else { l0 * (l1 / l0).powf(f) };
            let rad = r0 * (r1 / r0).powf(f);
            let mv = rng.f();
            let u = rng.below(n);
            let us: Vec<usize>;
            let old: Vec<(Point, usize)>;
            if mv < 0.1 && self.p.swappable.len() > 1 {
                let a = self.p.swappable[rng.below(self.p.swappable.len())];
                let b = self.p.swappable[rng.below(self.p.swappable.len())];
                if a == b {
                    continue;
                }
                us = vec![a, b];
            } else {
                us = vec![u];
            }
            old = us.iter().map(|&u| (self.s.pos[u], self.s.k[u])).collect();
            let (nets, decs) = self.affected(&us);
            let (w0, o0) = self.local_cost(&us, &nets, &decs, lambda);
            if us.len() == 2 {
                // swap centres of the bounding boxes
                let c0 = self.bbox(us[0]).center();
                let c1 = self.bbox(us[1]).center();
                self.s.pos[us[0]] = self.s.pos[us[0]] + (c1 - c0);
                self.s.pos[us[1]] = self.s.pos[us[1]] + (c0 - c1);
            } else if mv < 0.22 {
                let c0 = self.bbox(u).center();
                self.s.k[u] = (self.s.k[u] + 1 + rng.below(3)) % 4;
                let c1 = self.bbox(u).center();
                self.s.pos[u] = self.s.pos[u] + (c0 - c1);
            } else {
                let dx = (rng.f() * 2.0 - 1.0) * rad;
                let dy = (rng.f() * 2.0 - 1.0) * rad;
                self.s.pos[u] = self.s.pos[u] + pt(dx, dy);
            }
            let (w1, o1) = self.local_cost(&us, &nets, &decs, lambda);
            let mut reject = false;
            if hard {
                for &v in &us {
                    if self.overlap(v) > 1e-9 {
                        reject = true;
                    }
                }
            }
            let delta = (w1 - w0) + lambda * (o1 - o0);
            if !reject && (delta <= 0.0 || rng.f() < (-delta / t).exp()) {
                for &n in &nets {
                    self.net_cost[n] = self.hpwl(n);
                }
            } else {
                for (&v, &(p, k)) in us.iter().zip(&old) {
                    self.s.pos[v] = p;
                    self.s.k[v] = k;
                }
            }
        }
    }

    /// Overlap of `u` with the fixed parts and the units marked in `placed`.
    fn overlap_with(&self, u: usize, placed: &[bool]) -> f64 {
        let mut s = self.fixed_overlap(u);
        for v in 0..self.p.units.len() {
            if v != u && placed[v] {
                s += self.pair_overlap(u, v);
            }
        }
        s
    }

    /// Legalize biggest units first: each unit only has to avoid the fixed parts and the units
    /// legalized before it; if it overlaps them it moves to the nearest free spot (spiral search
    /// over position and rotation, wire length as tie breaker). Smaller units it lands on are
    /// moved when their turn comes. Returns the units that found no spot, biggest first.
    pub fn legalize(&mut self, step: f64, max_r: f64) -> Vec<usize> {
        let n = self.p.units.len();
        let mut order: Vec<usize> = (0..n).collect();
        order.sort_by(|&a, &b| self.p.units[b].area.partial_cmp(&self.p.units[a].area).unwrap());
        let mut placed = vec![false; n];
        let mut failed = Vec::new();
        for &u in &order {
            if self.overlap_with(u, &placed) <= 1e-9 {
                placed[u] = true;
                continue;
            }
            let (nets, decs) = self.affected(&[u]);
            let home = self.s.pos[u];
            let home_k = self.s.k[u];
            let mut best: Option<(f64, Point, usize)> = None;
            let mut found_at = f64::INFINITY;
            let mut ring = 0usize;
            loop {
                let d = ring as f64 * step;
                if d > max_r || d > found_at + 1.5 {
                    break;
                }
                let m = ring as isize;
                let mut cands: Vec<Point> = Vec::new();
                if m == 0 {
                    cands.push(home);
                } else {
                    for i in -m..=m {
                        cands.push(home + pt(i as f64 * step, -m as f64 * step));
                        cands.push(home + pt(i as f64 * step, m as f64 * step));
                    }
                    for i in -m + 1..m {
                        cands.push(home + pt(-m as f64 * step, i as f64 * step));
                        cands.push(home + pt(m as f64 * step, i as f64 * step));
                    }
                }
                for c in cands {
                    for k in 0..4 {
                        self.s.pos[u] = c;
                        self.s.k[u] = k;
                        if self.overlap_with(u, &placed) > 1e-9 {
                            continue;
                        }
                        let w: f64 = nets.iter().map(|&n| self.hpwl(n)).sum::<f64>()
                            + decs.iter().map(|&d| self.decap(d)).sum::<f64>()
                            + self.field_cost(u);
                        let cost = w + 2.0 * c.dist(home);
                        if best.is_none_or(|b| cost < b.0) {
                            best = Some((cost, c, k));
                        }
                        found_at = found_at.min(d);
                    }
                }
                ring += 1;
            }
            match best {
                Some((_, c, k)) => {
                    self.s.pos[u] = c;
                    self.s.k[u] = k;
                }
                None => {
                    self.s.pos[u] = home;
                    self.s.k[u] = home_k;
                    failed.push(u);
                }
            }
            placed[u] = true;
            for &n in &nets {
                self.net_cost[n] = self.hpwl(n);
            }
        }
        failed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spanning_tree_of_a_chain() {
        // a chain in order: the tree follows it, the box would be 2 + 2
        let q = [pt(0.0, 0.0), pt(1.0, 0.0), pt(1.0, 1.0), pt(2.0, 1.0), pt(2.0, 2.0)];
        assert!((mst_len(&q) - 4.0).abs() < 1e-9);
        // a detour costs the tree, not the box
        let q = [pt(0.0, 0.0), pt(2.0, 0.0), pt(1.0, 0.0), pt(1.0, 1.9)];
        assert!((mst_len(&q) - 3.9).abs() < 1e-9);
        assert_eq!(mst_len(&[pt(3.0, 3.0)]), 0.0);
    }
}
