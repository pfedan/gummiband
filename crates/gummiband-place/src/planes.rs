//! Split power plane for the new outline: every cell of a raster goes to the net of the nearest
//! power pin (Voronoi), each net keeps one connected region without holes, neighbouring regions
//! keep a gap, and the regions are traced into rectilinear polygons.

use std::collections::VecDeque;

use gummiband_core::geom::{Point, Polygon, pt};

use crate::variant::Outline;

pub struct PlaneSplit {
    /// Per net (same order as the input): outline polygon, or None if the net got no region.
    pub regions: Vec<Option<Vec<Point>>>,
    pub notes: Vec<String>,
}

const NONE: u16 = u16::MAX;
const INVALID: u16 = u16::MAX - 1;

struct Grid {
    ox: f64,
    oy: f64,
    h: f64,
    nx: usize,
    ny: usize,
    c: Vec<u16>,
}

impl Grid {
    fn idx(&self, i: usize, j: usize) -> usize {
        j * self.nx + i
    }
    fn center(&self, i: usize, j: usize) -> Point {
        pt(self.ox + (i as f64 + 0.5) * self.h, self.oy + (j as f64 + 0.5) * self.h)
    }
    fn cell_of(&self, p: Point) -> Option<(usize, usize)> {
        let i = ((p.x - self.ox) / self.h).floor();
        let j = ((p.y - self.oy) / self.h).floor();
        if i < 0.0 || j < 0.0 || i >= self.nx as f64 || j >= self.ny as f64 {
            None
        } else {
            Some((i as usize, j as usize))
        }
    }
    fn n4(&self, i: usize, j: usize) -> impl Iterator<Item = (usize, usize)> + '_ {
        let (nx, ny) = (self.nx as isize, self.ny as isize);
        [(-1isize, 0isize), (1, 0), (0, -1), (0, 1)].into_iter().filter_map(move |(di, dj)| {
            let (a, b) = (i as isize + di, j as isize + dj);
            (a >= 0 && b >= 0 && a < nx && b < ny).then_some((a as usize, b as usize))
        })
    }

    /// 4-connected components of the cells of `net`: list of cell lists.
    fn components(&self, net: u16) -> Vec<Vec<usize>> {
        let mut seen = vec![false; self.c.len()];
        let mut out = Vec::new();
        for j in 0..self.ny {
            for i in 0..self.nx {
                let s = self.idx(i, j);
                if self.c[s] != net || seen[s] {
                    continue;
                }
                let mut comp = Vec::new();
                let mut q = VecDeque::from([(i, j)]);
                seen[s] = true;
                while let Some((a, b)) = q.pop_front() {
                    comp.push(self.idx(a, b));
                    for (x, y) in self.n4(a, b) {
                        let t = self.idx(x, y);
                        if self.c[t] == net && !seen[t] {
                            seen[t] = true;
                            q.push_back((x, y));
                        }
                    }
                }
                out.push(comp);
            }
        }
        out
    }

    /// Cells not in `net` that are enclosed by it (not 4-reachable from the grid border).
    fn holes(&self, net: u16) -> Vec<usize> {
        let mut reach = vec![false; self.c.len()];
        let mut q = VecDeque::new();
        for j in 0..self.ny {
            for i in 0..self.nx {
                if (i == 0 || j == 0 || i == self.nx - 1 || j == self.ny - 1) && self.c[self.idx(i, j)] != net {
                    reach[self.idx(i, j)] = true;
                    q.push_back((i, j));
                }
            }
        }
        while let Some((a, b)) = q.pop_front() {
            for (x, y) in self.n4(a, b) {
                let t = self.idx(x, y);
                if self.c[t] != net && !reach[t] {
                    reach[t] = true;
                    q.push_back((x, y));
                }
            }
        }
        (0..self.c.len()).filter(|&k| self.c[k] != net && !reach[k]).collect()
    }

    /// Boundary of a hole free 4-connected region as a polygon (collinear points merged).
    fn trace(&self, net: u16) -> Vec<Point> {
        let inside = |i: isize, j: isize| -> bool {
            i >= 0 && j >= 0 && (i as usize) < self.nx && (j as usize) < self.ny && self.c[self.idx(i as usize, j as usize)] == net
        };
        // directed edges on the cell corner lattice, region on the left (y down: clockwise on screen)
        let w = self.nx + 1;
        let mut next: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
        let v = |i: usize, j: usize| j * w + i;
        let mut start = None;
        for j in 0..self.ny as isize {
            for i in 0..self.nx as isize {
                if !inside(i, j) {
                    continue;
                }
                let (iu, ju) = (i as usize, j as usize);
                if !inside(i, j - 1) {
                    next.insert(v(iu + 1, ju), v(iu, ju));
                }
                if !inside(i - 1, j) {
                    next.insert(v(iu, ju), v(iu, ju + 1));
                }
                if !inside(i, j + 1) {
                    next.insert(v(iu, ju + 1), v(iu + 1, ju + 1));
                }
                if !inside(i + 1, j) {
                    next.insert(v(iu + 1, ju + 1), v(iu + 1, ju));
                }
                if start.is_none() {
                    start = Some(v(iu + 1, ju));
                }
            }
        }
        let Some(s) = start else { return vec![] };
        let mut loop_pts = vec![s];
        let mut cur = s;
        while let Some(&n) = next.get(&cur) {
            if n == s {
                break;
            }
            loop_pts.push(n);
            cur = n;
            if loop_pts.len() > next.len() + 1 {
                break;
            }
        }
        let p = |k: usize| pt(self.ox + (k % w) as f64 * self.h, self.oy + (k / w) as f64 * self.h);
        let raw: Vec<Point> = loop_pts.into_iter().map(p).collect();
        let n = raw.len();
        let mut out = Vec::new();
        for i in 0..n {
            let a = raw[(i + n - 1) % n];
            let b = raw[i];
            let c = raw[(i + 1) % n];
            if ((b - a).cross(c - b)).abs() > 1e-9 {
                out.push(b);
            }
        }
        out
    }
}

/// `pins[n]`: pin positions of plane net n. `keepouts`: polygons where the plane must not go.
pub fn split(outline: &Outline, pins: &[Vec<Point>], keepouts: &[Polygon], edge: f64, gap_cells: usize, h: f64) -> PlaneSplit {
    let b = outline.bbox();
    let nx = (b.w() / h).ceil() as usize;
    let ny = (b.h() / h).ceil() as usize;
    let mut g = Grid {
        ox: b.x0,
        oy: b.y0,
        h,
        nx,
        ny,
        c: vec![INVALID; nx * ny],
    };
    let mut notes = Vec::new();
    let half_diag = h * std::f64::consts::FRAC_1_SQRT_2;
    let nn = pins.len();
    for j in 0..ny {
        for i in 0..nx {
            let c = g.center(i, j);
            if !outline.inside(c, edge + half_diag) {
                continue;
            }
            if keepouts.iter().any(|k| k.contains(c) || k.boundary_dist(c) < 0.5 + half_diag) {
                continue;
            }
            let mut best = (f64::INFINITY, NONE);
            for (n, ps) in pins.iter().enumerate() {
                for p in ps {
                    let d = p.dist(c);
                    if d < best.0 {
                        best = (d, n as u16);
                    }
                }
            }
            let k = g.idx(i, j);
            g.c[k] = best.1;
        }
    }
    let seeds = |g: &Grid, n: usize, comp: &[usize]| -> usize {
        pins[n]
            .iter()
            .filter(|p| g.cell_of(**p).is_some_and(|(i, j)| comp.binary_search(&g.idx(i, j)).is_ok()))
            .count()
    };
    let keep_main = |g: &mut Grid, notes: &mut Vec<String>, what: &str| {
        for n in 0..nn {
            let mut comps = g.components(n as u16);
            if comps.len() <= 1 {
                continue;
            }
            for c in comps.iter_mut() {
                c.sort_unstable();
            }
            let best = (0..comps.len())
                .max_by_key(|&k| (seeds(g, n, &comps[k]), comps[k].len()))
                .unwrap();
            let mut dropped = 0;
            for (k, c) in comps.iter().enumerate() {
                if k != best {
                    dropped += seeds(g, n, c);
                    for &cell in c {
                        g.c[cell] = NONE;
                    }
                }
            }
            if dropped > 0 {
                notes.push(format!("net {n}: {dropped} pins outside its main region ({what})"));
            }
        }
    };
    keep_main(&mut g, &mut notes, "Voronoi");
    // regrow the freed cells from the main regions
    let regrow = |g: &mut Grid| {
        let mut q = VecDeque::new();
        for j in 0..ny {
            for i in 0..nx {
                let k = g.idx(i, j);
                if g.c[k] < INVALID - 1 + 1 && g.c[k] != NONE && g.c[k] != INVALID {
                    q.push_back((i, j));
                }
            }
        }
        while let Some((a, bb)) = q.pop_front() {
            let net = g.c[g.idx(a, bb)];
            let ns: Vec<(usize, usize)> = g.n4(a, bb).collect();
            for (x, y) in ns {
                let t = g.idx(x, y);
                if g.c[t] == NONE {
                    g.c[t] = net;
                    q.push_back((x, y));
                }
            }
        }
    };
    regrow(&mut g);
    // holes: a region enclosed by another one gets a corridor to the outside through the
    // enclosing region (which stays connected, like a C); tiny enclosed bits are merged
    for _ in 0..2 * nn + 2 {
        let mut changed = false;
        for n in 0..nn {
            let hs = g.holes(n as u16);
            if hs.is_empty() {
                continue;
            }
            let mut in_hole = vec![false; g.c.len()];
            for &k in &hs {
                in_hole[k] = true;
            }
            let mut enclosed: Vec<u16> = Vec::new();
            for &k in &hs {
                let o = g.c[k];
                if (o as usize) < nn && !enclosed.contains(&o) {
                    enclosed.push(o);
                }
            }
            let mut carved = false;
            for &o in &enclosed {
                let cells = hs.iter().filter(|&&k| g.c[k] == o).count();
                if (cells as f64) * h * h < 20.0 {
                    continue;
                }
                // shortest path from region o across n to a cell outside the enclosure
                let mut parent = vec![usize::MAX; g.c.len()];
                let mut q = VecDeque::new();
                for &k in &hs {
                    if g.c[k] == o {
                        parent[k] = k;
                        q.push_back(k);
                    }
                }
                let mut goal = None;
                while let Some(k) = q.pop_front() {
                    let (i, j) = (k % nx, k / nx);
                    if g.c[k] != n as u16 && !in_hole[k] {
                        goal = Some(k);
                        break;
                    }
                    if i == 0 || j == 0 || i == nx - 1 || j == ny - 1 {
                        goal = Some(k);
                        break;
                    }
                    let ns: Vec<(usize, usize)> = g.n4(i, j).collect();
                    for (x, y) in ns {
                        let t = g.idx(x, y);
                        if parent[t] == usize::MAX && (g.c[t] == n as u16 || in_hole[t] || g.c[t] != o) {
                            parent[t] = k;
                            q.push_back(t);
                        }
                    }
                }
                let Some(mut k) = goal else { continue };
                let w = 3isize;
                while parent[k] != k {
                    let (i, j) = ((k % nx) as isize, (k / nx) as isize);
                    for dj in -w..=w {
                        for di in -w..=w {
                            let (x, y) = (i + di, j + dj);
                            if x >= 0 && y >= 0 && x < nx as isize && y < ny as isize {
                                let t = g.idx(x as usize, y as usize);
                                if g.c[t] == n as u16 {
                                    g.c[t] = o;
                                }
                            }
                        }
                    }
                    k = parent[k];
                }
                notes.push(format!("net {o}: enclosed by net {n}, corridor to the outside"));
                carved = true;
                changed = true;
            }
            if carved {
                continue; // recompute the holes of n next round
            }
            let mut lost: Vec<u16> = Vec::new();
            for k in hs {
                let o = g.c[k];
                if o < nn as u16 && !lost.contains(&o) {
                    lost.push(o);
                }
                g.c[k] = n as u16;
                changed = true;
            }
            for o in lost {
                notes.push(format!("net {o}: small region enclosed by net {n}, merged into it"));
            }
        }
        keep_main(&mut g, &mut notes, "after the corridors");
        regrow(&mut g);
        if !changed {
            break;
        }
    }
    // gap: the bigger of two neighbouring regions gives way
    let mut area = vec![0usize; nn];
    for &c in &g.c {
        if (c as usize) < nn {
            area[c as usize] += 1;
        }
    }
    let r = gap_cells as isize;
    // a region that the gap would split is protected: its neighbours give way instead
    let mut protected = vec![false; nn];
    let base = g.c.clone();
    for _ in 0..=nn {
        g.c = base.clone();
        let gives_way = |a: u16, b: u16| -> bool {
            let (pa, pb) = (protected[a as usize], protected[b as usize]);
            if pa != pb {
                return pb;
            }
            (area[a as usize], a) > (area[b as usize], b)
        };
        let mut clear = Vec::new();
        for j in 0..ny as isize {
            for i in 0..nx as isize {
                let a = g.c[g.idx(i as usize, j as usize)];
                if a as usize >= nn {
                    continue;
                }
                'n: for dj in -r..=r {
                    for di in -r..=r {
                        let (x, y) = (i + di, j + dj);
                        if x < 0 || y < 0 || x >= nx as isize || y >= ny as isize {
                            continue;
                        }
                        let b2 = g.c[g.idx(x as usize, y as usize)];
                        if (b2 as usize) < nn && b2 != a && gives_way(a, b2) {
                            clear.push(g.idx(i as usize, j as usize));
                            break 'n;
                        }
                    }
                }
            }
        }
        for k in clear {
            g.c[k] = NONE;
        }
        let mut split_any = false;
        for n in 0..nn {
            if !protected[n] && g.components(n as u16).len() > 1 {
                protected[n] = true;
                split_any = true;
            }
        }
        if !split_any {
            break;
        }
    }
    keep_main(&mut g, &mut notes, "after the gaps");
    let mut regions = Vec::new();
    for n in 0..nn {
        let cells = g.c.iter().filter(|&&c| c == n as u16).count();
        if (cells as f64) * h * h < 4.0 {
            if !pins[n].is_empty() {
                notes.push(format!("net {n}: no region"));
            }
            regions.push(None);
            continue;
        }
        regions.push(Some(g.trace(n as u16)));
    }
    PlaneSplit { regions, notes }
}
