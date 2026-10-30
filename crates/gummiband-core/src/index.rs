//! Uniform grid over bounding boxes for proximity queries.

use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hasher};

use crate::geom::{Point, Rect};

/// Multiplicative hasher for the small integer keys of the inner loops (nothing in this crate
/// depends on the iteration order of a map, and no key comes from outside).
#[derive(Default, Clone, Copy)]
pub struct FastHasher(u64);

impl FastHasher {
    #[inline]
    fn add(&mut self, x: u64) {
        self.0 = (self.0.rotate_left(5) ^ x).wrapping_mul(0x517c_c1b7_2722_0a95);
    }
}

impl Hasher for FastHasher {
    #[inline]
    fn finish(&self) -> u64 {
        self.0.rotate_left(26)
    }
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.add(b as u64);
        }
    }
    #[inline]
    fn write_u8(&mut self, x: u8) {
        self.add(x as u64);
    }
    #[inline]
    fn write_u32(&mut self, x: u32) {
        self.add(x as u64);
    }
    #[inline]
    fn write_u64(&mut self, x: u64) {
        self.add(x);
    }
    #[inline]
    fn write_i32(&mut self, x: i32) {
        self.add(x as u32 as u64);
    }
    #[inline]
    fn write_i64(&mut self, x: i64) {
        self.add(x as u64);
    }
    #[inline]
    fn write_usize(&mut self, x: usize) {
        self.add(x as u64);
    }
}

pub type FastMap<K, V> = HashMap<K, V, BuildHasherDefault<FastHasher>>;

pub struct GridIndex {
    origin: Point,
    cell: f64,
    nx: usize,
    ny: usize,
    cells: Vec<Vec<u32>>,
}

impl GridIndex {
    pub fn new(bounds: Rect, cell: f64) -> GridIndex {
        let b = bounds.grow(cell);
        let nx = ((b.width() / cell).ceil() as usize).max(1);
        let ny = ((b.height() / cell).ceil() as usize).max(1);
        GridIndex {
            origin: b.min,
            cell,
            nx,
            ny,
            cells: vec![Vec::new(); nx * ny],
        }
    }

    fn range(&self, r: &Rect) -> (usize, usize, usize, usize) {
        let f = |v: f64, o: f64, n: usize| (((v - o) / self.cell).floor().max(0.0) as usize).min(n - 1);
        (
            f(r.min.x, self.origin.x, self.nx),
            f(r.max.x, self.origin.x, self.nx),
            f(r.min.y, self.origin.y, self.ny),
            f(r.max.y, self.origin.y, self.ny),
        )
    }

    pub fn insert(&mut self, id: u32, r: &Rect) {
        let (x0, x1, y0, y1) = self.range(r);
        for y in y0..=y1 {
            for x in x0..=x1 {
                self.cells[y * self.nx + x].push(id);
            }
        }
    }

    /// Ids whose boxes may overlap `r` (deduplicated, sorted).
    pub fn query(&self, r: &Rect) -> Vec<u32> {
        let (x0, x1, y0, y1) = self.range(r);
        let mut out = Vec::new();
        for y in y0..=y1 {
            for x in x0..=x1 {
                out.extend_from_slice(&self.cells[y * self.nx + x]);
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }

    pub fn query_point(&self, p: Point, radius: f64) -> Vec<u32> {
        let mut r = Rect::empty();
        r.add(p);
        self.query(&r.grow(radius))
    }
}

/// Static grid of points in compressed rows: every item sits in exactly one cell (the cell of
/// its key point), so a query needs no deduplication. Items with an extent are found by
/// growing the query radius by their largest half size.
pub struct PointGrid {
    origin: Point,
    cell: f64,
    nx: usize,
    ny: usize,
    start: Vec<u32>,
    ids: Vec<u32>,
}

impl PointGrid {
    pub fn new(bounds: Rect, cell: f64, keys: &[Point]) -> PointGrid {
        let b = bounds.grow(cell);
        let nx = ((b.width() / cell).ceil() as usize).max(1);
        let ny = ((b.height() / cell).ceil() as usize).max(1);
        let mut g = PointGrid {
            origin: b.min,
            cell,
            nx,
            ny,
            start: vec![0; nx * ny + 1],
            ids: Vec::new(),
        };
        g.rebuild(keys);
        g
    }

    /// New keys on the same grid, without new allocations once the buffers are large enough.
    pub fn rebuild(&mut self, keys: &[Point]) {
        self.start.fill(0);
        // start[c + 1] counts cell c, the prefix sum then makes it the end of cell c
        for p in keys {
            let c = self.cell_of(*p);
            self.start[c + 1] += 1;
        }
        let mut sum = 0;
        for s in &mut self.start {
            sum += *s;
            *s = sum;
        }
        self.ids.clear();
        self.ids.resize(keys.len(), 0);
        // filling from the back keeps the ids of a cell ascending and leaves start[c] at the
        // begin of cell c
        for (i, p) in keys.iter().enumerate().rev() {
            let c = self.cell_of(*p) + 1;
            self.start[c] -= 1;
            self.ids[self.start[c] as usize] = i as u32;
        }
        // cell c now begins at start[c + 1]: shift back by one
        self.start.rotate_left(1);
        *self.start.last_mut().unwrap() = keys.len() as u32;
    }

    /// The ids by cell, ascending within a cell.
    pub fn ids(&self) -> &[u32] {
        &self.ids
    }

    fn clamp(&self, v: f64, o: f64, n: usize) -> usize {
        (((v - o) / self.cell).floor().max(0.0) as usize).min(n - 1)
    }

    pub fn cell_of(&self, p: Point) -> usize {
        self.clamp(p.y, self.origin.y, self.ny) * self.nx + self.clamp(p.x, self.origin.x, self.nx)
    }

    /// Calls `f` once for every item whose key lies within the square of half size `radius`
    /// around `p` (and possibly a few more).
    pub fn for_each_near(&self, p: Point, radius: f64, mut f: impl FnMut(u32)) {
        self.for_each_run(p, radius, |lo, hi| {
            for &id in &self.ids[lo..hi] {
                f(id);
            }
        });
    }

    /// Like `for_each_near`, but hands out runs of positions in `ids()`, for callers that keep
    /// their items in grid order.
    pub fn for_each_run(&self, p: Point, radius: f64, mut f: impl FnMut(usize, usize)) {
        let x0 = self.clamp(p.x - radius, self.origin.x, self.nx);
        let x1 = self.clamp(p.x + radius, self.origin.x, self.nx);
        let y0 = self.clamp(p.y - radius, self.origin.y, self.ny);
        let y1 = self.clamp(p.y + radius, self.origin.y, self.ny);
        for y in y0..=y1 {
            let row = y * self.nx;
            f(self.start[row + x0] as usize, self.start[row + x1 + 1] as usize);
        }
    }
}
