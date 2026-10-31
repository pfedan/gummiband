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

/// The cell along one axis, clamped to the grid: the cast truncates, which is the floor for
/// positive values, and anything below the first cell (or NaN) lands in it. `inv` is 1 / cell;
/// the cells of all grids here are powers of two, where the product is exactly the quotient.
#[inline]
pub fn axis_cell(v: f64, o: f64, inv: f64, n: usize) -> usize {
    (((v - o) * inv) as i64).clamp(0, n as i64 - 1) as usize
}

pub struct GridIndex {
    origin: Point,
    inv: f64,
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
            inv: 1.0 / cell,
            nx,
            ny,
            cells: vec![Vec::new(); nx * ny],
        }
    }

    fn range(&self, r: &Rect) -> (usize, usize, usize, usize) {
        let f = |v: f64, o: f64, n: usize| axis_cell(v, o, self.inv, n);
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
    #[cfg_attr(not(feature = "gpu"), allow(dead_code))]
    cell: f64,
    inv: f64,
    nx: usize,
    ny: usize,
    start: Vec<u32>,
    ids: Vec<u32>,
    /// The cell of every key, by key, and the buffer for the next keys.
    cells: Vec<u32>,
    next: Vec<u32>,
    /// Scratch of `update`.
    runs: Vec<(u32, u32)>,
}

impl PointGrid {
    pub fn new(bounds: Rect, cell: f64, keys: &[Point]) -> PointGrid {
        let b = bounds.grow(cell);
        let nx = ((b.width() / cell).ceil() as usize).max(1);
        let ny = ((b.height() / cell).ceil() as usize).max(1);
        let mut g = PointGrid {
            origin: b.min,
            cell,
            inv: 1.0 / cell,
            nx,
            ny,
            start: vec![0; nx * ny + 1],
            ids: Vec::new(),
            cells: Vec::new(),
            next: Vec::new(),
            runs: Vec::new(),
        };
        g.rebuild(keys);
        g
    }

    /// New keys on the same grid, without new allocations once the buffers are large enough.
    pub fn rebuild(&mut self, keys: &[Point]) {
        let mut cells = std::mem::take(&mut self.next);
        cells.clear();
        cells.extend(keys.iter().map(|&p| self.cell_of(p) as u32));
        let old = std::mem::replace(&mut self.cells, cells);
        // a relaxation moves few keys into other cells from round to round, and a large grid
        // costs more to sum up than its keys: then only the cells between change
        if old.len() != self.cells.len() || !self.update(&old) {
            self.build();
        }
        self.next = old;
    }

    /// The grid after keys moved from their `old` cells to `cells`, the same as `build` makes:
    /// only the cells from where a key leaves to where it enters change, and the keys there.
    /// False, and nothing changed, if that is a large part of the grid.
    fn update(&mut self, old: &[u32]) -> bool {
        let mut runs = std::mem::take(&mut self.runs);
        runs.clear();
        runs.extend(
            old.iter()
                .zip(&self.cells)
                .filter(|(a, b)| a != b)
                .map(|(&a, &b)| (a.min(b), a.max(b))),
        );
        runs.sort_unstable();
        // overlapping runs share keys: merge them
        let mut n = 0;
        for i in 0..runs.len() {
            if n > 0 && runs[i].0 <= runs[n - 1].1 {
                runs[n - 1].1 = runs[n - 1].1.max(runs[i].1);
            } else {
                runs[n] = runs[i];
                n += 1;
            }
        }
        runs.truncate(n);
        let span: usize = runs.iter().map(|&(lo, hi)| (hi - lo) as usize + 1).sum();
        let small = span <= self.start.len() / 8;
        if small {
            for &(lo, hi) in &runs {
                let (lo, hi) = (lo as usize, hi as usize);
                // the keys of these cells keep their places in `ids` as a whole
                let (a, b) = (self.start[lo] as usize, self.start[hi + 1] as usize);
                let cells = &self.cells;
                let ids = &mut self.ids[a..b];
                ids.sort_unstable_by_key(|&i| (cells[i as usize] as u64) << 32 | i as u64);
                let mut j = 0;
                for c in lo..=hi {
                    self.start[c] = (a + j) as u32;
                    while j < ids.len() && cells[ids[j] as usize] as usize == c {
                        j += 1;
                    }
                }
            }
        }
        self.runs = runs;
        small
    }

    /// The grid of `cells` from scratch.
    fn build(&mut self) {
        let cells = &self.cells;
        self.start.fill(0);
        // start[c] counts cell c, the prefix sum then makes it the end of cell c (and the
        // entry behind the last cell the number of keys)
        for &c in cells {
            self.start[c as usize] += 1;
        }
        let mut sum = 0;
        for s in &mut self.start {
            sum += *s;
            *s = sum;
        }
        self.ids.clear();
        self.ids.resize(cells.len(), 0);
        // filling from the back keeps the ids of a cell ascending and leaves start[c] at the
        // begin of cell c
        for (i, &c) in cells.iter().enumerate().rev() {
            let s = &mut self.start[c as usize];
            *s -= 1;
            self.ids[*s as usize] = i as u32;
        }
    }

    /// The cell of every key of the last build, by key.
    pub fn key_cells(&self) -> &[u32] {
        &self.cells
    }

    /// The ids by cell, ascending within a cell.
    pub fn ids(&self) -> &[u32] {
        &self.ids
    }

    /// Where each cell begins in `ids()`, one entry more than there are cells.
    #[cfg(feature = "gpu")]
    pub fn starts(&self) -> &[u32] {
        &self.start
    }

    /// Origin, cell size and number of cells in x and y.
    #[cfg(feature = "gpu")]
    pub fn layout(&self) -> (Point, f64, usize, usize) {
        (self.origin, self.cell, self.nx, self.ny)
    }

    fn clamp(&self, v: f64, o: f64, n: usize) -> usize {
        axis_cell(v, o, self.inv, n)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::pt;

    /// Keys that move a little from round to round: the grid that follows them is the grid
    /// built from scratch.
    #[test]
    fn point_grid_follows_moving_keys() {
        let mut bounds = Rect::empty();
        bounds.add(pt(0.0, 0.0));
        // keys on a part of the grid, so that a few moves change a small part of it
        bounds.add(pt(80.0, 40.0));
        let mut seed = 7u64;
        let mut unit = move || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut keys: Vec<Point> = (0..400).map(|_| pt(unit() * 20.0, unit() * 10.0)).collect();
        let mut grid = PointGrid::new(bounds, 0.25, &keys);
        for round in 0..60 {
            // some rounds move a few keys, some many, some none
            let share = [0.0, 0.01, 0.1, 1.0][round % 4];
            for k in keys.iter_mut() {
                if unit() < share {
                    *k = pt(k.x + (unit() - 0.5) * 0.6, k.y + (unit() - 0.5) * 0.6);
                }
            }
            let old = grid.cells.clone();
            grid.rebuild(&keys);
            if share == 0.01 {
                assert_ne!(old, grid.cells, "round {round} moves keys to other cells");
                let mut again = PointGrid::new(bounds, 0.25, &keys);
                again.cells = old;
                again.build();
                again.cells = grid.cells.clone();
                assert!(again.update(&grid.next), "round {round} updates in place");
                assert_eq!(again.ids, grid.ids);
            }
            let fresh = PointGrid::new(bounds, 0.25, &keys);
            assert_eq!(grid.ids, fresh.ids, "round {round}");
            assert_eq!(grid.start, fresh.start, "round {round}");
        }
    }
}
