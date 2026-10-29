//! Uniform grid over bounding boxes for proximity queries.

use crate::geom::{Point, Rect};

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
            ids: vec![0; keys.len()],
        };
        let cells: Vec<usize> = keys.iter().map(|p| g.cell_of(*p)).collect();
        for &c in &cells {
            g.start[c + 1] += 1;
        }
        for i in 0..nx * ny {
            g.start[i + 1] += g.start[i];
        }
        let mut fill = g.start.clone();
        for (i, &c) in cells.iter().enumerate() {
            g.ids[fill[c] as usize] = i as u32;
            fill[c] += 1;
        }
        g
    }

    fn clamp(&self, v: f64, o: f64, n: usize) -> usize {
        (((v - o) / self.cell).floor().max(0.0) as usize).min(n - 1)
    }

    fn cell_of(&self, p: Point) -> usize {
        self.clamp(p.y, self.origin.y, self.ny) * self.nx + self.clamp(p.x, self.origin.x, self.nx)
    }

    /// Calls `f` once for every item whose key lies within the square of half size `radius`
    /// around `p` (and possibly a few more).
    pub fn for_each_near(&self, p: Point, radius: f64, mut f: impl FnMut(u32)) {
        let x0 = self.clamp(p.x - radius, self.origin.x, self.nx);
        let x1 = self.clamp(p.x + radius, self.origin.x, self.nx);
        let y0 = self.clamp(p.y - radius, self.origin.y, self.ny);
        let y1 = self.clamp(p.y + radius, self.origin.y, self.ny);
        for y in y0..=y1 {
            let row = y * self.nx;
            for &id in &self.ids[self.start[row + x0] as usize..self.start[row + x1 + 1] as usize] {
                f(id);
            }
        }
    }
}
