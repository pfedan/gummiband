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
