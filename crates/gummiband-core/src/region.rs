//! Regions for the pours of several nets on the same layers: every spot of the board goes to the
//! net whose pad is nearest, so two pours never overlap and each net gets outlines of its own.
//!
//! The regions are found on a raster over the bounding box of the board outline (KiCad clips
//! the fill at the board edge itself). Between two regions a corridor of two cells stays free,
//! wider than the clearance of the pours, so the fills of two nets cannot touch whatever the
//! filler makes of outlines that share a border. A pad that would get a region to itself, in
//! the middle of the pads of another net, gets none: a pour around one pad joins nothing, and
//! it would cut the pour of the other net apart.

use crate::board::{Board, NetId};
use crate::fill::{edt_open, pieces_of};
use crate::geom::{Point, Polygon, pt};

/// One outline of one net.
#[derive(Debug, Clone)]
pub struct Region {
    pub net: NetId,
    /// The outline as one ring; a hole (a region of another net inside) is cut in through a
    /// slit of no width, as KiCad writes outlines with holes.
    pub ring: Polygon,
    /// The pads of the net in it.
    pub pins: Vec<usize>,
}

#[derive(Debug, Clone, Default)]
pub struct Regions {
    pub regions: Vec<Region>,
    /// Pads that got no region: alone among the pads of other nets.
    pub alone: Vec<usize>,
}

/// The raster step for a pour clearance: the corridor between two regions is two cells, and
/// across a diagonal border 1.4 cells are left of it.
pub fn step_for(clearance: f64) -> f64 {
    (clearance * 1.5).max(0.25)
}

/// The regions of `nets` by nearest pad, on a raster of `step` mm (coarser on a very large
/// board).
pub fn by_nearest_pad(board: &Board, nets: &[NetId], step: f64) -> Regions {
    let bounds = board.main_outline().map_or_else(|| board.bbox(), |o| o.bbox());
    let (w, h) = (bounds.width(), bounds.height());
    let mut out = Regions::default();
    if nets.is_empty() || w <= 0.0 || h <= 0.0 {
        return out;
    }
    let step = step.max((w * h / 4.0e6).sqrt());
    let (nx, ny) = ((w / step).ceil() as usize, (h / step).ceil() as usize);
    let n = nx * ny;
    let cell_of = |p: Point| -> usize {
        let c = (((p.x - bounds.min.x) / step).floor().max(0.0) as usize).min(nx - 1);
        let r = (((p.y - bounds.min.y) / step).floor().max(0.0) as usize).min(ny - 1);
        r * nx + c
    };
    // a grid vertex, with the last column and row cut at the bounds
    let vertex = |i: usize, j: usize| {
        pt(
            (bounds.min.x + i as f64 * step).min(bounds.max.x),
            (bounds.min.y + j as f64 * step).min(bounds.max.y),
        )
    };
    let real = |p: usize| !board.pins[p].footprint.is_empty();
    let mut sites: Vec<Vec<usize>> = nets
        .iter()
        .map(|&net| {
            board.nets[net as usize]
                .pins
                .iter()
                .copied()
                .filter(|&p| real(p))
                .collect()
        })
        .collect();

    // label: the index into `nets` of the nearest pad, or NONE in the corridors
    const NONE: u32 = u32::MAX;
    let mut label = vec![NONE; n];
    let mut comps: Vec<(Vec<u32>, usize, Vec<u32>)> = Vec::new();
    for _round in 0..6 {
        let mut best = vec![f32::INFINITY; n];
        label.iter_mut().for_each(|l| *l = NONE);
        for (k, pins) in sites.iter().enumerate() {
            if pins.is_empty() {
                continue;
            }
            let mut m = vec![true; n];
            for &p in pins {
                m[cell_of(board.pins[p].pos)] = false;
            }
            let d = edt_open(&m, nx, ny);
            for i in 0..n {
                if d[i] < best[i] {
                    best[i] = d[i];
                    label[i] = k as u32;
                }
            }
        }
        // the corridor: a cell next to a cell of another net (across a side or a corner)
        let near_other: Vec<bool> = (0..n)
            .map(|i| {
                let (c, r) = ((i % nx) as isize, (i / nx) as isize);
                (-1..=1).any(|dr| {
                    (-1..=1).any(|dc| {
                        let (c2, r2) = (c + dc, r + dr);
                        c2 >= 0
                            && r2 >= 0
                            && (c2 as usize) < nx
                            && (r2 as usize) < ny
                            && label[r2 as usize * nx + c2 as usize] != label[i]
                    })
                })
            })
            .collect();
        for i in 0..n {
            if near_other[i] {
                label[i] = NONE;
            }
        }
        // the pieces of every net and the pads each holds. A pad in the corridor (next to a
        // pad of another net) counts for the piece of its net within three cells of it
        comps.clear();
        let mut alone: Vec<Vec<usize>> = vec![Vec::new(); nets.len()];
        for (k, pins) in sites.iter().enumerate() {
            let m: Vec<bool> = label.iter().map(|&l| l == k as u32).collect();
            let (lab, count) = pieces_of(&m, nx, ny);
            let of: Vec<u32> = pins
                .iter()
                .map(|&p| piece_near(&lab, nx, ny, cell_of(board.pins[p].pos)))
                .collect();
            let mut held = vec![0usize; count];
            for &l in of.iter().filter(|&&l| l != 0) {
                held[l as usize - 1] += 1;
            }
            for (&p, &l) in pins.iter().zip(&of) {
                if l == 0 || held[l as usize - 1] < 2 {
                    alone[k].push(p);
                }
            }
            comps.push((lab, count, of));
        }
        if alone.iter().all(Vec::is_empty) {
            break;
        }
        for (k, gone) in alone.into_iter().enumerate() {
            sites[k].retain(|p| !gone.contains(p));
            out.alone.extend(gone);
        }
    }

    // ---- the outline of every piece
    for (k, (lab, count, of)) in comps.iter().enumerate() {
        // (after a last round that still took pads out, `of` has more entries than `sites`)
        if sites[k].is_empty() || of.len() != sites[k].len() {
            continue;
        }
        for c in 1..=*count as u32 {
            let pins: Vec<usize> = sites[k]
                .iter()
                .zip(of)
                .filter(|(_, l)| **l == c)
                .map(|(&p, _)| p)
                .collect();
            if pins.is_empty() {
                continue;
            }
            let rings = rings_of(nx, ny, |col, row| {
                col >= 0
                    && row >= 0
                    && (col as usize) < nx
                    && (row as usize) < ny
                    && lab[row as usize * nx + col as usize] == c
            });
            // with the cells on the left and y down, the outside ring has a negative area and
            // the holes a positive one
            let area = |r: &Vec<(usize, usize)>| {
                let mut a = 0.0;
                for i in 0..r.len() {
                    let (p, q) = (r[i], r[(i + 1) % r.len()]);
                    a += p.0 as f64 * q.1 as f64 - q.0 as f64 * p.1 as f64;
                }
                a / 2.0
            };
            let Some(outer) = rings.iter().find(|r| area(r) < 0.0) else {
                continue;
            };
            let mut ring = chamfer(outer, nx, ny, &vertex);
            let mut holes: Vec<Vec<Point>> = rings
                .iter()
                .filter(|r| area(r) > 0.0)
                .map(|r| chamfer(r, nx, ny, &vertex))
                .collect();
            holes.sort_by(|a, b| top_of(a).1.y.total_cmp(&top_of(b).1.y));
            for hole in holes {
                cut_in(&mut ring, hole);
            }
            out.regions.push(Region {
                net: nets[k],
                ring: Polygon::new(ring),
                pins,
            });
        }
    }
    out.alone.sort_unstable();
    out.alone.dedup();
    out
}

/// The piece (label from 1, 0: none) at a cell or, where the cell has none, at the nearest cell
/// within three cells of it.
fn piece_near(lab: &[u32], nx: usize, ny: usize, cell: usize) -> u32 {
    if lab[cell] != 0 {
        return lab[cell];
    }
    let (c, r) = ((cell % nx) as isize, (cell / nx) as isize);
    let mut best: Option<(isize, u32)> = None;
    for dr in -3..=3isize {
        for dc in -3..=3isize {
            let (c2, r2) = (c + dc, r + dr);
            if c2 < 0 || r2 < 0 || c2 as usize >= nx || r2 as usize >= ny {
                continue;
            }
            let l = lab[r2 as usize * nx + c2 as usize];
            let d = dc * dc + dr * dr;
            if l != 0 && best.is_none_or(|b| d < b.0) {
                best = Some((d, l));
            }
        }
    }
    best.map_or(0, |b| b.1)
}

/// The boundary of a set of cells as rings of grid vertices, the cells on the left of every
/// edge. Where two cells of the set touch at a corner only, the rings do not pass through it.
fn rings_of(nx: usize, ny: usize, inside: impl Fn(isize, isize) -> bool) -> Vec<Vec<(usize, usize)>> {
    use std::collections::HashMap;
    // directed edges by their start vertex; direction 0: +x, 1: +y, 2: -x, 3: -y
    let mut from: HashMap<(usize, usize), Vec<u8>> = HashMap::new();
    for row in 0..ny as isize {
        for col in 0..nx as isize {
            if !inside(col, row) {
                continue;
            }
            let (c, r) = (col as usize, row as usize);
            // with y down and the cell on the left: along the top edge in -x, and so on
            if !inside(col, row - 1) {
                from.entry((c + 1, r)).or_default().push(2);
            }
            if !inside(col - 1, row) {
                from.entry((c, r)).or_default().push(1);
            }
            if !inside(col, row + 1) {
                from.entry((c, r + 1)).or_default().push(0);
            }
            if !inside(col + 1, row) {
                from.entry((c + 1, r + 1)).or_default().push(3);
            }
        }
    }
    let step = |v: (usize, usize), d: u8| match d {
        0 => (v.0 + 1, v.1),
        1 => (v.0, v.1 + 1),
        2 => (v.0 - 1, v.1),
        _ => (v.0, v.1 - 1),
    };
    let mut starts: Vec<(usize, usize)> = from.keys().copied().collect();
    starts.sort_unstable();
    let mut rings = Vec::new();
    for s in starts {
        while let Some(d0) = from.get_mut(&s).and_then(Vec::pop) {
            let mut ring = vec![s];
            let (mut v, mut d) = (step(s, d0), d0);
            while v != s {
                // the turns in the order that keeps to the own cell: with y down and the cells
                // on the left that is the turn to the left (d - 1), then straight, then right
                let outs = from.get_mut(&v).expect("a closed boundary");
                let pick = [(d + 3) % 4, d, (d + 1) % 4]
                    .into_iter()
                    .find_map(|want| outs.iter().position(|&x| x == want))
                    .expect("a closed boundary");
                let next = outs.swap_remove(pick);
                if next != d {
                    ring.push(v);
                }
                d = next;
                v = step(v, d);
            }
            // the start is a corner only if the ring turns there
            if d == d0 {
                ring.remove(0);
            }
            rings.push(ring);
        }
    }
    rings
}

/// A ring of grid vertices as points, its corners inside the raster cut by half a cell on each
/// edge, which turns a staircase into a slanted line. Corners on the border of the raster stay.
fn chamfer(ring: &[(usize, usize)], nx: usize, ny: usize, vertex: &impl Fn(usize, usize) -> Point) -> Vec<Point> {
    let on_border = |v: (usize, usize)| v.0 == 0 || v.1 == 0 || v.0 == nx || v.1 == ny;
    let n = ring.len();
    let mut out: Vec<Point> = Vec::with_capacity(2 * n);
    let mut push = |p: Point| {
        if out.last().is_none_or(|q: &Point| q.dist(p) > 1e-9) {
            out.push(p);
        }
    };
    for i in 0..n {
        let (prev, v, next) = (ring[(i + n - 1) % n], ring[i], ring[(i + 1) % n]);
        let at = vertex(v.0, v.1);
        if on_border(v) {
            push(at);
        } else {
            // (edges run along the axes: their length in cells is the difference that is not 0)
            let cells = |a: (usize, usize), b: (usize, usize)| (a.0.abs_diff(b.0) + a.1.abs_diff(b.1)).max(1) as f64;
            push(at.lerp(vertex(prev.0, prev.1), 0.5 / cells(prev, v)));
            push(at.lerp(vertex(next.0, next.1), 0.5 / cells(v, next)));
        }
    }
    if out.len() > 1 && out[0].dist(*out.last().unwrap()) <= 1e-9 {
        out.pop();
    }
    // points on a straight line between their neighbours say nothing
    let mut i = 0;
    while out.len() > 3 && i < out.len() {
        let m = out.len();
        let (a, b, c) = (out[(i + m - 1) % m], out[i], out[(i + 1) % m]);
        if (b - a).cross(c - b).abs() < 1e-9 && (b - a).dot(c - b) > 0.0 {
            out.remove(i);
        } else {
            i += 1;
        }
    }
    out
}

/// The index and the point of a ring with the smallest y (then x).
fn top_of(ring: &[Point]) -> (usize, Point) {
    let mut best = 0;
    for (i, p) in ring.iter().enumerate() {
        if (p.y, p.x) < (ring[best].y, ring[best].x) {
            best = i;
        }
    }
    (best, ring[best])
}

/// Cuts a hole into a ring through a slit from the top of the hole straight up to the ring.
fn cut_in(ring: &mut Vec<Point>, hole: Vec<Point>) {
    let (hi, p) = top_of(&hole);
    // the edge of the ring that the way up from p meets first
    let mut hit: Option<(usize, f64)> = None;
    for i in 0..ring.len() {
        let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
        let (lo, hi_x) = (a.x.min(b.x), a.x.max(b.x));
        if (b.x - a.x).abs() < 1e-12 || p.x < lo || p.x >= hi_x {
            continue;
        }
        let y = a.y + (p.x - a.x) * (b.y - a.y) / (b.x - a.x);
        if y < p.y - 1e-12 && hit.is_none_or(|h| y > h.1) {
            hit = Some((i, y));
        }
    }
    let Some((at, y)) = hit else { return };
    let x = pt(p.x, y);
    let mut add = vec![x];
    add.extend(hole[hi..].iter().copied());
    add.extend(hole[..hi].iter().copied());
    add.push(p);
    add.push(x);
    ring.splice(at + 1..at + 1, add);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{LayerSet, Net, Obstacle, ObstacleKind, Pin};
    use crate::geom::{Shape, roundrect_polygon};

    /// A 40 x 20 mm board with a pad per entry (net, x, y).
    fn board(pads: &[(&str, f64, f64)]) -> Board {
        let mut b = crate::topo::tests::two_pad_board();
        b.outline = vec![Polygon::new(vec![
            pt(10., 10.),
            pt(50., 10.),
            pt(50., 30.),
            pt(10., 30.),
        ])];
        b.nets.clear();
        b.pins.clear();
        b.obstacles.clear();
        for (i, &(net, x, y)) in pads.iter().enumerate() {
            let n = b.nets.iter().position(|n| n.name == net).unwrap_or_else(|| {
                b.nets.push(Net {
                    name: net.into(),
                    class: 0,
                    pins: Vec::new(),
                    planes: LayerSet::default(),
                    priority: 0,
                });
                b.nets.len() - 1
            });
            b.nets[n].pins.push(i);
            b.obstacles.push(Obstacle {
                shape: Shape::Polygon(roundrect_polygon(pt(x, y), 1.0, 1.0, 0.0, 0.0, 0.005)),
                layers: LayerSet::single(0),
                net: Some(n as NetId),
                kind: ObstacleKind::Pad,
                hole: None,
                clearance: 0.2,
                pin: Some(i),
                locked: true,
            });
            b.pins.push(Pin {
                name: format!("P.{i}"),
                footprint: "P".into(),
                net: n as NetId,
                pos: pt(x, y),
                layers: LayerSet::single(0),
                obstacles: vec![i],
                size: (1.0, 1.0),
            });
        }
        b
    }

    fn overlap(a: &Polygon, b: &Polygon) -> bool {
        a.pts.iter().any(|p| b.contains(*p) && b.boundary_dist(*p) > 1e-6)
            || b.pts.iter().any(|p| a.contains(*p) && a.boundary_dist(*p) > 1e-6)
            || a.dist_to_poly(b) <= 0.0
    }

    #[test]
    fn two_nets_on_two_halves_get_a_half_each() {
        let b = board(&[
            ("GND", 14., 14.),
            ("GND", 16., 26.),
            ("GND", 24., 20.),
            ("GNDR", 36., 14.),
            ("GNDR", 44., 26.),
            ("GNDR", 38., 22.),
        ]);
        let r = by_nearest_pad(&b, &[0, 1], 0.25);
        assert!(r.alone.is_empty());
        assert_eq!(r.regions.len(), 2);
        let (left, right) = (&r.regions[0], &r.regions[1]);
        assert_eq!((left.net, right.net), (0, 1));
        assert_eq!((left.pins.len(), right.pins.len()), (3, 3));
        // each holds its pads, none holds a pad of the other
        for (i, pin) in b.pins.iter().enumerate() {
            assert_eq!(left.ring.contains(pin.pos), i < 3, "{i}");
            assert_eq!(right.ring.contains(pin.pos), i >= 3, "{i}");
        }
        // they do not overlap, and between them more than the clearance of a pour is free
        assert!(!overlap(&left.ring, &right.ring));
        let gap = left.ring.dist_to_poly(&right.ring);
        assert!(gap > 0.3 && gap < 0.8, "{gap}");
        // together they cover the board but for the corridor, and stay inside its bounds
        let area = left.ring.signed_area().abs() + right.ring.signed_area().abs();
        assert!(area > 0.95 * 800.0 && area < 800.0, "{area}");
        for p in left.ring.pts.iter().chain(&right.ring.pts) {
            assert!(p.x >= 10.0 && p.x <= 50.0 && p.y >= 10.0 && p.y <= 30.0, "{p:?}");
        }
        // a slanted border is a slanted line, not a staircase of the raster: few corners
        assert!(left.ring.pts.len() < 200, "{}", left.ring.pts.len());
    }

    #[test]
    fn a_pad_alone_among_the_pads_of_another_net_gets_no_region() {
        let b = board(&[
            ("GND", 14., 14.),
            ("GND", 46., 26.),
            ("GND", 24., 20.),
            ("GND", 40., 14.),
            ("GNDR", 30., 20.),
        ]);
        let r = by_nearest_pad(&b, &[0, 1], 0.25);
        assert_eq!(r.alone, vec![4]);
        assert_eq!(r.regions.len(), 1);
        assert_eq!(r.regions[0].net, 0);
        // the whole board goes to the other net
        assert!((r.regions[0].ring.signed_area().abs() - 800.0).abs() < 1e-6);
        assert_eq!(r.regions[0].ring.pts.len(), 4);
    }

    #[test]
    fn a_region_inside_another_is_cut_in_as_a_hole() {
        // GNDR in the middle with two pads, GND around it on all sides
        let b = board(&[
            ("GND", 12., 12.),
            ("GND", 48., 12.),
            ("GND", 12., 28.),
            ("GND", 48., 28.),
            ("GND", 30., 11.),
            ("GND", 30., 29.),
            ("GNDR", 29., 20.),
            ("GNDR", 31., 20.),
        ]);
        let r = by_nearest_pad(&b, &[0, 1], 0.25);
        assert!(r.alone.is_empty());
        assert_eq!(r.regions.len(), 2);
        let (around, middle) = (&r.regions[0], &r.regions[1]);
        assert_eq!((around.net, middle.net), (0, 1));
        // the middle is not part of the ring around it: the slit makes the ring one outline
        assert!(!around.ring.contains(pt(30., 20.)));
        assert!(around.ring.contains(pt(12., 12.)) && around.ring.contains(pt(30., 28.5)));
        assert!(middle.ring.contains(pt(30., 20.)));
        let hole = middle.ring.signed_area().abs();
        let ring = around.ring.signed_area().abs();
        assert!(ring + hole < 800.0 && ring + hole > 0.9 * 800.0, "{ring} {hole}");
    }
}
