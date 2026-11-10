//! Planar geometry in millimetres.

use crate::math;
use std::f64::consts::PI;
use std::ops::{Add, Mul, Neg, Sub};

#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

pub const fn pt(x: f64, y: f64) -> Point {
    Point { x, y }
}

impl Point {
    pub fn dot(self, o: Point) -> f64 {
        self.x * o.x + self.y * o.y
    }
    pub fn cross(self, o: Point) -> f64 {
        self.x * o.y - self.y * o.x
    }
    pub fn len(self) -> f64 {
        math::hypot(self.x, self.y)
    }
    pub fn len2(self) -> f64 {
        self.dot(self)
    }
    pub fn dist(self, o: Point) -> f64 {
        (self - o).len()
    }
    pub fn norm(self) -> Point {
        let l = self.len();
        if l > 0.0 { self * (1.0 / l) } else { self }
    }
    /// Left normal (counter-clockwise in a y-up frame).
    pub fn perp(self) -> Point {
        pt(-self.y, self.x)
    }
    pub fn rotate(self, angle_rad: f64) -> Point {
        let (s, c) = math::sin_cos(angle_rad);
        pt(self.x * c - self.y * s, self.x * s + self.y * c)
    }
    pub fn lerp(self, o: Point, t: f64) -> Point {
        self + (o - self) * t
    }
}

impl Add for Point {
    type Output = Point;
    fn add(self, o: Point) -> Point {
        pt(self.x + o.x, self.y + o.y)
    }
}
impl Sub for Point {
    type Output = Point;
    fn sub(self, o: Point) -> Point {
        pt(self.x - o.x, self.y - o.y)
    }
}
impl Mul<f64> for Point {
    type Output = Point;
    fn mul(self, k: f64) -> Point {
        pt(self.x * k, self.y * k)
    }
}
impl Neg for Point {
    type Output = Point;
    fn neg(self) -> Point {
        pt(-self.x, -self.y)
    }
}

/// Twice the signed area of triangle abc; positive if counter-clockwise (y up).
pub fn orient(a: Point, b: Point, c: Point) -> f64 {
    (b - a).cross(c - a)
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Rect {
    pub min: Point,
    pub max: Point,
}

impl Rect {
    pub fn empty() -> Rect {
        Rect {
            min: pt(f64::INFINITY, f64::INFINITY),
            max: pt(f64::NEG_INFINITY, f64::NEG_INFINITY),
        }
    }
    pub fn add(&mut self, p: Point) {
        self.min.x = self.min.x.min(p.x);
        self.min.y = self.min.y.min(p.y);
        self.max.x = self.max.x.max(p.x);
        self.max.y = self.max.y.max(p.y);
    }
    pub fn grow(self, d: f64) -> Rect {
        Rect {
            min: pt(self.min.x - d, self.min.y - d),
            max: pt(self.max.x + d, self.max.y + d),
        }
    }
    pub fn overlaps(&self, o: &Rect) -> bool {
        self.min.x <= o.max.x && o.min.x <= self.max.x && self.min.y <= o.max.y && o.min.y <= self.max.y
    }
    pub fn contains(&self, p: Point) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y
    }
    /// True if the segment a-b has a point in the rectangle (clipping against the slabs).
    pub fn touches_seg(&self, a: Point, b: Point) -> bool {
        self.clip_seg(a, b).is_some()
    }

    /// The part of the segment a-b in the rectangle, as parameters (0 at a, 1 at b).
    pub fn clip_seg(&self, a: Point, b: Point) -> Option<(f64, f64)> {
        let (mut t0, mut t1) = (0.0f64, 1.0f64);
        let d = b - a;
        for (p, q, lo, hi) in [(a.x, d.x, self.min.x, self.max.x), (a.y, d.y, self.min.y, self.max.y)] {
            if q.abs() < 1e-15 {
                if p < lo || p > hi {
                    return None;
                }
            } else {
                let (u, v) = ((lo - p) / q, (hi - p) / q);
                t0 = t0.max(u.min(v));
                t1 = t1.min(u.max(v));
                if t0 > t1 {
                    return None;
                }
            }
        }
        Some((t0, t1))
    }

    /// Distance from the segment a-b to the rectangle, 0 where they touch.
    pub fn dist_seg(&self, a: Point, b: Point) -> f64 {
        if self.touches_seg(a, b) {
            return 0.0;
        }
        let to_box = |p: Point| {
            let dx = (self.min.x - p.x).max(p.x - self.max.x).max(0.0);
            let dy = (self.min.y - p.y).max(p.y - self.max.y).max(0.0);
            math::hypot(dx, dy)
        };
        [
            self.min,
            pt(self.max.x, self.min.y),
            self.max,
            pt(self.min.x, self.max.y),
        ]
        .into_iter()
        .map(|c| point_seg_dist(c, a, b))
        .fold(to_box(a).min(to_box(b)), f64::min)
    }
    pub fn width(&self) -> f64 {
        self.max.x - self.min.x
    }
    pub fn height(&self) -> f64 {
        self.max.y - self.min.y
    }
    pub fn center(&self) -> Point {
        (self.min + self.max) * 0.5
    }
}

/// Distance from p to segment ab.
pub fn point_seg_dist(p: Point, a: Point, b: Point) -> f64 {
    point_seg_offset(p, a, b).len()
}

/// From the point of segment a-b nearest to p to p: its length is `point_seg_dist`.
pub fn point_seg_offset(p: Point, a: Point, b: Point) -> Point {
    let ab = b - a;
    let l2 = ab.len2();
    if l2 == 0.0 {
        return p - a;
    }
    let t = ((p - a).dot(ab) / l2).clamp(0.0, 1.0);
    p - (a + ab * t)
}

/// Proper or touching intersection of segments ab and cd.
pub fn segs_intersect(a: Point, b: Point, c: Point, d: Point) -> bool {
    let d1 = orient(c, d, a);
    let d2 = orient(c, d, b);
    let d3 = orient(a, b, c);
    let d4 = orient(a, b, d);
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0)) {
        return true;
    }
    (d1 == 0.0 && point_seg_dist(a, c, d) == 0.0)
        || (d2 == 0.0 && point_seg_dist(b, c, d) == 0.0)
        || (d3 == 0.0 && point_seg_dist(c, a, b) == 0.0)
        || (d4 == 0.0 && point_seg_dist(d, a, b) == 0.0)
}

pub fn seg_seg_dist(a: Point, b: Point, c: Point, d: Point) -> f64 {
    if segs_intersect(a, b, c, d) {
        return 0.0;
    }
    let offs = [
        point_seg_offset(a, c, d),
        point_seg_offset(b, c, d),
        point_seg_offset(c, a, b),
        point_seg_offset(d, a, b),
    ];
    // the root only of those that can be the nearest: the squares tell the others apart
    let sq = offs.map(|o| o.len2());
    let least = sq[0].min(sq[1]).min(sq[2]).min(sq[3]);
    offs.iter()
        .zip(sq)
        .filter(|&(_, q)| q <= least * (1.0 + 1e-9))
        .fold(f64::NAN, |m, (o, _)| m.min(o.len()))
}

/// The nearest points of segments ab and cd, the first on ab: one point twice where they cross.
pub fn seg_seg_nearest(a: Point, b: Point, c: Point, d: Point) -> (Point, Point) {
    let (ab, cd) = (b - a, d - c);
    let den = ab.cross(cd);
    if den.abs() > 1e-12 {
        let t = (c - a).cross(cd) / den;
        let u = (c - a).cross(ab) / den;
        if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
            let x = a + ab * t;
            return (x, x);
        }
    }
    let cands = [
        (a, a - point_seg_offset(a, c, d)),
        (b, b - point_seg_offset(b, c, d)),
        (c - point_seg_offset(c, a, b), c),
        (d - point_seg_offset(d, a, b), d),
    ];
    cands
        .into_iter()
        .min_by(|x, y| (x.0 - x.1).len2().total_cmp(&(y.0 - y.1).len2()))
        .unwrap()
}

/// Closed polygon, vertices in either orientation, no repeated last vertex.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Polygon {
    pub pts: Vec<Point>,
}

impl Polygon {
    pub fn new(pts: Vec<Point>) -> Polygon {
        Polygon { pts }
    }

    pub fn signed_area(&self) -> f64 {
        let n = self.pts.len();
        (0..n).map(|i| self.pts[i].cross(self.pts[(i + 1) % n])).sum::<f64>() * 0.5
    }

    pub fn bbox(&self) -> Rect {
        let mut r = Rect::empty();
        for &p in &self.pts {
            r.add(p);
        }
        r
    }

    pub fn edges(&self) -> impl Iterator<Item = (Point, Point)> + '_ {
        let n = self.pts.len();
        (0..n).map(move |i| (self.pts[i], self.pts[(i + 1) % n]))
    }

    /// Even-odd point in polygon test (boundary counts as inside within numerical noise).
    pub fn contains(&self, p: Point) -> bool {
        let mut inside = false;
        let n = self.pts.len();
        let mut j = n - 1;
        for i in 0..n {
            let (a, b) = (self.pts[i], self.pts[j]);
            if (a.y > p.y) != (b.y > p.y) {
                let x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
                if p.x < x {
                    inside = !inside;
                }
            }
            j = i;
        }
        inside
    }

    /// Distance from p to the polygon area (0 inside).
    pub fn dist_to_point(&self, p: Point) -> f64 {
        if self.contains(p) {
            return 0.0;
        }
        self.boundary_dist(p)
    }

    pub fn boundary_dist(&self, p: Point) -> f64 {
        self.edges()
            .map(|(a, b)| point_seg_dist(p, a, b))
            .fold(f64::INFINITY, f64::min)
    }

    /// Distance from segment ab to the polygon area (0 if they touch or overlap).
    pub fn dist_to_seg(&self, a: Point, b: Point) -> f64 {
        if self.contains(a) || self.contains(b) {
            return 0.0;
        }
        self.edges()
            .map(|(c, d)| seg_seg_dist(a, b, c, d))
            .fold(f64::INFINITY, f64::min)
    }

    /// The polygon with its sides moved out by d. Corners stay sharp, so they reach further than
    /// d (up to four times at a spike).
    pub fn grown(&self, d: f64) -> Polygon {
        let mut pts: Vec<Point> = Vec::with_capacity(self.pts.len());
        for &p in &self.pts {
            if pts.last().is_none_or(|q| q.dist(p) > 1e-9) {
                pts.push(p);
            }
        }
        if pts.len() > 1 && pts[0].dist(pts[pts.len() - 1]) <= 1e-9 {
            pts.pop();
        }
        let out = self.signed_area().signum();
        let n = pts.len();
        let normal = |a: Point, b: Point| {
            let e = (b - a).norm();
            pt(e.y, -e.x) * out
        };
        let moved = (0..n)
            .map(|i| {
                let (n1, n2) = (normal(pts[(i + n - 1) % n], pts[i]), normal(pts[i], pts[(i + 1) % n]));
                pts[i] + (n1 + n2) * (d / (1.0 + n1.dot(n2)).max(0.125))
            })
            .collect();
        Polygon::new(moved)
    }

    pub fn dist_to_poly(&self, o: &Polygon) -> f64 {
        if o.pts.iter().any(|&p| self.contains(p)) || self.pts.iter().any(|&p| o.contains(p)) {
            return 0.0;
        }
        let mut d = f64::INFINITY;
        for (a, b) in self.edges() {
            d = d.min(o.dist_to_seg(a, b));
        }
        d
    }
}

/// Copper shape as the router sees it.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Shape {
    Circle {
        c: Point,
        r: f64,
    },
    /// Segment with round caps (a track).
    Capsule {
        a: Point,
        b: Point,
        r: f64,
    },
    Polygon(Polygon),
}

impl Shape {
    pub fn translated(&self, d: Point) -> Shape {
        match self {
            Shape::Circle { c, r } => Shape::Circle { c: *c + d, r: *r },
            Shape::Capsule { a, b, r } => Shape::Capsule {
                a: *a + d,
                b: *b + d,
                r: *r,
            },
            Shape::Polygon(p) => Shape::Polygon(Polygon::new(p.pts.iter().map(|q| *q + d).collect())),
        }
    }

    /// The shape grown by d on all sides.
    pub fn grown(&self, d: f64) -> Shape {
        match self {
            Shape::Circle { c, r } => Shape::Circle { c: *c, r: r + d },
            Shape::Capsule { a, b, r } => Shape::Capsule { a: *a, b: *b, r: r + d },
            Shape::Polygon(p) => Shape::Polygon(p.grown(d)),
        }
    }

    pub fn bbox(&self) -> Rect {
        match self {
            Shape::Circle { c, r } => {
                let mut b = Rect::empty();
                b.add(*c);
                b.grow(*r)
            }
            Shape::Capsule { a, b, r } => {
                let mut bb = Rect::empty();
                bb.add(*a);
                bb.add(*b);
                bb.grow(*r)
            }
            Shape::Polygon(p) => p.bbox(),
        }
    }

    /// Distance between the copper of two shapes (0 if overlapping).
    pub fn dist(&self, o: &Shape) -> f64 {
        use Shape::*;
        match (self, o) {
            (Circle { c, r }, other) | (other, Circle { c, r }) => (other.dist_to_point(*c) - r).max(0.0),
            (Capsule { a, b, r }, other) | (other, Capsule { a, b, r }) => (other.dist_to_seg(*a, *b) - r).max(0.0),
            (Polygon(p), Polygon(q)) => p.dist_to_poly(q),
        }
    }

    pub fn dist_to_point(&self, p: Point) -> f64 {
        match self {
            Shape::Circle { c, r } => (p.dist(*c) - r).max(0.0),
            Shape::Capsule { a, b, r } => (point_seg_dist(p, *a, *b) - r).max(0.0),
            Shape::Polygon(poly) => poly.dist_to_point(p),
        }
    }

    pub fn dist_to_seg(&self, a: Point, b: Point) -> f64 {
        match self {
            Shape::Circle { c, r } => (point_seg_dist(*c, a, b) - r).max(0.0),
            Shape::Capsule { a: c, b: d, r } => (seg_seg_dist(a, b, *c, *d) - r).max(0.0),
            Shape::Polygon(poly) => poly.dist_to_seg(a, b),
        }
    }

    /// Polygon that contains the shape, with at most `max_error` extra copper.
    pub fn to_polygon(&self, max_error: f64) -> Polygon {
        match self {
            Shape::Circle { c, r } => circle_polygon(*c, *r, max_error),
            Shape::Capsule { a, b, r } => capsule_polygon(*a, *b, *r, max_error),
            Shape::Polygon(p) => p.clone(),
        }
    }
}

/// Number of segments for a circumscribed polygon of a circle so that it deviates at most
/// `max_error` from the circle.
pub fn arc_segments(r: f64, max_error: f64, angle: f64) -> usize {
    if r <= max_error {
        return 4.max((angle / (PI / 2.0)).ceil() as usize);
    }
    // Circumscribed polygon: r / cos(a/2) - r <= e  =>  a <= 2 acos(r / (r + e))
    let step = 2.0 * math::acos(r / (r + max_error));
    ((angle / step).ceil() as usize).max(2)
}

pub fn circle_polygon(c: Point, r: f64, max_error: f64) -> Polygon {
    let n = arc_segments(r, max_error, 2.0 * PI).max(8);
    let rr = r / math::cos(PI / n as f64);
    Polygon::new(
        (0..n)
            .map(|i| c + pt(rr, 0.0).rotate(2.0 * PI * (i as f64 + 0.5) / n as f64))
            .collect(),
    )
}

pub fn capsule_polygon(a: Point, b: Point, r: f64, max_error: f64) -> Polygon {
    let d = b - a;
    if d.len() < 1e-9 {
        return circle_polygon(a, r, max_error);
    }
    let dir = d.norm();
    let n = arc_segments(r, max_error, PI).max(4);
    let rr = r / math::cos(PI / (2.0 * n as f64));
    let mut pts = Vec::with_capacity(2 * n + 2);
    let base = dir.perp();
    // cap around b, from +perp to -perp through +dir
    pts.push(b + base * r);
    for i in 0..n {
        let ang = -PI * (i as f64 + 0.5) / n as f64;
        pts.push(b + base.rotate(ang) * rr);
    }
    pts.push(b - base * r);
    pts.push(a - base * r);
    for i in 0..n {
        let ang = PI - PI * (i as f64 + 0.5) / n as f64;
        pts.push(a + base.rotate(ang) * rr);
    }
    pts.push(a + base * r);
    Polygon::new(pts)
}

/// Rectangle with rounded corners centred at the origin, rotated and moved to `at`.
pub fn roundrect_polygon(at: Point, w: f64, h: f64, radius: f64, angle_rad: f64, max_error: f64) -> Polygon {
    let r = radius.min(w / 2.0).min(h / 2.0).max(0.0);
    let hx = w / 2.0 - r;
    let hy = h / 2.0 - r;
    let mut pts = Vec::new();
    if r <= 1e-9 {
        pts = vec![
            pt(-w / 2.0, -h / 2.0),
            pt(w / 2.0, -h / 2.0),
            pt(w / 2.0, h / 2.0),
            pt(-w / 2.0, h / 2.0),
        ];
    } else {
        let n = arc_segments(r, max_error, PI / 2.0).max(2);
        let rr = r / math::cos(PI / (4.0 * n as f64));
        let corners = [
            (pt(hx, hy), 0.0),
            (pt(-hx, hy), PI / 2.0),
            (pt(-hx, -hy), PI),
            (pt(hx, -hy), 1.5 * PI),
        ];
        for (c, a0) in corners {
            pts.push(c + pt(r, 0.0).rotate(a0));
            for i in 0..n {
                let a = a0 + PI / 2.0 * (i as f64 + 0.5) / n as f64;
                pts.push(c + pt(rr, 0.0).rotate(a));
            }
            pts.push(c + pt(r, 0.0).rotate(a0 + PI / 2.0));
        }
        pts.dedup_by(|a, b| a.dist(*b) < 1e-9);
    }
    Polygon::new(pts.into_iter().map(|p| at + p.rotate(angle_rad)).collect())
}

/// Arc through three points: centre, radius, start angle and signed sweep.
pub fn arc_from_three(a: Point, m: Point, b: Point) -> Option<(Point, f64, f64, f64)> {
    let d = 2.0 * orient(a, m, b);
    if d.abs() < 1e-12 {
        return None;
    }
    let a2 = a.len2();
    let m2 = m.len2();
    let b2 = b.len2();
    let cx = (a2 * (m.y - b.y) + m2 * (b.y - a.y) + b2 * (a.y - m.y)) / d;
    let cy = (a2 * (b.x - m.x) + m2 * (a.x - b.x) + b2 * (m.x - a.x)) / d;
    let c = pt(cx, cy);
    let r = c.dist(a);
    let sa = math::atan2((a - c).y, (a - c).x);
    let ea = math::atan2((b - c).y, (b - c).x);
    let ccw = orient(a, m, b) > 0.0;
    let mut sweep = ea - sa;
    if ccw {
        while sweep <= 0.0 {
            sweep += 2.0 * PI;
        }
    } else {
        while sweep >= 0.0 {
            sweep -= 2.0 * PI;
        }
    }
    Some((c, r, sa, sweep))
}

/// Polyline approximation of an arc given by three points.
pub fn arc_polyline(a: Point, m: Point, b: Point, max_error: f64) -> Vec<Point> {
    let Some((c, r, sa, sweep)) = arc_from_three(a, m, b) else {
        return vec![a, b];
    };
    let n = arc_segments(r, max_error, sweep.abs()).max(2);
    let mut v: Vec<Point> = (0..=n)
        .map(|i| c + pt(r, 0.0).rotate(sa + sweep * i as f64 / n as f64))
        .collect();
    v[0] = a;
    v[n] = b;
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nearest_points_of_two_segments() {
        // apart: the end of one and its foot on the other
        let (p, q) = seg_seg_nearest(pt(0., 0.), pt(2., 0.), pt(1., 1.), pt(3., 2.));
        assert!(p.dist(pt(1., 0.)) < 1e-12 && q.dist(pt(1., 1.)) < 1e-12, "{p:?} {q:?}");
        // crossing: the crossing twice
        let (p, q) = seg_seg_nearest(pt(0., 0.), pt(2., 2.), pt(0., 2.), pt(2., 0.));
        assert!(p.dist(pt(1., 1.)) < 1e-12 && q.dist(pt(1., 1.)) < 1e-12, "{p:?} {q:?}");
        // parallel
        let (p, q) = seg_seg_nearest(pt(0., 0.), pt(1., 0.), pt(2., 1.), pt(3., 1.));
        assert!(p.dist(pt(1., 0.)) < 1e-12 && q.dist(pt(2., 1.)) < 1e-12, "{p:?} {q:?}");
    }

    #[test]
    fn polygons_grow_outwards_in_both_directions() {
        // a repeated point at the end, as files may have it, and an inner corner
        let l = vec![
            pt(0., 0.),
            pt(4., 0.),
            pt(4., 2.),
            pt(2., 2.),
            pt(2., 4.),
            pt(0., 4.),
            pt(0., 0.),
        ];
        for pts in [l.clone(), l.into_iter().rev().collect()] {
            let g = Polygon::new(pts).grown(0.5);
            assert_eq!(g.pts.len(), 6);
            let bb = g.bbox();
            assert_eq!((bb.min.x, bb.min.y, bb.max.x, bb.max.y), (-0.5, -0.5, 4.5, 4.5));
            assert!(!g.contains(pt(2.7, 2.7)) && g.contains(pt(2.3, 2.3)));
        }
    }

    #[test]
    fn polygon_contains_and_distance() {
        let sq = Polygon::new(vec![pt(0., 0.), pt(2., 0.), pt(2., 2.), pt(0., 2.)]);
        assert!(sq.contains(pt(1., 1.)));
        assert!(!sq.contains(pt(3., 1.)));
        assert!((sq.dist_to_point(pt(3., 1.)) - 1.0).abs() < 1e-12);
        assert!((sq.dist_to_seg(pt(3., -5.), pt(3., 5.)) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn circumscribed_polygons_contain_the_shape() {
        let c = circle_polygon(pt(0., 0.), 1.0, 0.005);
        for i in 0..360 {
            let p = pt(0.999, 0.0).rotate(i as f64 * PI / 180.0);
            assert!(c.contains(p));
        }
        let cap = capsule_polygon(pt(0., 0.), pt(5., 0.), 0.5, 0.005);
        assert!(cap.contains(pt(-0.49, 0.0)));
        assert!(cap.contains(pt(5.49, 0.0)));
        assert!(cap.contains(pt(2.5, 0.49)));
        assert!(!cap.contains(pt(2.5, 0.52)));
        let rr = roundrect_polygon(pt(10., 10.), 2.0, 1.0, 0.25, 0.3, 0.005);
        assert!(rr.contains(pt(10., 10.)));
        assert!((rr.signed_area().abs() - (2.0 - (4.0 - PI) * 0.0625)).abs() < 0.02);
    }

    #[test]
    fn arc_through_three_points() {
        let (c, r, _, sweep) = arc_from_three(pt(1., 0.), pt(0., 1.), pt(-1., 0.)).unwrap();
        assert!(c.dist(pt(0., 0.)) < 1e-12);
        assert!((r - 1.0).abs() < 1e-12);
        assert!((sweep - PI).abs() < 1e-12);
    }
}
