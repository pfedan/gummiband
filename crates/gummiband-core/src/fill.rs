//! A model of the copper fill of an area net.
//!
//! An area is a zone outline of a net on one copper layer: a pour on an outer layer or a plane
//! on an inner one. KiCad fills it later; the router so far only knows the outline. This module
//! computes what the fill will be on the copper as it is now, and what it joins:
//!
//! 1. the outline, inside the board and its edge clearance, without the outlines of other nets'
//!    areas that win by priority;
//! 2. without the copper of other nets (pads, tracks, vias, holes) grown by the clearance, and
//!    without the thermal gap around the pads of its own net that get a thermal relief;
//! 3. without everything narrower than the minimum width (an opening: erode by half the width,
//!    grow again);
//! 4. with the thermal spokes that end in what is left;
//! 5. then the pieces, joined with the pads, tracks and vias of the net over all layers into
//!    groups. The area is the group with the most fill among those that join at least two
//!    pads (where none does: the largest piece, and the result says so); a pin outside it is
//!    not reached, a group with pins outside it is an island, and for every island the best
//!    site for one via that joins it to the area is looked for.
//!
//! It works on a raster (cell centres, an exact Euclidean distance transform), so a statement is
//! good to about one cell: 0.04 to 0.05 mm by default. The step is chosen so that the minimum
//! width is an odd number of cells, which makes "narrower than the minimum width" exact for
//! strips along the axes. A strip within one cell of the minimum width may come out either way:
//! [`Options::slack`] moves every clearance by a part of a cell, and a caller that runs the model
//! with half a cell less and half a cell more sees which pads depend on such a strip.
//!
//! The fill is never stored: it is computed from the board and its copper whenever it is asked
//! for. `gummiband_app::areas` turns it into the report of `gummiband areas` and `area_report`.
//! The model is the router's own; it has not been held against KiCad's filler at thin strips
//! yet (`bench/areas-kicad.sh` does that where KiCad is installed).

use std::collections::HashMap;

use crate::board::{Board, LayerId, LayerSet, NetId, ObstacleKind};
use crate::geom::{Point, Polygon, Rect, Shape, pt};

/// How the pads of the area's net join the fill.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
pub enum Connect {
    /// Not at all: the fill keeps its clearance from the pad.
    None,
    /// Through four spokes across a gap.
    Thermal,
    /// Solid copper.
    Solid,
    /// Thermal reliefs on through hole pads, solid on the others.
    ThermalTht,
}

/// A zone outline of one net on one layer, with the settings of its fill.
#[derive(Debug, Clone)]
pub struct Area {
    pub net: NetId,
    pub layer: LayerId,
    pub name: String,
    /// Outline rings, even-odd: a ring inside another is a hole.
    pub rings: Vec<Polygon>,
    /// KiCad fills the area with the higher priority first; `tie` decides between equals.
    pub priority: i64,
    /// What KiCad compares when two areas have the same priority (the zone's id).
    pub tie: String,
    /// Clearance of the fill to other nets' copper (at least the net class clearance holds).
    pub clearance: f64,
    /// Narrower fill than this is left out.
    pub min_width: f64,
    pub connect: Connect,
    pub thermal_gap: f64,
    pub spoke_width: f64,
    /// Pieces of fill that hold nothing of the net are left out below this area in mm²
    /// (infinity: always, 0: never).
    pub island_min: f64,
}

impl Area {
    /// An area with the settings the tools write for a pour (`ground_pour`).
    pub fn pour(net: NetId, layer: LayerId, rings: Vec<Polygon>) -> Area {
        Area {
            net,
            layer,
            name: String::new(),
            rings,
            priority: 0,
            tie: String::new(),
            clearance: 0.2,
            min_width: 0.2,
            connect: Connect::Thermal,
            thermal_gap: 0.3,
            spoke_width: 0.3,
            island_min: f64::INFINITY,
        }
    }

    fn wins_over(&self, o: &Area) -> bool {
        (self.priority, &self.tie) > (o.priority, &o.tie)
    }

    fn contains(&self, p: Point) -> bool {
        self.rings.iter().filter(|r| r.contains(p)).count() % 2 == 1
    }
}

#[derive(Debug, Clone)]
pub struct Options {
    /// Wanted raster step in mm; the step used makes the smallest minimum width an odd number
    /// of cells.
    pub step: f64,
    /// Upper limit of raster cells per layer; a larger board gets a coarser step.
    pub max_cells: usize,
    /// How single pads join an area, by pin, where the pad says so itself.
    pub pad_connect: HashMap<usize, Connect>,
    /// Rule areas that forbid copper fill.
    pub no_pour: Vec<(LayerSet, Polygon)>,
    /// Diameter and drill of the via a site is looked for; default: the net class.
    pub via: Option<(f64, f64)>,
    /// Added to every clearance of the fill and to the thermal gap, in mm (the room for vias
    /// stays as it is). With plus and minus half a raster step the width of every strip of
    /// fill changes by one cell: what then changes is within the precision of the raster.
    pub slack: f64,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            step: 0.05,
            max_cells: 12_000_000,
            pad_connect: HashMap::new(),
            no_pour: Vec::new(),
            via: None,
            slack: 0.0,
        }
    }
}

/// A raster over the board; a cell stands for its centre.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Raster {
    pub x0: f64,
    pub y0: f64,
    pub step: f64,
    pub nx: usize,
    pub ny: usize,
}

impl Raster {
    pub fn new(bounds: Rect, step: f64) -> Raster {
        Raster {
            x0: bounds.min.x,
            y0: bounds.min.y,
            step,
            nx: (bounds.width() / step).ceil() as usize + 1,
            ny: (bounds.height() / step).ceil() as usize + 1,
        }
    }

    /// A raster whose step divides `min_width` into an odd number of cells, as close to `step`
    /// as the cell limit allows.
    pub fn for_width(bounds: Rect, step: f64, min_width: f64, max_cells: usize) -> Raster {
        if min_width <= 0.0 {
            let fit = (bounds.width() * bounds.height() / max_cells as f64).sqrt();
            return Raster::new(bounds, step.max(fit));
        }
        let mut m = ((min_width / step - 1.0) / 2.0).round().max(0.0) as usize * 2 + 1;
        loop {
            let r = Raster::new(bounds, min_width / m as f64);
            if r.n() <= max_cells || m == 1 {
                return r;
            }
            m -= 2;
        }
    }

    pub fn n(&self) -> usize {
        self.nx * self.ny
    }

    pub fn center(&self, i: usize) -> Point {
        pt(
            self.x0 + ((i % self.nx) as f64 + 0.5) * self.step,
            self.y0 + ((i / self.nx) as f64 + 0.5) * self.step,
        )
    }

    pub fn cell(&self, p: Point) -> Option<usize> {
        let c = ((p.x - self.x0) / self.step).floor();
        let r = ((p.y - self.y0) / self.step).floor();
        (c >= 0.0 && r >= 0.0 && (c as usize) < self.nx && (r as usize) < self.ny)
            .then(|| r as usize * self.nx + c as usize)
    }

    fn range(&self, r: Rect) -> (usize, usize, usize, usize) {
        let c0 = (((r.min.x - self.x0) / self.step).floor().max(0.0)) as usize;
        let r0 = (((r.min.y - self.y0) / self.step).floor().max(0.0)) as usize;
        let c1 = ((((r.max.x - self.x0) / self.step).ceil() + 1.0).max(0.0) as usize).min(self.nx);
        let r1 = ((((r.max.y - self.y0) / self.step).ceil() + 1.0).max(0.0) as usize).min(self.ny);
        (c0, r0, c1, r1)
    }

    /// Calls `f` for every cell whose centre lies within `grow` of the shape.
    fn each_in(&self, s: &Shape, grow: f64, mut f: impl FnMut(usize)) {
        let (c0, r0, c1, r1) = self.range(s.bbox().grow(grow));
        for r in r0..r1 {
            for c in c0..c1 {
                let i = r * self.nx + c;
                if s.dist_to_point(self.center(i)) <= grow + 1e-9 {
                    f(i);
                }
            }
        }
    }

    /// Sets the cells within `grow` of the shape.
    pub fn mark(&self, m: &mut [bool], s: &Shape, grow: f64) {
        self.each_in(s, grow, |i| m[i] = true);
    }

    /// Flips the cells inside the polygon (even-odd scanline): rings marked one after the other
    /// give the area with its holes.
    pub fn toggle(&self, m: &mut [bool], p: &Polygon) {
        if p.pts.len() < 3 {
            return;
        }
        let (c0, r0, c1, r1) = self.range(p.bbox());
        let mut xs = Vec::new();
        for r in r0..r1 {
            let y = self.y0 + (r as f64 + 0.5) * self.step;
            xs.clear();
            for (a, b) in p.edges() {
                if (a.y > y) != (b.y > y) {
                    xs.push(a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y));
                }
            }
            xs.sort_by(f64::total_cmp);
            for w in xs.chunks(2) {
                if w.len() < 2 {
                    break;
                }
                for c in c0..c1 {
                    let x = self.x0 + (c as f64 + 0.5) * self.step;
                    if x >= w[0] && x < w[1] {
                        let i = r * self.nx + c;
                        m[i] = !m[i];
                    }
                }
            }
        }
    }

    /// The cells inside the rings, even-odd.
    pub fn rings(&self, rings: &[Polygon]) -> Vec<bool> {
        let mut m = vec![false; self.n()];
        for r in rings {
            self.toggle(&mut m, r);
        }
        m
    }
}

/// Euclidean distance (in cells) from every cell to the nearest cell where `m` is false
/// (Felzenszwalb and Huttenlocher); outside the raster counts as false. A cell that is false has
/// 0, its neighbour 1: the border between them lies half a cell before that.
pub fn edt(m: &[bool], nx: usize, ny: usize) -> Vec<f32> {
    edt_with(m, nx, ny, true)
}

/// As [`edt`], but outside the raster counts for nothing: the distance to the nearest false
/// cell of the raster itself (very large where it has none).
pub fn edt_open(m: &[bool], nx: usize, ny: usize) -> Vec<f32> {
    edt_with(m, nx, ny, false)
}

#[allow(clippy::needless_range_loop)]
fn edt_with(m: &[bool], nx: usize, ny: usize, border: bool) -> Vec<f32> {
    const INF: f64 = 1e18;
    fn dt1(f: &[f64], d: &mut [f64], v: &mut [usize], z: &mut [f64]) {
        let n = f.len();
        let mut k = 0usize;
        v[0] = 0;
        z[0] = -INF;
        z[1] = INF;
        for q in 1..n {
            loop {
                let p = v[k];
                let s = ((f[q] + (q * q) as f64) - (f[p] + (p * p) as f64)) / (2.0 * (q as f64 - p as f64));
                if s <= z[k] && k > 0 {
                    k -= 1;
                    continue;
                }
                if s <= z[k] {
                    // k == 0: replace the first parabola
                    v[0] = q;
                    z[0] = -INF;
                    z[1] = INF;
                    break;
                }
                k += 1;
                v[k] = q;
                z[k] = s;
                z[k + 1] = INF;
                break;
            }
        }
        k = 0;
        for q in 0..n {
            while z[k + 1] < q as f64 {
                k += 1;
            }
            let p = v[k];
            let dq = q as f64 - p as f64;
            d[q] = dq * dq + f[p];
        }
    }
    // pad by one cell on every side: false where the border counts
    let (w, h) = (nx + 2, ny + 2);
    let mut g = vec![if border { 0f64 } else { INF }; w * h];
    for r in 0..ny {
        for c in 0..nx {
            g[(r + 1) * w + c + 1] = 0.0;
        }
    }
    for r in 0..ny {
        for c in 0..nx {
            if m[r * nx + c] {
                g[(r + 1) * w + c + 1] = INF;
            }
        }
    }
    let len = w.max(h);
    let (mut f, mut d, mut v, mut z) = (vec![0f64; len], vec![0f64; len], vec![0usize; len], vec![0f64; len + 1]);
    for c in 0..w {
        for r in 0..h {
            f[r] = g[r * w + c];
        }
        dt1(&f[..h], &mut d[..h], &mut v, &mut z);
        for r in 0..h {
            g[r * w + c] = d[r];
        }
    }
    for r in 0..h {
        f[..w].copy_from_slice(&g[r * w..(r + 1) * w]);
        dt1(&f[..w], &mut d[..w], &mut v, &mut z);
        g[r * w..(r + 1) * w].copy_from_slice(&d[..w]);
    }
    let mut out = vec![0f32; nx * ny];
    for r in 0..ny {
        for c in 0..nx {
            out[r * nx + c] = g[(r + 1) * w + c + 1].sqrt() as f32;
        }
    }
    out
}

/// Labels the pieces of `m` (cells joined across a side): 0 where `m` is false, else the piece
/// number from 1. Returns the labels and the number of pieces.
pub fn pieces_of(m: &[bool], nx: usize, ny: usize) -> (Vec<u32>, usize) {
    let mut lab = vec![0u32; m.len()];
    let mut count = 0usize;
    let mut stack: Vec<usize> = Vec::new();
    for start in 0..m.len() {
        if !m[start] || lab[start] != 0 {
            continue;
        }
        count += 1;
        lab[start] = count as u32;
        stack.push(start);
        while let Some(i) = stack.pop() {
            let (c, r) = (i % nx, i / nx);
            let mut visit = |j: usize| {
                if m[j] && lab[j] == 0 {
                    lab[j] = count as u32;
                    stack.push(j);
                }
            };
            if c > 0 {
                visit(i - 1);
            }
            if c + 1 < nx {
                visit(i + 1);
            }
            if r > 0 {
                visit(i - nx);
            }
            if r + 1 < ny {
                visit(i + nx);
            }
        }
    }
    (lab, count)
}

/// Why a pin is or is not part of the area.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub enum Reach {
    /// The fill of the area touches the pad on its layer.
    Fill,
    /// Tracks, vias or other pads of the net lead from the pad to the area.
    Copper,
    /// The pad lies on a piece of fill that is not joined to the area (index into
    /// [`NetFill::islands`]).
    Island(usize),
    /// The net has an area on a layer of the pad and the pad lies in its outline, but no fill
    /// gets to it: copper of other nets is closer than clearance plus minimum width all around.
    Walled,
    /// As [`Reach::Walled`] for a pad with a thermal relief: none of its spokes ends in fill.
    NoSpoke,
    /// The pad lies outside the outlines of the net's areas on its layers.
    Outside,
    /// The net has no area on a layer of the pad: the pin needs a via or a track.
    OtherLayer,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct PinReach {
    pub pin: usize,
    pub name: String,
    pub reach: Reach,
    /// The group of copper the pad belongs to, as in [`Piece::group`]: a pad hangs on a piece
    /// of fill, directly or through tracks, when they have the same group.
    pub group: usize,
    /// Straight distance in mm from the pad centre to the nearest spot where a via of the net
    /// stands clear on every layer and lands in the fill of the area (-1: no such spot).
    pub site_dist: f64,
    /// For a pad that is not reached and is large enough to hold a via of the net (an exposed
    /// pad): the spot in the pad where a via stands clear on every layer and lands in the area
    /// on another layer. A via in a pad is a decision for the user, never taken by default.
    pub pad_site: Option<ViaSite>,
}

impl PinReach {
    pub fn reached(&self) -> bool {
        matches!(self.reach, Reach::Fill | Reach::Copper)
    }
}

/// A piece of fill on one layer.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Piece {
    pub layer: LayerId,
    /// mm².
    pub area: f64,
    /// A point of the piece.
    pub at: Point,
    /// Its widest spot: the diameter of the largest circle that fits, mm.
    pub widest: f64,
    /// Pads it touches.
    pub pins: Vec<usize>,
    /// The group of copper it belongs to (pieces of one group hang together).
    pub group: usize,
    /// Part of the area.
    pub main: bool,
    /// Holds nothing of the net: KiCad leaves such a piece out where the zone says so.
    pub floating: bool,
}

/// Where one via joins an island (or, set in a pad, that pad) to the area.
#[derive(Debug, Clone, serde::Serialize)]
pub struct ViaSite {
    pub at: Point,
    /// Room beyond what the via needs to the nearest copper of another net, hole or edge, mm.
    pub room: f64,
    /// Layer on which it stands in the island, and layer on which it lands in the area.
    pub from: LayerId,
    pub to: LayerId,
    /// It lands on a piece of fill that is not part of the area itself (a pour on a layer
    /// without pads of the net, or another island): that piece needs this second via to hang
    /// on the area.
    pub anchor: Option<Point>,
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub enum NoVia {
    /// The island is narrower than a via: its widest spot and the via diameter, mm.
    TooNarrow { widest: f64, need: f64 },
    /// Wide enough, but copper of another net, a hole or the edge is in the way on some layer
    /// under every spot.
    Blocked,
    /// A via has room, but no fill of the area lies above or below the island there.
    NothingOpposite,
}

/// A group of copper with pins that is not joined to the area.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Island {
    pub pins: Vec<usize>,
    pub names: Vec<String>,
    /// Layers on which it has fill.
    pub layers: Vec<LayerId>,
    /// Fill area in mm², without what lies on its pads.
    pub area: f64,
    /// Its widest spot off the pads: the diameter of the largest circle that fits, mm.
    pub widest: f64,
    pub via: Option<ViaSite>,
    pub no_via: Option<NoVia>,
}

/// The fill of one net and what it reaches.
#[derive(Debug, Clone, serde::Serialize)]
pub struct NetFill {
    pub net: NetId,
    pub raster: Raster,
    /// Per copper layer the fill, where the net has an area on it.
    #[serde(skip)]
    pub fill: Vec<Option<Vec<bool>>>,
    pub pieces: Vec<Piece>,
    /// The pads of the net (kept copper that touches no pad is not listed).
    pub pins: Vec<PinReach>,
    pub islands: Vec<Island>,
    /// Fill of the area in mm² (the main group), and of everything else.
    pub area_main: f64,
    pub area_rest: f64,
    /// Pads the area joins. Below two it is the largest piece of fill and no more: nothing is
    /// connected through it yet.
    pub main_pads: usize,
    /// Groups of copper of the net that hold a pad and are not the area: islands, and pads or
    /// routed pieces without fill. KiCad's ratsnest has one line for each (one less where the
    /// area itself holds no pad).
    pub groups_apart: usize,
}

impl NetFill {
    pub fn unreached(&self) -> impl Iterator<Item = &PinReach> {
        self.pins.iter().filter(|p| !p.reached())
    }
}

struct Groups {
    parent: Vec<usize>,
}

impl Groups {
    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]];
            x = self.parent[x];
        }
        x
    }
    fn join(&mut self, a: usize, b: usize) {
        let (a, b) = (self.find(a), self.find(b));
        if a != b {
            self.parent[a.max(b)] = a.min(b);
        }
    }
}

/// The two axes of a pad for its thermal spokes: along its longest side and across; a round
/// pad gets them at 45 degrees, as KiCad draws them.
fn spoke_axes(s: &Shape) -> [Point; 2] {
    let u = match s {
        Shape::Circle { .. } => pt(std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::FRAC_1_SQRT_2),
        Shape::Capsule { a, b, .. } if a.dist(*b) > 1e-9 => (*b - *a).norm(),
        Shape::Capsule { .. } => pt(1.0, 0.0),
        Shape::Polygon(p) => p
            .edges()
            .max_by(|x, y| x.0.dist(x.1).total_cmp(&y.0.dist(y.1)))
            .filter(|e| e.0.dist(e.1) > 1e-9)
            .map_or(pt(1.0, 0.0), |e| (e.1 - e.0).norm()),
    };
    [u, u.perp()]
}

/// How far the shape reaches from `c` along the unit vector `d`.
fn extent(s: &Shape, c: Point, d: Point) -> f64 {
    match s {
        Shape::Circle { c: m, r } => (*m - c).dot(d) + r,
        Shape::Capsule { a, b, r } => (*a - c).dot(d).max((*b - c).dot(d)) + r,
        Shape::Polygon(p) => p.pts.iter().map(|q| (*q - c).dot(d)).fold(0.0, f64::max),
    }
}

/// The fill of the areas of `net` on the copper of `board` as it is (pads, kept tracks and
/// vias), and what it reaches. `areas` holds the areas of all nets: those of other nets cut
/// the ones of `net` where they win by priority.
pub fn fill_net(board: &Board, areas: &[Area], net: NetId, o: &Options) -> NetFill {
    let nl = board.layers.len();
    let own_areas: Vec<&Area> = areas.iter().filter(|a| a.net == net && !a.rings.is_empty()).collect();
    let min_w = own_areas
        .iter()
        .map(|a| a.min_width)
        .filter(|w| *w > 0.0)
        .fold(f64::INFINITY, f64::min);
    let min_w = if min_w.is_finite() { min_w } else { 0.0 };
    let g = Raster::for_width(board.bbox().grow(1.0), o.step, min_w, o.max_cells);
    let (n, s, nx, ny) = (g.n(), g.step, g.nx, g.ny);
    let class = board.class_of(net);
    let (via_d, via_drill) = o.via.unwrap_or((class.via_diameter, class.via_drill));

    // the board and the distance to its edge
    let mut inboard = vec![board.outline.is_empty(); n];
    for p in &board.outline {
        g.toggle(&mut inboard, p);
    }
    let edge = edt(&inboard, nx, ny);
    let edge_need = |slack: f64| ((board.rules.edge_clearance + slack).max(0.0) / s + 0.5) as f32;
    let gap_to = |other: Option<NetId>| board.clearance(Some(net), other).max(board.item_gap(net, other));
    // cells closed to copper of the net on a layer: within the clearance of other nets' copper
    // (at least `least`), of a hole, of the edge
    let closed = |l: LayerId, least: f64, for_via: bool| -> Vec<bool> {
        let mut k = vec![false; n];
        let slack = if for_via { 0.0 } else { o.slack };
        for ob in &board.obstacles {
            if !ob.layers.has(l) {
                continue;
            }
            let need = match ob.kind {
                ObstacleKind::Hole => (board.hole_gap(net) + slack).max(0.0),
                ObstacleKind::Pad | ObstacleKind::Track | ObstacleKind::Via | ObstacleKind::Graphic
                    if ob.net != Some(net) =>
                {
                    (gap_to(ob.net).max(least) + slack).max(0.0)
                }
                ObstacleKind::Keepout {
                    vias: true,
                    mask: false,
                    ..
                } if for_via => 0.0,
                _ => continue,
            };
            g.mark(&mut k, &ob.shape, need);
        }
        if !for_via {
            for (ls, poly) in &o.no_pour {
                if ls.has(l) {
                    let m = g.rings(std::slice::from_ref(poly));
                    for i in 0..n {
                        k[i] |= m[i];
                    }
                }
            }
        }
        let edge_need = edge_need(slack);
        for i in 0..n {
            if !inboard[i] || edge[i] < edge_need {
                k[i] = true;
            }
        }
        k
    };
    let connect_of = |pin: usize, hole: bool, a: &Area| -> Connect {
        match o.pad_connect.get(&pin).copied().unwrap_or(a.connect) {
            Connect::ThermalTht if hole => Connect::Thermal,
            Connect::ThermalTht => Connect::Solid,
            c => c,
        }
    };
    let real_pin = |p: usize| !board.pins[p].footprint.is_empty();
    let pins: Vec<usize> = board.nets[net as usize]
        .pins
        .iter()
        .copied()
        .filter(|&p| real_pin(p))
        .collect();

    // ---- the fill per layer
    let mut fill: Vec<Option<Vec<bool>>> = vec![None; nl];
    // pads with a thermal relief none of whose spokes ends in fill, by layer
    let mut no_spoke: Vec<(usize, LayerId)> = Vec::new();
    for l in 0..nl as LayerId {
        let mine: Vec<&&Area> = own_areas.iter().filter(|a| a.layer == l).collect();
        if mine.is_empty() {
            continue;
        }
        let mut layer_fill = vec![false; n];
        for a in mine {
            let k = closed(l, a.clearance, false);
            let mut open = g.rings(&a.rings);
            for i in 0..n {
                open[i] &= !k[i];
            }
            // areas of other nets that KiCad fills first
            for other in areas.iter().filter(|x| x.layer == l && x.net != net && x.wins_over(a)) {
                let inv: Vec<bool> = g.rings(&other.rings).iter().map(|&x| !x).collect();
                let d = edt(&inv, nx, ny);
                let need = a.clearance.max(other.clearance).max(gap_to(Some(other.net))) + o.slack;
                let need = (need.max(0.0) / s + 0.5) as f32;
                for i in 0..n {
                    if d[i] < need {
                        open[i] = false;
                    }
                }
            }
            // the pads of the net: a gap around those with a thermal relief
            let mut base = open.clone();
            let mut thermal: Vec<(usize, usize)> = Vec::new();
            let thermal_gap = (a.thermal_gap + o.slack).max(0.0);
            for &p in &pins {
                for &oi in &board.pins[p].obstacles {
                    let ob = &board.obstacles[oi];
                    if ob.kind != ObstacleKind::Pad || !ob.layers.has(l) {
                        continue;
                    }
                    let gap = match connect_of(p, ob.hole.is_some(), a) {
                        Connect::Solid | Connect::ThermalTht => continue,
                        Connect::Thermal => {
                            thermal.push((p, oi));
                            thermal_gap
                        }
                        Connect::None => (a.clearance.max(gap_to(Some(net))) + o.slack).max(0.0),
                    };
                    g.each_in(&ob.shape, gap, |i| base[i] = false);
                }
            }
            // nothing narrower than the minimum width
            let r = (a.min_width / 2.0 / s) as f32;
            let d1 = edt(&base, nx, ny);
            let not_core: Vec<bool> = d1.iter().map(|&d| d < r + 0.5 - 1e-3).collect();
            let d2 = edt(&not_core, nx, ny);
            let mut f: Vec<bool> = (0..n).map(|i| base[i] && d2[i] <= r + 1e-3).collect();
            // the spokes that end in fill
            let mut spokes: Vec<usize> = Vec::new();
            for &(p, oi) in &thermal {
                let ob = &board.obstacles[oi];
                let c = ob.shape.bbox().center();
                if !a.contains(c) {
                    continue;
                }
                let mut any = false;
                for axis in spoke_axes(&ob.shape) {
                    for d in [axis, -axis] {
                        let from = extent(&ob.shape, c, d);
                        let to = from + thermal_gap + 1.5 * s;
                        // the way across the gap is open, and it ends in fill
                        let steps = ((to - from) / (s * 0.5)).ceil() as usize;
                        let clear = (0..=steps).all(|k| {
                            let q = c + d * (from + (to - from) * k as f64 / steps.max(1) as f64);
                            g.cell(q).is_some_and(|i| open[i])
                        });
                        if !clear || !g.cell(c + d * to).is_some_and(|i| f[i]) {
                            continue;
                        }
                        any = true;
                        let bar = Shape::Capsule {
                            a: c,
                            b: c + d * to,
                            r: 0.0,
                        };
                        // (a bar with square sides; its round end lies in fill anyway)
                        g.each_in(&bar, a.spoke_width / 2.0, |i| {
                            if open[i] {
                                spokes.push(i);
                            }
                        });
                    }
                }
                if !any {
                    no_spoke.push((p, l));
                }
            }
            for i in spokes {
                f[i] = true;
            }
            for i in 0..n {
                layer_fill[i] |= f[i];
            }
        }
        fill[l as usize] = Some(layer_fill);
    }

    // ---- the copper of the net per layer, its pieces, and the groups over all layers
    let mut labels: Vec<Vec<u32>> = Vec::with_capacity(nl);
    let mut offset: Vec<usize> = Vec::with_capacity(nl + 1);
    let mut total = 0usize;
    let mut pad_cells = vec![false; n];
    for l in 0..nl as LayerId {
        let mut m = fill[l as usize].clone().unwrap_or_else(|| vec![false; n]);
        for ob in &board.obstacles {
            if ob.net == Some(net)
                && ob.layers.has(l)
                && matches!(ob.kind, ObstacleKind::Pad | ObstacleKind::Track | ObstacleKind::Via)
            {
                g.mark(&mut m, &ob.shape, 0.0);
                if ob.kind == ObstacleKind::Pad {
                    g.mark(&mut pad_cells, &ob.shape, 0.0);
                }
            }
        }
        let (lab, count) = pieces_of(&m, nx, ny);
        labels.push(lab);
        offset.push(total);
        total += count;
    }
    offset.push(total);
    let mut groups = Groups {
        parent: (0..total).collect(),
    };
    // the node of an obstacle's copper on a layer
    let node_of = |oi: usize, l: LayerId| -> Option<usize> {
        let ob = &board.obstacles[oi];
        let lab = &labels[l as usize];
        let at = |i: usize| (lab[i] != 0).then(|| offset[l as usize] + lab[i] as usize - 1);
        if let Some(x) = g
            .cell(ob.shape.bbox().center())
            .filter(|&i| ob.shape.dist_to_point(g.center(i)) <= 1e-9)
            && let Some(node) = at(x)
        {
            return Some(node);
        }
        let mut found = None;
        g.each_in(&ob.shape, 0.0, |i| {
            if found.is_none() {
                found = at(i);
            }
        });
        found
    };
    for (oi, ob) in board.obstacles.iter().enumerate() {
        if ob.net != Some(net) || !matches!(ob.kind, ObstacleKind::Pad | ObstacleKind::Via) {
            continue;
        }
        let mut first: Option<usize> = None;
        for l in ob.layers.iter().filter(|&l| (l as usize) < nl) {
            if let Some(node) = node_of(oi, l) {
                match first {
                    None => first = Some(node),
                    Some(f) => groups.join(f, node),
                }
            }
        }
    }
    // a pin of several shapes is one piece of copper
    let mut pin_node: HashMap<usize, usize> = HashMap::new();
    for &p in &board.nets[net as usize].pins {
        let mut first: Option<usize> = None;
        for &oi in &board.pins[p].obstacles {
            for l in board.obstacles[oi].layers.iter().filter(|&l| (l as usize) < nl) {
                if let Some(node) = node_of(oi, l) {
                    match first {
                        None => first = Some(node),
                        Some(f) => groups.join(f, node),
                    }
                }
            }
        }
        if let Some(f) = first {
            pin_node.insert(p, f);
        }
    }
    // fill per group (what lies on a pad is the pad's copper already), and which groups hold
    // something of the net
    let mut group_fill: HashMap<usize, usize> = HashMap::new();
    for l in 0..nl {
        if let Some(f) = &fill[l] {
            let mut per_label: HashMap<u32, usize> = HashMap::new();
            for i in 0..n {
                if f[i] && !pad_cells[i] {
                    *per_label.entry(labels[l][i]).or_default() += 1;
                }
            }
            for (lab, cells) in per_label {
                *group_fill.entry(groups.find(offset[l] + lab as usize - 1)).or_default() += cells;
            }
        }
    }
    let mut holds: HashMap<usize, Vec<usize>> = HashMap::new();
    for &p in &board.nets[net as usize].pins {
        if let Some(&node) = pin_node.get(&p) {
            holds.entry(groups.find(node)).or_default().push(p);
        }
    }
    // the area: the group with the most fill among those that join at least two pads. Where no
    // group does (a plane before its first via, a pour that copper of other nets cuts off from
    // every pad but one), the largest piece is taken: that is what the pins are to reach, and a
    // sliver at one pad is not
    let pads_of = |gr: usize| holds.get(&gr).map_or(0, |v| v.iter().filter(|&&p| real_pin(p)).count());
    let main: Option<usize> = group_fill
        .iter()
        .map(|(&gr, &cells)| {
            let pads = pads_of(gr);
            (pads >= 2, cells, pads, std::cmp::Reverse(gr))
        })
        .max()
        .map(|x| x.3.0);
    let main_pads = main.map_or(0, pads_of);
    let cell_mm2 = s * s;

    // ---- pieces of fill
    let mut pieces: Vec<Piece> = Vec::new();
    // per layer: piece index (into `pieces`) of every fill cell, and the distance inside the fill
    let mut piece_at: Vec<Option<(Vec<u32>, usize)>> = vec![None; nl];
    let mut depth: Vec<Option<Vec<f32>>> = vec![None; nl];
    for l in 0..nl {
        let Some(f) = &fill[l] else { continue };
        let (lab, count) = pieces_of(f, nx, ny);
        let d = edt(f, nx, ny);
        let base = pieces.len();
        let mut cells = vec![0usize; count];
        let mut sample = vec![usize::MAX; count];
        let mut deep = vec![0f32; count];
        for i in 0..n {
            if lab[i] != 0 {
                let k = lab[i] as usize - 1;
                cells[k] += 1;
                if d[i] > deep[k] {
                    deep[k] = d[i];
                    sample[k] = i;
                }
            }
        }
        for k in 0..count {
            let group = groups.find(offset[l] + labels[l][sample[k]] as usize - 1);
            pieces.push(Piece {
                layer: l as LayerId,
                area: cells[k] as f64 * cell_mm2,
                at: g.center(sample[k]),
                widest: (2.0 * (deep[k] as f64 - 0.5) * s).max(0.0),
                pins: Vec::new(),
                group,
                main: Some(group) == main,
                floating: !holds.contains_key(&group),
            });
        }
        piece_at[l] = Some((lab, base));
        depth[l] = Some(d);
    }
    // the pads each piece touches
    for &p in &pins {
        for &oi in &board.pins[p].obstacles {
            let ob = &board.obstacles[oi];
            if ob.kind != ObstacleKind::Pad {
                continue;
            }
            for l in ob.layers.iter().filter(|&l| (l as usize) < nl) {
                let Some((lab, base)) = &piece_at[l as usize] else {
                    continue;
                };
                g.each_in(&ob.shape, 0.0, |i| {
                    let (c, r) = (i % nx, i / nx);
                    let mut near = [i, i, i, i, i];
                    if c > 0 {
                        near[1] = i - 1;
                    }
                    if c + 1 < nx {
                        near[2] = i + 1;
                    }
                    if r > 0 {
                        near[3] = i - nx;
                    }
                    if r + 1 < ny {
                        near[4] = i + nx;
                    }
                    for j in near {
                        if lab[j] != 0 {
                            let k = base + lab[j] as usize - 1;
                            if !pieces[k].pins.contains(&p) {
                                pieces[k].pins.push(p);
                            }
                        }
                    }
                });
            }
        }
    }
    // ---- where a via of the net has room: clear on every layer, off every hole
    let need_v = (via_d / 2.0 / s + 0.5) as f32;
    let mut room = vec![f32::INFINITY; n];
    for l in 0..nl as LayerId {
        let open: Vec<bool> = closed(l, 0.0, true).iter().map(|&x| !x).collect();
        let d = edt(&open, nx, ny);
        for i in 0..n {
            room[i] = room[i].min(d[i]);
        }
    }
    for ob in &board.obstacles {
        if let Some(h) = ob.hole {
            let c = ob.shape.bbox().center();
            let keep = h + board.rules.hole_to_hole + via_drill / 2.0;
            g.each_in(&Shape::Circle { c, r: keep }, 0.0, |i| room[i] = 0.0);
        }
    }
    // (a via in a pad of the net is looked for apart, for the pads nothing else reaches)
    let room_in_pads = room.clone();
    for i in 0..n {
        if pad_cells[i] {
            room[i] = 0.0;
        }
    }
    let group_at = |groups: &mut Groups, l: usize, i: usize| -> Option<usize> {
        (labels[l][i] != 0).then(|| groups.find(offset[l] + labels[l][i] as usize - 1))
    };
    let is_fill = |l: usize, i: usize| fill[l].as_ref().is_some_and(|f| f[i]);
    // groups of fill apart from the area which one via joins to it (the pour on a layer without
    // pads of the net, or another island): group -> the best cell for that via
    let mut anchors: HashMap<usize, (f32, usize)> = HashMap::new();
    if let Some(m) = main {
        for l in 0..nl {
            if fill[l].is_none() {
                continue;
            }
            for i in 0..n {
                if room[i] < need_v || !is_fill(l, i) {
                    continue;
                }
                let Some(gr) = group_at(&mut groups, l, i) else {
                    continue;
                };
                if gr == m {
                    continue;
                }
                let on_main = (0..nl).any(|l2| l2 != l && is_fill(l2, i) && group_at(&mut groups, l2, i) == Some(m));
                if on_main && anchors.get(&gr).is_none_or(|a| room[i] > a.0) {
                    anchors.insert(gr, (room[i], i));
                }
            }
        }
    }
    // spots where a via lands in the area, or in a piece that hangs on it through its anchor
    let mut lands = vec![false; n];
    if let Some(m) = main {
        for l in 0..nl {
            if fill[l].is_none() {
                continue;
            }
            for i in 0..n {
                if room[i] >= need_v
                    && is_fill(l, i)
                    && group_at(&mut groups, l, i).is_some_and(|gr| gr == m || anchors.contains_key(&gr))
                {
                    lands[i] = true;
                }
            }
        }
    }
    // the way to a site is measured over the whole raster: its border is no site
    let to_site = {
        let inv: Vec<bool> = lands.iter().map(|&x| !x).collect();
        edt_open(&inv, nx, ny)
    };
    let any_site = lands.iter().any(|&x| x);

    // ---- islands
    let mut island_groups: Vec<usize> = holds
        .iter()
        .filter(|(gr, ps)| Some(**gr) != main && group_fill.contains_key(gr) && ps.iter().any(|&p| real_pin(p)))
        .map(|(&gr, _)| gr)
        .collect();
    island_groups.sort_unstable();
    let mut islands: Vec<Island> = Vec::new();
    for &gr in &island_groups {
        let mut ps: Vec<usize> = holds[&gr].iter().copied().filter(|&p| real_pin(p)).collect();
        ps.sort_unstable();
        let mut best: Option<(f32, usize, usize, usize, Option<usize>)> = None;
        let (mut fits, mut widest, mut cells) = (false, 0f32, 0usize);
        let mut layers = Vec::new();
        for l in 0..nl {
            if fill[l].is_none() {
                continue;
            }
            let d = depth[l].as_ref().unwrap();
            let mut on_layer = false;
            for i in 0..n {
                if !is_fill(l, i) || pad_cells[i] || group_at(&mut groups, l, i) != Some(gr) {
                    continue;
                }
                on_layer = true;
                cells += 1;
                widest = widest.max(d[i]);
                if room[i] < need_v {
                    continue;
                }
                fits = true;
                if !lands[i] {
                    continue;
                }
                // the layer on which it lands: in the area itself if it can
                let mut to: Option<(usize, Option<usize>)> = None;
                for l2 in (0..nl).filter(|&l2| l2 != l && is_fill(l2, i)) {
                    let Some(g2) = group_at(&mut groups, l2, i) else {
                        continue;
                    };
                    if Some(g2) == main {
                        to = Some((l2, None));
                        break;
                    }
                    if to.is_none()
                        && g2 != gr
                        && let Some(a) = anchors.get(&g2)
                    {
                        to = Some((l2, Some(a.1)));
                    }
                }
                let Some((to, anchor)) = to else { continue };
                // a site that needs no second via wins, then the one with the most room
                let better = best.is_none_or(|b| {
                    let (old, new) = (b.4.is_none(), anchor.is_none());
                    (new && !old) || (new == old && room[i] > b.0)
                });
                if better {
                    best = Some((room[i], i, l, to, anchor));
                }
            }
            if on_layer {
                layers.push(l as LayerId);
            }
        }
        let widest = (2.0 * (widest as f64 - 0.5) * s).max(0.0);
        let via = best.map(|(r, i, from, to, anchor)| ViaSite {
            at: g.center(i),
            room: (r - need_v) as f64 * s,
            from: from as LayerId,
            to: to as LayerId,
            anchor: anchor.map(|a| g.center(a)),
        });
        let no_via = match (&via, fits) {
            (Some(_), _) => None,
            (None, true) => Some(NoVia::NothingOpposite),
            (None, false) if widest < via_d => Some(NoVia::TooNarrow { widest, need: via_d }),
            (None, false) => Some(NoVia::Blocked),
        };
        islands.push(Island {
            names: ps.iter().map(|&p| board.pins[p].name.clone()).collect(),
            pins: ps,
            layers,
            area: cells as f64 * cell_mm2,
            widest,
            via,
            no_via,
        });
    }

    // ---- the pins
    let mut out_pins: Vec<PinReach> = Vec::new();
    for &p in &pins {
        let pin = &board.pins[p];
        let group = pin_node.get(&p).map(|&x| groups.find(x));
        let reach = if group.is_some() && group == main {
            let touches = pieces.iter().any(|pc| pc.main && pc.pins.contains(&p));
            if touches { Reach::Fill } else { Reach::Copper }
        } else if let Some(k) = group.and_then(|gr| island_groups.iter().position(|&x| x == gr)) {
            Reach::Island(k)
        } else {
            let on_layer: Vec<&&Area> = own_areas.iter().filter(|a| pin.layers.has(a.layer)).collect();
            if on_layer.is_empty() {
                Reach::OtherLayer
            } else if !on_layer.iter().any(|a| a.contains(pin.pos)) {
                Reach::Outside
            } else if no_spoke.iter().any(|&(q, _)| q == p) {
                Reach::NoSpoke
            } else {
                Reach::Walled
            }
        };
        let site_dist = match g.cell(pin.pos) {
            Some(i) if any_site => to_site[i] as f64 * s,
            _ => -1.0,
        };
        // a via in the pad: the whole via on the pad, clear on every layer, in the area on a
        // layer the pad is not on (or in a piece that one more via joins to the area); a spot
        // that needs no second via wins, then the one nearest the middle of the pad
        let mut pad_site: Option<(bool, f64, usize, LayerId, usize)> = None;
        if !matches!(reach, Reach::Fill | Reach::Copper)
            && let Some(m) = main
        {
            for &oi in &pin.obstacles {
                let ob = &board.obstacles[oi];
                let Some(from) = ob.layers.iter().find(|&l| (l as usize) < nl) else {
                    continue;
                };
                if ob.kind != ObstacleKind::Pad || ob.hole.is_some() {
                    continue;
                }
                g.each_in(&ob.shape, 0.0, |i| {
                    let c = g.center(i);
                    let d = c.dist(pin.pos);
                    if room_in_pads[i] < need_v {
                        return;
                    }
                    let on_pad = (0..8).all(|k| {
                        let a = k as f64 * std::f64::consts::FRAC_PI_4;
                        let q = c + pt(crate::math::cos(a), crate::math::sin(a)) * (via_d / 2.0);
                        ob.shape.dist_to_point(q) <= 1e-9
                    });
                    if !on_pad {
                        return;
                    }
                    for l2 in (0..nl).filter(|&l2| !ob.layers.has(l2 as LayerId) && is_fill(l2, i)) {
                        let second = match group_at(&mut groups, l2, i) {
                            Some(gr) if gr == m => None,
                            Some(gr) => match anchors.get(&gr) {
                                Some(a) => Some(a.1),
                                None => continue,
                            },
                            None => continue,
                        };
                        let key = (second.is_some(), d);
                        if pad_site.is_none_or(|b| key < (b.0, b.1)) {
                            pad_site = Some((second.is_some(), d, i, from, l2 * n + second.unwrap_or(0)));
                        }
                    }
                });
            }
        }
        let pad_site = pad_site.map(|(second, _, i, from, packed)| ViaSite {
            at: g.center(i),
            room: (room_in_pads[i] - need_v) as f64 * s,
            from,
            to: (packed / n) as LayerId,
            anchor: second.then(|| g.center(packed % n)),
        });
        out_pins.push(PinReach {
            pin: p,
            name: pin.name.clone(),
            reach,
            group: group.unwrap_or(usize::MAX),
            site_dist,
            pad_site,
        });
    }
    // floating pieces go where the zone says so, now that the via sites are found: a piece that
    // a via can anchor counted for those (and never the area itself: a plane that nothing
    // reaches yet is what the pins are to reach)
    for l in 0..nl {
        let Some((lab, base)) = &piece_at[l] else { continue };
        let limit = own_areas
            .iter()
            .filter(|a| a.layer as usize == l)
            .map(|a| a.island_min)
            .fold(f64::INFINITY, f64::min);
        let f = fill[l].as_mut().unwrap();
        for i in 0..n {
            if lab[i] != 0 {
                let pc = &pieces[base + lab[i] as usize - 1];
                if pc.floating && !pc.main && pc.area < limit {
                    f[i] = false;
                }
            }
        }
    }

    let groups_apart = holds
        .iter()
        .filter(|(gr, ps)| Some(**gr) != main && ps.iter().any(|&p| real_pin(p)))
        .count();
    let area_main: f64 = pieces.iter().filter(|p| p.main).map(|p| p.area).sum();
    let area_rest: f64 = pieces.iter().filter(|p| !p.main).map(|p| p.area).sum();
    NetFill {
        net,
        raster: g,
        fill,
        pieces,
        pins: out_pins,
        islands,
        area_main,
        area_rest,
        main_pads,
        groups_apart,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{LayerSet, Net, Obstacle, Pin};
    use crate::geom::roundrect_polygon;

    /// A 20 x 20 mm board of two layers with a 1 mm square pad per entry (net, x, y, layer).
    fn board(pads: &[(&str, f64, f64, u8)]) -> Board {
        let mut b = crate::topo::tests::two_pad_board();
        b.outline = vec![Polygon::new(vec![pt(0., 0.), pt(20., 0.), pt(20., 20.), pt(0., 20.)])];
        b.nets.clear();
        b.pins.clear();
        b.obstacles.clear();
        for (i, &(net, x, y, layer)) in pads.iter().enumerate() {
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
                layers: LayerSet::single(layer),
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
                layers: LayerSet::single(layer),
                obstacles: vec![i],
                size: (1.0, 1.0),
            });
        }
        b
    }

    fn track(b: &mut Board, net: &str, layer: u8, pts: &[(f64, f64)], width: f64) {
        let net = b.nets.iter().position(|n| n.name == net).map(|n| n as NetId);
        for w in pts.windows(2) {
            b.obstacles.push(Obstacle {
                shape: Shape::Capsule {
                    a: pt(w[0].0, w[0].1),
                    b: pt(w[1].0, w[1].1),
                    r: width / 2.0,
                },
                layers: LayerSet::single(layer),
                net,
                kind: ObstacleKind::Track,
                hole: None,
                clearance: 0.2,
                pin: None,
                locked: false,
            });
        }
    }

    fn via(b: &mut Board, net: &str, x: f64, y: f64) {
        let net = b.nets.iter().position(|n| n.name == net).map(|n| n as NetId);
        b.obstacles.push(Obstacle {
            shape: Shape::Circle { c: pt(x, y), r: 0.3 },
            layers: LayerSet::all(2),
            net,
            kind: ObstacleKind::Via,
            hole: Some(0.15),
            clearance: 0.2,
            pin: None,
            locked: false,
        });
    }

    fn over_board(b: &Board, net: &str, layer: u8) -> Area {
        let net = b.nets.iter().position(|n| n.name == net).unwrap() as NetId;
        Area::pour(net, layer, vec![b.outline[0].clone()])
    }

    fn solid(mut a: Area) -> Area {
        a.connect = Connect::Solid;
        a
    }

    #[test]
    fn the_distance_transform_counts_cells_to_the_nearest_false_one() {
        // a 5 x 1 strip of true cells in a 7 x 3 raster
        let mut m = vec![false; 21];
        for c in 1..6 {
            m[7 + c] = true;
        }
        let d = edt(&m, 7, 3);
        assert_eq!(&d[7..14], &[0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0]);
        // a 5 x 5 block: the middle is 3 cells from the nearest false one
        let d = edt(&[true; 25], 5, 5);
        assert_eq!(d[12], 3.0);
    }

    #[test]
    fn an_empty_board_fills_up_to_the_edge_clearance() {
        let b = board(&[("GND", 5., 5., 0)]);
        let f = fill_net(&b, &[solid(over_board(&b, "GND", 0))], 0, &Options::default());
        assert_eq!(f.pieces.len(), 1);
        // 20 mm less 0.3 mm on each side (a cell counts when its centre does: half a cell more),
        // corners rounded by the opening
        let side = 19.4 + f.raster.step;
        assert!((f.area_main - side * side).abs() < 0.5, "{}", f.area_main);
        assert_eq!(f.pins[0].reach, Reach::Fill);
        assert!(f.islands.is_empty());
    }

    #[test]
    fn a_track_of_another_net_across_the_board_cuts_the_fill_in_two() {
        let mut b = board(&[
            ("GND", 5., 5., 0),
            ("GND", 15., 5., 0),
            ("S", 10., 1., 0),
            ("S", 10., 19., 0),
            ("GND", 5., 15., 0),
        ]);
        track(&mut b, "S", 0, &[(10., 1.), (10., 19.)], 0.2);
        let f = fill_net(&b, &[solid(over_board(&b, "GND", 0))], 0, &Options::default());
        // the cut reaches the edge clearance at both ends: two pieces, the left with two pads
        // is the area
        assert_eq!(f.pieces.len(), 2);
        assert_eq!(f.main_pads, 2);
        assert_eq!(f.islands.len(), 1);
        assert_eq!(f.unreached().count(), 1);
        // one layer only: nothing to land on
        assert_eq!(f.islands[0].no_via, Some(NoVia::NothingOpposite));
        // the same pour on the other layer holds nothing of the net: the island hangs on the
        // area through that piece with two vias, its own and one that anchors the piece
        let areas = [solid(over_board(&b, "GND", 0)), solid(over_board(&b, "GND", 1))];
        let f = fill_net(&b, &areas, 0, &Options::default());
        let site = f.islands[0].via.as_ref().expect("a via site");
        let anchor = site.anchor.expect("an anchor");
        assert_eq!(f.islands.len(), 1);
        assert!((site.at.x < 10.0) != (anchor.x < 10.0), "{site:?}");
        // with the same pour on the other layer, which a via joins to the first pad's side, one
        // more via joins the island
        via(&mut b, "GND", 5.0, 8.0);
        let areas = [solid(over_board(&b, "GND", 0)), solid(over_board(&b, "GND", 1))];
        let f = fill_net(&b, &areas, 0, &Options::default());
        assert_eq!(f.islands.len(), 1);
        assert_eq!(f.islands[0].pins, vec![1]);
        let site = f.islands[0].via.as_ref().expect("a via site");
        assert_eq!((site.from, site.to), (0, 1));
        // and the board with that via has no island
        via(&mut b, "GND", site.at.x, site.at.y);
        let f = fill_net(&b, &areas, 0, &Options::default());
        assert!(f.islands.is_empty());
        assert_eq!(f.unreached().count(), 0);
        assert!(f.pins.iter().any(|p| p.reach == Reach::Fill));
    }

    #[test]
    fn where_the_fill_joins_no_two_pads_its_largest_piece_is_the_area() {
        // one pad on each side of the cut, and the same pour on the other layer: no piece joins
        // two pads, so the largest piece (the other layer, whole) is what the pads are to reach
        let mut b = board(&[
            ("GND", 5., 5., 0),
            ("GND", 15., 5., 0),
            ("S", 8., 1., 0),
            ("S", 8., 19., 0),
        ]);
        track(&mut b, "S", 0, &[(8., 1.), (8., 19.)], 0.2);
        let areas = [solid(over_board(&b, "GND", 0)), solid(over_board(&b, "GND", 1))];
        let f = fill_net(&b, &areas, 0, &Options::default());
        assert_eq!((f.main_pads, f.groups_apart), (0, 2));
        let main: Vec<_> = f.pieces.iter().filter(|p| p.main).collect();
        assert_eq!((main.len(), main[0].layer), (1, 1));
        assert_eq!(f.unreached().count(), 2);
        // each side is an island with a via of its own, and none needs a second one
        assert_eq!(f.islands.len(), 2);
        assert!(
            f.islands
                .iter()
                .all(|i| i.via.as_ref().is_some_and(|v| v.anchor.is_none()))
        );
        // on one layer the larger side is taken, and it joins one pad
        let f = fill_net(&b, &areas[..1], 0, &Options::default());
        assert_eq!(f.main_pads, 1);
        assert_eq!(f.pins[1].reach, Reach::Fill);
        assert_eq!(f.pins[0].reach, Reach::Island(0));
    }

    #[test]
    fn slack_moves_every_clearance_of_the_fill() {
        // 0.2 mm of fill is left between the tracks: exactly the minimum width. Half a cell
        // more clearance on each side takes it away, half a cell less keeps it
        let x = 10.0 + 0.6 / 2.0 + 0.1;
        let mut b = board(&[("GND", 5., 10., 0), ("S", 20.0 - x, 1., 0), ("T", x, 1., 0)]);
        track(&mut b, "S", 0, &[(20.0 - x, 1.), (20.0 - x, 19.)], 0.2);
        track(&mut b, "T", 0, &[(x, 1.), (x, 19.)], 0.2);
        let at = |slack: f64| {
            let o = Options {
                slack,
                ..Default::default()
            };
            let f = fill_net(&b, &[solid(over_board(&b, "GND", 0))], 0, &o);
            f.fill[0].as_ref().unwrap()[f.raster.cell(pt(10.0, 10.0)).unwrap()]
        };
        let step = fill_net(&b, &[solid(over_board(&b, "GND", 0))], 0, &Options::default())
            .raster
            .step;
        assert!(at(-step / 2.0));
        assert!(!at(step / 2.0));
    }

    #[test]
    fn a_strip_narrower_than_the_minimum_width_is_no_fill() {
        // two tracks of another net leave 0.5 mm between their copper at x = 10: with 0.2 mm
        // clearance on both sides 0.1 mm of fill would be left, under the 0.2 mm minimum
        let mut b = board(&[("GND", 5., 10., 0), ("S", 9.65, 1., 0), ("T", 10.35, 1., 0)]);
        track(&mut b, "S", 0, &[(9.65, 1.), (9.65, 19.)], 0.2);
        track(&mut b, "T", 0, &[(10.35, 1.), (10.35, 19.)], 0.2);
        let f = fill_net(&b, &[solid(over_board(&b, "GND", 0))], 0, &Options::default());
        let i = f.raster.cell(pt(10.0, 10.0)).unwrap();
        assert!(!f.fill[0].as_ref().unwrap()[i]);
        // 0.3 mm more between them: 0.4 mm of fill, which stays
        let mut b = board(&[("GND", 5., 10., 0), ("S", 9.5, 1., 0), ("T", 10.5, 1., 0)]);
        track(&mut b, "S", 0, &[(9.5, 1.), (9.5, 19.)], 0.2);
        track(&mut b, "T", 0, &[(10.5, 1.), (10.5, 19.)], 0.2);
        let f = fill_net(&b, &[solid(over_board(&b, "GND", 0))], 0, &Options::default());
        let i = f.raster.cell(pt(10.0, 10.0)).unwrap();
        assert!(f.fill[0].as_ref().unwrap()[i]);
    }

    #[test]
    fn the_minimum_width_is_exact_along_the_axes() {
        // fill left between two tracks: gap between the copper less twice the clearance
        let left = |gap: f64| {
            let x = 10.0 + gap / 2.0 + 0.1;
            let mut b = board(&[("GND", 5., 10., 0), ("S", 20.0 - x, 1., 0), ("T", x, 1., 0)]);
            track(&mut b, "S", 0, &[(20.0 - x, 1.), (20.0 - x, 19.)], 0.2);
            track(&mut b, "T", 0, &[(x, 1.), (x, 19.)], 0.2);
            let f = fill_net(&b, &[solid(over_board(&b, "GND", 0))], 0, &Options::default());
            let i = f.raster.cell(pt(10.0, 10.0)).unwrap();
            f.fill[0].as_ref().unwrap()[i]
        };
        // 0.2 mm minimum width: a strip of 0.17 mm goes, one of 0.23 mm stays
        assert!(!left(0.4 + 0.17));
        assert!(left(0.4 + 0.23));
    }

    #[test]
    fn a_pad_walled_in_by_other_nets_is_not_reached() {
        // a ring of tracks of another net around the second pad
        let mut b = board(&[("GND", 5., 5., 0), ("GND", 15., 15., 0), ("S", 13., 13., 0)]);
        track(
            &mut b,
            "S",
            0,
            &[(13., 13.), (17., 13.), (17., 17.), (13., 17.), (13., 13.)],
            0.2,
        );
        let f = fill_net(&b, &[over_board(&b, "GND", 0)], 0, &Options::default());
        // inside the ring there is fill around the pad: an island with its pad
        assert_eq!(f.pins[1].reach, Reach::Island(0));
        assert_eq!(f.islands[0].pins, vec![1]);
        // a ring at the clearance from the pad leaves no fill at all
        let mut b = board(&[("GND", 5., 5., 0), ("GND", 15., 15., 0), ("S", 14.2, 14.2, 0)]);
        b.obstacles[2].shape = Shape::Circle {
            c: pt(14.2, 14.2),
            r: 0.05,
        };
        track(
            &mut b,
            "S",
            0,
            &[(14.2, 14.2), (15.8, 14.2), (15.8, 15.8), (14.2, 15.8), (14.2, 14.2)],
            0.2,
        );
        let f = fill_net(&b, &[solid(over_board(&b, "GND", 0))], 0, &Options::default());
        assert_eq!(f.pins[1].reach, Reach::Walled);
        let f = fill_net(&b, &[over_board(&b, "GND", 0)], 0, &Options::default());
        assert_eq!(f.pins[1].reach, Reach::NoSpoke);
        assert_eq!(f.pins[0].reach, Reach::Fill);
    }

    #[test]
    fn a_thermal_relief_joins_the_pad_through_its_spokes() {
        let b = board(&[("GND", 5., 5., 0), ("GND", 15., 15., 0)]);
        let f = fill_net(&b, &[over_board(&b, "GND", 0)], 0, &Options::default());
        assert_eq!(f.unreached().count(), 0);
        let fill = f.fill[0].as_ref().unwrap();
        // the gap is open on the diagonal, the spoke crosses it on the axis
        assert!(!fill[f.raster.cell(pt(5.65, 5.65)).unwrap()]);
        assert!(fill[f.raster.cell(pt(5.65, 5.0)).unwrap()]);
        // a pad that says "no connection" stays apart
        let mut o = Options::default();
        o.pad_connect.insert(1, Connect::None);
        let f = fill_net(&b, &[over_board(&b, "GND", 0)], 0, &o);
        assert_eq!(f.pins[1].reach, Reach::Walled);
    }

    #[test]
    fn an_island_narrower_than_a_via_gets_none() {
        // a channel of fill 0.4 mm wide between two long tracks holds the second pad
        let mut b = board(&[
            ("GND", 5., 3., 0),
            ("GND", 10., 10., 0),
            ("S", 2., 9.0, 0),
            ("T", 2., 11.0, 0),
            ("GND", 15., 3., 0),
        ]);
        b.obstacles[1].shape = Shape::Polygon(roundrect_polygon(pt(10., 10.), 0.4, 0.4, 0.0, 0.0, 0.005));
        track(&mut b, "S", 0, &[(0.2, 9.5), (19.8, 9.5)], 0.2);
        track(&mut b, "T", 0, &[(0.2, 10.5), (19.8, 10.5)], 0.2);
        let areas = [solid(over_board(&b, "GND", 0)), solid(over_board(&b, "GND", 1))];
        let f = fill_net(&b, &areas, 0, &Options::default());
        assert_eq!(f.pins[1].reach, Reach::Island(0));
        match f.islands[0].no_via {
            Some(NoVia::TooNarrow { widest, need }) => assert!(widest < 0.45 && need == 0.6, "{widest}"),
            ref x => panic!("{x:?}"),
        }
    }

    #[test]
    fn a_pad_that_holds_a_via_gets_a_site_in_it() {
        // a 3 mm pad in a ring of another net that leaves no room for fill around it; the area
        // is the pour on the other layer, which a via joins to the two pads on the left
        let mut b = board(&[
            ("GND", 3., 4., 0),
            ("GND", 3., 16., 0),
            ("GND", 12., 10., 0),
            ("S", 9.9, 7.9, 0),
            ("GND", 16., 3., 0),
        ]);
        b.obstacles[2].shape = Shape::Polygon(roundrect_polygon(pt(12., 10.), 3.0, 3.0, 0.0, 0.0, 0.005));
        b.obstacles[3].shape = Shape::Circle {
            c: pt(9.9, 7.9),
            r: 0.1,
        };
        track(
            &mut b,
            "S",
            0,
            &[(9.9, 7.9), (14.1, 7.9), (14.1, 12.1), (9.9, 12.1), (9.9, 7.9)],
            0.2,
        );
        // the small pad next to it holds no via
        b.obstacles[4].shape = Shape::Polygon(roundrect_polygon(pt(16., 3.), 0.4, 0.4, 0.0, 0.0, 0.005));
        via(&mut b, "GND", 3.0, 6.0);
        let areas = [over_board(&b, "GND", 0), over_board(&b, "GND", 1)];
        let f = fill_net(&b, &areas, 0, &Options::default());
        assert_eq!(f.pins[2].reach, Reach::NoSpoke);
        let site = f.pins[2].pad_site.as_ref().expect("a site in the pad");
        // in the middle of the pad, from its layer to the other, no second via needed
        assert!(site.at.dist(pt(12., 10.)) < 0.1, "{site:?}");
        assert_eq!((site.from, site.to, site.anchor), (0, 1, None));
        // pads that are reached, or too small for a via, have none
        assert!(f.pins[0].pad_site.is_none());
        assert_eq!((f.pins[3].pin, f.pins[3].reach), (4, Reach::Fill));
        // with the via set the pad hangs on the area
        via(&mut b, "GND", site.at.x, site.at.y);
        let f = fill_net(&b, &areas, 0, &Options::default());
        assert_eq!(f.pins[2].reach, Reach::Copper);
        assert_eq!(f.unreached().count(), 0);
    }

    #[test]
    fn the_distance_to_a_via_site_is_not_cut_short_by_the_edge_of_the_board() {
        // the area lies at the right on layer 1, the pad in the corner at the left on layer 0:
        // the corner is 3 mm from the board edge and more than 10 mm from the area
        let b = board(&[("GND", 3., 3., 0), ("GND", 17., 10., 1), ("GND", 17., 14., 1)]);
        let right = Polygon::new(vec![pt(14., 0.), pt(20., 0.), pt(20., 20.), pt(14., 20.)]);
        let f = fill_net(&b, &[solid(Area::pour(0, 1, vec![right]))], 0, &Options::default());
        assert_eq!(f.pins[0].reach, Reach::OtherLayer);
        assert!(
            f.pins[0].site_dist > 10.5 && f.pins[0].site_dist < 12.0,
            "{}",
            f.pins[0].site_dist
        );
    }

    #[test]
    fn a_pin_on_another_layer_is_named_with_its_distance_to_a_via_site() {
        // a plane on layer 1, the pads on layer 0
        let b = board(&[("GND", 5., 5., 0), ("S", 6.2, 5., 0)]);
        let f = fill_net(&b, &[solid(over_board(&b, "GND", 1))], 0, &Options::default());
        assert_eq!(f.pins[0].reach, Reach::OtherLayer);
        // the nearest site is just off the pad, away from the other net's pad
        assert!(
            f.pins[0].site_dist > 0.4 && f.pins[0].site_dist < 0.8,
            "{}",
            f.pins[0].site_dist
        );
        // with a via in reach of the pad the pin hangs on the plane
        let mut b = b;
        via(&mut b, "GND", 4.4, 5.0);
        let f = fill_net(&b, &[solid(over_board(&b, "GND", 1))], 0, &Options::default());
        assert_eq!(f.pins[0].reach, Reach::Copper);
    }

    #[test]
    fn an_area_of_another_net_with_a_higher_priority_cuts_the_fill() {
        let b = board(&[("GND", 5., 5., 0), ("V", 15., 15., 0)]);
        let mut v = solid(Area::pour(
            1,
            0,
            vec![Polygon::new(vec![
                pt(10., 10.),
                pt(20., 10.),
                pt(20., 20.),
                pt(10., 20.),
            ])],
        ));
        v.priority = 1;
        let areas = [solid(over_board(&b, "GND", 0)), v];
        let f = fill_net(&b, &areas, 0, &Options::default());
        let fill = f.fill[0].as_ref().unwrap();
        assert!(fill[f.raster.cell(pt(5., 15.)).unwrap()]);
        assert!(!fill[f.raster.cell(pt(15., 15.)).unwrap()]);
        // it keeps its clearance from the other outline
        assert!(!fill[f.raster.cell(pt(9.9, 15.)).unwrap()]);
        assert!(fill[f.raster.cell(pt(9.7, 15.)).unwrap()]);
        // the other net fills its own outline
        let f = fill_net(&b, &areas, 1, &Options::default());
        assert!(f.fill[0].as_ref().unwrap()[f.raster.cell(pt(15., 15.)).unwrap()]);
        assert_eq!(f.pins[0].reach, Reach::Fill);
    }
}
