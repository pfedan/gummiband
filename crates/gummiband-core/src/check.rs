//! Clearance check of routed geometry against the board and itself.

use rayon::prelude::*;

use crate::board::{Board, NetId, ObstacleKind};
use crate::geom::{Point, Rect, Shape, point_seg_dist, seg_seg_dist};
use crate::index::GridIndex;
use crate::realize::Geometry;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Violation {
    pub what: String,
    /// Nets involved (the routed item first).
    pub nets: (NetId, Option<NetId>),
    pub at: crate::geom::Point,
    pub actual: f64,
    pub required: f64,
}

impl Violation {
    /// A way for the current narrower than the minimum width (see [`current_ways`]), not a
    /// clearance: `actual` and `required` are widths.
    pub fn is_current(&self) -> bool {
        self.what.starts_with("current ")
    }

    /// A width and not a clearance: a way for the current, or a piece below the minimum width
    /// of its class out of the courtyards that allow it ([`min_widths`]).
    pub fn is_width(&self) -> bool {
        self.is_current() || self.what.starts_with("width ")
    }
}

/// One place with a problem. The check reports every short piece of track that comes too close,
/// so a single curve around a pad can make twenty violations out of one problem.
#[derive(Debug, Clone)]
pub struct Spot {
    /// Where it is worst.
    pub at: crate::geom::Point,
    pub nets: (NetId, Option<NetId>),
    /// How much clearance is missing there.
    pub missing: f64,
    pub short: bool,
}

/// The violations of one pair of nets within 2 mm count as one spot.
pub fn spots(violations: &[Violation]) -> Vec<Spot> {
    let mut out: Vec<Spot> = Vec::new();
    for v in violations {
        let key = (
            v.nets.0.min(v.nets.1.unwrap_or(u32::MAX)),
            v.nets.0.max(v.nets.1.unwrap_or(u32::MAX)),
        );
        let missing = v.required - v.actual;
        let same = |s: &&mut Spot| {
            let k = s.nets.1.unwrap_or(u32::MAX);
            (s.nets.0.min(k), s.nets.0.max(k)) == key && s.at.dist(v.at) < 2.0
        };
        match out.iter_mut().find(|s| same(s)) {
            Some(s) => {
                if missing > s.missing {
                    s.missing = missing;
                    s.at = v.at;
                }
                s.short |= v.actual <= 0.0;
            }
            None => out.push(Spot {
                at: v.at,
                nets: v.nets,
                missing,
                short: v.actual <= 0.0,
            }),
        }
    }
    out
}

enum Piece {
    Seg {
        a: crate::geom::Point,
        b: crate::geom::Point,
        r: f64,
        layer: u8,
        net: NetId,
    },
    Via {
        c: crate::geom::Point,
        r: f64,
        /// Drill diameter (0 for a branch point without hole).
        drill: f64,
        net: NetId,
        /// A pair of vias at c - half and c + half (zero: one via at c); r and drill are those
        /// of each.
        half: Point,
    },
}

/// The distance between the centres (lines) of two vias or via pairs.
fn vias_dist(c1: Point, h1: Point, c2: Point, h2: Point) -> f64 {
    match (h1 == Point::default(), h2 == Point::default()) {
        (true, true) => c1.dist(c2),
        (false, true) => via_dist(c2, c1, h1),
        (true, false) => via_dist(c1, c2, h2),
        (false, false) => seg_seg_dist(c1 - h1, c1 + h1, c2 - h2, c2 + h2),
    }
}

/// The distance of point p to the centre (line) of a via or via pair.
fn via_dist(p: Point, c: Point, half: Point) -> f64 {
    if half == Point::default() {
        p.dist(c)
    } else {
        point_seg_dist(p, c - half, c + half)
    }
}

impl Piece {
    fn bbox(&self) -> Rect {
        let mut r = Rect::empty();
        match *self {
            Piece::Seg { a, b, r: rr, .. } => {
                r.add(a);
                r.add(b);
                r.grow(rr)
            }
            Piece::Via { c, r: rr, half, .. } => {
                r.add(c - half);
                r.add(c + half);
                r.grow(rr)
            }
        }
    }
}

/// The copper of a geometry and the obstacles of its board, indexed for the clearance test.
pub struct Copper<'a> {
    board: &'a Board,
    pieces: Vec<Piece>,
    idx: GridIndex,
    oidx: GridIndex,
    /// The edges of the board outline, and an index of them: a panel has hundreds.
    esegs: Vec<(Point, Point)>,
    eidx: GridIndex,
}

impl<'a> Copper<'a> {
    pub fn new(board: &'a Board, geo: &Geometry) -> Self {
        let mut pieces = Vec::new();
        for t in &geo.tracks {
            for w in t.pts.windows(2) {
                pieces.push(Piece::Seg {
                    a: w[0],
                    b: w[1],
                    r: t.piece_width(w[0], w[1]) / 2.0,
                    layer: t.layer,
                    net: t.net,
                });
            }
        }
        // (a branch point of soft tracks has no copper of its own)
        for v in geo.vias.iter().filter(|v| v.drill > 0.0) {
            let h = if v.is_pair() { v.half.len() } else { 0.0 };
            pieces.push(Piece::Via {
                c: v.at,
                r: v.diameter / 2.0 - h,
                drill: v.drill - 2.0 * h,
                net: v.net,
                half: v.half,
            });
        }
        let mut idx = GridIndex::new(board.bbox().grow(5.0), 1.0);
        for (i, p) in pieces.iter().enumerate() {
            idx.insert(i as u32, &p.bbox());
        }
        let mut oidx = GridIndex::new(board.bbox().grow(5.0), 1.0);
        for (i, o) in board.obstacles.iter().enumerate() {
            oidx.insert(i as u32, &o.shape.bbox());
        }
        let esegs: Vec<(Point, Point)> = board.outline.iter().flat_map(|o| o.edges()).collect();
        let mut eidx = GridIndex::new(board.bbox().grow(5.0), 1.0);
        for (i, &(a, b)) in esegs.iter().enumerate() {
            let mut r = Rect::empty();
            r.add(a);
            r.add(b);
            eidx.insert(i as u32, &r);
        }
        Copper {
            board,
            pieces,
            idx,
            oidx,
            esegs,
            eidx,
        }
    }

    /// Do these tracks (from, to, width, layer) and vias (centre, diameter) of `net` keep the
    /// clearances of [`check`] to the board and to the copper?
    pub fn clear(&self, net: NetId, segs: &[(Point, Point, f64, u8)], vias: &[(Point, f64)]) -> bool {
        let segs = segs.iter().map(|&(a, b, w, layer)| Piece::Seg {
            a,
            b,
            r: w / 2.0,
            layer,
            net,
        });
        let drill = self.board.class_of(net).via_drill;
        let vias = vias.iter().map(|&(c, d)| Piece::Via {
            c,
            r: d / 2.0,
            drill,
            net,
            half: Point::default(),
        });
        segs.chain(vias).all(|p| self.violations(&p, None).is_empty())
    }

    /// The violations of piece p; with `after` only those against the pieces after that one,
    /// so that every pair counts once.
    fn violations(&self, p: &Piece, after: Option<usize>) -> Vec<Violation> {
        let board = self.board;
        let routing = board.routing_layers();
        let clr = |n: NetId| board.class_of(n).clearance.max(board.rules.min_clearance);
        let mut out = Vec::new();
        let (net, layer_has): (NetId, Box<dyn Fn(u8) -> bool>) = match *p {
            Piece::Seg { net, layer, .. } => (net, Box::new(move |l| l == layer)),
            Piece::Via { net, .. } => (net, Box::new(|_| true)),
        };
        let shape = match *p {
            Piece::Seg { a, b, r, .. } => Shape::Capsule { a, b, r },
            Piece::Via { c, r, half, .. } if half != Point::default() => Shape::Capsule {
                a: c - half,
                b: c + half,
                r,
            },
            Piece::Via { c, r, .. } => Shape::Circle { c, r },
        };
        let at = match *p {
            Piece::Seg { a, b, .. } => a.lerp(b, 0.5),
            Piece::Via { c, .. } => c,
        };
        let bb = p.bbox().grow(1.0);
        // board edge
        let line = match *p {
            Piece::Seg { a, b, .. } => Some((a, b)),
            Piece::Via { c, half, .. } if half != Point::default() => Some((c - half, c + half)),
            Piece::Via { .. } => None,
        };
        let r = match *p {
            Piece::Seg { r, .. } | Piece::Via { r, .. } => r,
        };
        // only edges within reach of the piece can be too near; the nearest of them is the
        // nearest of all whenever it is too near
        let mut reach = Rect::empty();
        match line {
            Some((a, b)) => {
                reach.add(a);
                reach.add(b);
            }
            None => reach.add(at),
        }
        let reach = reach.grow(board.rules.edge_clearance + r + 1e-3);
        let edge = self
            .eidx
            .query(&reach)
            .into_iter()
            .map(|k| {
                let (c, d) = self.esegs[k as usize];
                match line {
                    Some((a, b)) => seg_seg_dist(a, b, c, d),
                    None => point_seg_dist(at, c, d),
                }
            })
            .fold(f64::INFINITY, f64::min)
            - r;
        if edge < board.rules.edge_clearance - 1e-4 {
            out.push(Violation {
                what: "edge".into(),
                nets: (net, None),
                at,
                actual: edge,
                required: board.rules.edge_clearance,
            });
        }
        let h2h = board.rules.hole_to_hole;
        for j in self.oidx.query(&bb) {
            let o = &board.obstacles[j as usize];
            // hole to hole holds for every net, the own one too (KiCad's hole_to_hole)
            if let (Piece::Via { c, drill, half, .. }, Some(h)) = (p, o.hole)
                && *drill > 0.0
            {
                let oc = o.shape.bbox().center();
                let d = via_dist(oc, *c, *half) - h - drill / 2.0;
                // the same via, kept and in the geometry as well
                let itself = o.net == Some(net) && c.dist(oc) < 1e-3;
                if !itself && d < h2h - 1e-4 {
                    out.push(Violation {
                        what: format!(
                            "hole_to_hole {}~{}",
                            board.nets[net as usize].name,
                            o.net.map_or("-".to_string(), |n| board.nets[n as usize].name.clone())
                        ),
                        nets: (net, o.net),
                        at: *c,
                        actual: d,
                        required: h2h,
                    });
                }
            }
            if o.net == Some(net) && o.kind != ObstacleKind::Hole {
                continue;
            }
            // a via goes through all layers, also the planes
            let via = matches!(p, Piece::Via { .. });
            if !(o.layers.iter().any(|l| routing.has(l) && layer_has(l))
                || (via && (o.hole.is_some() || matches!(o.kind, ObstacleKind::Keepout { vias: true, .. }))))
            {
                continue;
            }
            let need = match o.kind {
                ObstacleKind::Keepout { tracks, vias, .. } => {
                    let applies = match p {
                        Piece::Seg { .. } => tracks,
                        Piece::Via { .. } => vias,
                    };
                    if !applies {
                        continue;
                    }
                    crate::board::KEEPOUT_GAP
                }
                ObstacleKind::Hole => board.hole_gap(net),
                _ => o.clearance.max(clr(net)).max(board.item_gap(net, o.net)),
            };
            let d = o.shape.dist(&shape);
            if d < need - 1e-4 {
                out.push(Violation {
                    what: format!(
                        "{} {}~{}",
                        match o.kind {
                            // clear of the copper rule: it is the pad's mask opening that asks for more
                            ObstacleKind::Pad if d >= board.clearance(Some(net), o.net) - 1e-4 => "mask",
                            ObstacleKind::Pad => "pad",
                            ObstacleKind::Track => "locked-track",
                            ObstacleKind::Via => "locked-via",
                            ObstacleKind::Keepout { mask: true, .. } => "mask",
                            ObstacleKind::Keepout { .. } => "keepout",
                            ObstacleKind::Zone => "zone",
                            ObstacleKind::Pour => "pour",
                            ObstacleKind::Graphic => "graphic",
                            ObstacleKind::Hole => "hole",
                        },
                        board.nets[net as usize].name,
                        o.net.map_or("-".to_string(), |n| board.nets[n as usize].name.clone())
                    ),
                    nets: (net, o.net),
                    at,
                    actual: d,
                    required: need,
                });
            }
        }
        for j in self.idx.query(&bb) {
            if after.is_some_and(|i| j as usize <= i) {
                continue;
            }
            let q = &self.pieces[j as usize];
            if let (
                Piece::Via {
                    c,
                    drill,
                    net: n1,
                    half: h1,
                    ..
                },
                Piece::Via {
                    c: c2,
                    drill: d2,
                    net: n2,
                    half: h2,
                    ..
                },
            ) = (p, q)
                && *drill > 0.0
                && *d2 > 0.0
                && c.dist(*c2) > 1e-3
            {
                let d = vias_dist(*c, *h1, *c2, *h2) - drill / 2.0 - d2 / 2.0;
                if d < h2h - 1e-4 {
                    out.push(Violation {
                        what: format!(
                            "hole_to_hole {}~{}",
                            board.nets[*n1 as usize].name, board.nets[*n2 as usize].name
                        ),
                        nets: (*n1, Some(*n2)),
                        at: *c,
                        actual: d,
                        required: h2h,
                    });
                }
            }
            let (qnet, d, both_layers) = match (p, q) {
                (
                    Piece::Seg { a, b, r, layer, .. },
                    Piece::Seg {
                        a: c,
                        b: d,
                        r: r2,
                        layer: l2,
                        net: n2,
                    },
                ) => (*n2, seg_seg_dist(*a, *b, *c, *d) - r - r2, layer == l2),
                (
                    Piece::Seg { a, b, r, .. },
                    Piece::Via {
                        c,
                        r: r2,
                        net: n2,
                        half,
                        ..
                    },
                )
                | (
                    Piece::Via {
                        c,
                        r: r2,
                        net: n2,
                        half,
                        ..
                    },
                    Piece::Seg { a, b, r, .. },
                ) => {
                    let n = if matches!(p, Piece::Via { .. }) {
                        match q {
                            Piece::Seg { net, .. } => *net,
                            _ => *n2,
                        }
                    } else {
                        *n2
                    };
                    let d = if *half == Point::default() {
                        point_seg_dist(*c, *a, *b)
                    } else {
                        seg_seg_dist(*a, *b, *c - *half, *c + *half)
                    };
                    (n, d - r - r2, true)
                }
                (
                    Piece::Via { c, r, half: h1, .. },
                    Piece::Via {
                        c: c2,
                        r: r2,
                        net: n2,
                        half: h2,
                        ..
                    },
                ) => (*n2, vias_dist(*c, *h1, *c2, *h2) - r - r2, true),
            };
            if qnet == net || !both_layers {
                continue;
            }
            let need = if matches!((p, q), (Piece::Seg { .. }, Piece::Seg { .. })) {
                board.track_clearance(net, qnet)
            } else {
                clr(net).max(clr(qnet)).max(board.item_gap(net, Some(qnet)))
            };
            if d < need - 1e-4 {
                out.push(Violation {
                    what: format!(
                        "{} {}~{}",
                        if matches!(p, Piece::Via { .. }) || matches!(q, Piece::Via { .. }) {
                            "via"
                        } else {
                            "track"
                        },
                        board.nets[net as usize].name,
                        board.nets[qnet as usize].name
                    ),
                    nets: (net, Some(qnet)),
                    at,
                    actual: d,
                    required: need,
                });
            }
        }
        out
    }
}

pub fn check(board: &Board, geo: &Geometry) -> Vec<Violation> {
    let copper = Copper::new(board, geo);
    let mut out: Vec<Violation> = copper
        .pieces
        .par_iter()
        .enumerate()
        .flat_map_iter(|(i, p)| copper.violations(p, Some(i)))
        .collect();
    // a custom rule may lower the clearance in a courtyard (fine pitch power ICs)
    if !board.courtyard_clearance.is_empty() {
        out.retain_mut(|v| match board.courtyard_clearance_at(v.at, v.required) {
            Some(min) if v.actual >= min - 1e-6 => false,
            Some(min) => {
                v.required = v.required.min(min);
                true
            }
            None => true,
        });
    }
    out.extend(min_widths(board, geo));
    out.extend(current_ways(board, geo));
    out
}

/// A piece of track narrower than the minimum width of its class that does not touch the
/// courtyard of a part the rule exempts, with its own width: KiCad's rule
/// (`!A.intersectsCourtyard(..)`) sees it the same way, with the shape of the track.
fn min_widths(board: &Board, geo: &Geometry) -> Vec<Violation> {
    let mut out = Vec::new();
    // the board minimum holds everywhere, in exempt courtyards too (a taper or a neck below it
    // is what KiCad reports as track_width or connection_width)
    let floor = board.rules.width_floor();
    for t in &geo.tracks {
        for w in t.pts.windows(2) {
            let width = t.piece_width(w[0], w[1]);
            if floor > 0.0 && width < floor - 1e-6 && w[0].dist(w[1]) > 1e-6 {
                out.push(Violation {
                    what: format!("width {} below the board minimum", board.nets[t.net as usize].name),
                    nets: (t.net, None),
                    at: w[0].lerp(w[1], 0.5),
                    actual: width,
                    required: floor,
                });
            }
        }
    }
    for t in &geo.tracks {
        let cls = board.class_of(t.net);
        let Some(min) = cls.min_width else {
            continue;
        };
        let exempt: Vec<&Rect> = cls.neck_refs.iter().filter_map(|r| board.courtyards.get(r)).collect();
        for w in t.pts.windows(2) {
            let width = t.piece_width(w[0], w[1]);
            if width < min - 1e-9 && !exempt.iter().any(|r| r.dist_seg(w[0], w[1]) <= width / 2.0) {
                out.push(Violation {
                    what: format!("width {}", board.nets[t.net as usize].name),
                    nets: (t.net, None),
                    at: w[0].lerp(w[1], 0.5),
                    actual: width,
                    required: min,
                });
            }
        }
    }
    out
}

/// Under a minimum width rule the current between two pins that can take the width must not pass
/// a neck: on the widest way through the copper of the net between any two of them, nothing may
/// carry less than the minimum. A fine pitch pin of an exempt part and the branch to it are free
/// to be narrow; a way through such a pin counts with the width of its pad. Copper counts with
/// what it carries on its layer, a layer change with what its vias carry together
/// ([`Board::current_width`], [`Board::via_current_width`]). Zones and locked copper of the net
/// count as well; a net with a plane is left out, and so are the nets of a differential pair
/// ([`Board::current_min`]).
fn current_ways(board: &Board, geo: &Geometry) -> Vec<Violation> {
    let mut out = Vec::new();
    for (n, net) in board.nets.iter().enumerate() {
        let n = n as NetId;
        let Some(min) = board.current_min(n) else {
            continue;
        };
        if board.planes.iter().any(|p| p.net == n) {
            continue;
        }
        let wide: Vec<usize> = (0..net.pins.len())
            .filter(|&i| !board.narrow_pin(net.pins[i]))
            .collect();
        if wide.len() < 2 {
            continue;
        }
        let (width, at) = widest_ways(board, geo, n, wide[0]);
        let worst = wide[1..]
            .iter()
            .filter(|&&i| width[i] > f64::NEG_INFINITY)
            .min_by(|&&i, &&j| width[i].total_cmp(&width[j]));
        if let Some(&i) = worst
            && width[i] < min - 1e-9
        {
            out.push(Violation {
                what: format!("current {}", net.name),
                nets: (n, None),
                at: at[i],
                actual: width[i],
                required: min,
            });
        }
    }
    out
}

/// Through the copper of net n from its pin `from` (an index into the pins of the net): per pin
/// of the net the width of the widest way there (minus infinity where there is none) and where
/// that way is narrowest. The way runs through the tracks, vias, zones and locked copper of the
/// net; through a narrow pin (`Board::narrow_pin`) it is as wide as there is room at the pin.
/// Widths are what the copper carries, as a width on the outer copper: a piece on a thin or
/// inner layer counts for less, and a layer change counts with the barrels of all vias that
/// join the same copper on both sides.
pub fn widest_ways(board: &Board, geo: &Geometry, n: NetId, from: usize) -> (Vec<f64>, Vec<Point>) {
    use crate::board::LayerSet;
    enum Item<'a> {
        Pin(usize),
        End(Point, u8),
        /// The pad of a via on one layer, or a point on one layer where soft tracks branch.
        Pad(Point, f64, u8),
        /// The barrels of the vias that join the same copper.
        Joint(Point),
        Region(&'a Shape, LayerSet),
    }
    let pins = &board.nets[n as usize].pins;
    let nl = board.layers.len() as u8;
    let mut items: Vec<Item> = pins.iter().map(|&p| Item::Pin(p)).collect();
    // (node, node, width, where it is narrowest)
    let mut edges: Vec<(usize, usize, f64, Point)> = Vec::new();
    let near = 1e-3;
    let tracks: Vec<&crate::realize::Track> = geo.tracks.iter().filter(|t| t.net == n && t.pts.len() > 1).collect();
    // copper can meet a track between its corners: a via over it, or the end of another track
    // on it (point, how near, layer; none: all)
    let mut touching: Vec<(Point, f64, Option<u8>)> = Vec::new();
    for v in geo.vias.iter().filter(|v| v.net == n) {
        touching.push((v.at, v.diameter / 2.0, v.layer.filter(|_| v.drill <= 0.0)));
    }
    for t in &tracks {
        touching.push((t.pts[0], near, Some(t.layer)));
        touching.push((t.pts[t.pts.len() - 1], near, Some(t.layer)));
    }
    for t in &tracks {
        // a node at every corner and where copper meets a piece: (piece, distance from its
        // start, point)
        let mut stops: Vec<(usize, f64, Point)> = (0..t.pts.len()).map(|i| (i, 0.0, t.pts[i])).collect();
        for &(q, reach, layer) in &touching {
            if layer.is_some_and(|l| l != t.layer) {
                continue;
            }
            for (i, w) in t.pts.windows(2).enumerate() {
                let (a, b) = (w[0], w[1]);
                let len = a.dist(b);
                if len < 1e-9 {
                    continue;
                }
                let along = ((q - a).dot(b - a) / len).clamp(0.0, len);
                let p = a + (b - a) * (along / len);
                if p.dist(q) <= reach + near && along > near && along < len - near {
                    stops.push((i, along, p));
                }
            }
        }
        stops.sort_by(|x, y| x.0.cmp(&y.0).then(x.1.total_cmp(&y.1)));
        stops.dedup_by(|x, y| x.2.dist(y.2) < 1e-9);
        let first = items.len();
        items.extend(stops.iter().map(|s| Item::End(s.2, t.layer)));
        for k in 1..stops.len() {
            // a stretch between two stops lies on one piece: the one its middle is on
            let (a, b) = (stops[k - 1].2, stops[k].2);
            let i = if stops[k].1 > 0.0 { stops[k].0 } else { stops[k].0 - 1 };
            let w = t.piece_width(t.pts[i], t.pts[i + 1]);
            edges.push((
                first + k - 1,
                first + k,
                board.current_width(t.layer, w),
                a.lerp(b, 0.5),
            ));
        }
    }
    // a via is a pad on every layer it spans and a barrel (drill; 0: no limit) between them
    let mut barrels: Vec<(Point, f64, Vec<usize>)> = Vec::new();
    for v in geo.vias.iter().filter(|v| v.net == n) {
        match v.layer.filter(|_| v.drill <= 0.0) {
            Some(l) => items.push(Item::Pad(v.at, v.diameter / 2.0, l)),
            None => {
                let first = items.len();
                items.extend((0..nl).map(|l| Item::Pad(v.at, v.diameter / 2.0, l)));
                barrels.push((v.at, v.drill, (first..items.len()).collect()));
            }
        }
    }
    let mut piece = |items: &mut Vec<Item>, a: Point, b: Point, layer: u8, w: f64, at: Point| {
        items.push(Item::End(a, layer));
        items.push(Item::End(b, layer));
        edges.push((items.len() - 2, items.len() - 1, board.current_width(layer, w), at));
    };
    for o in board.obstacles.iter().filter(|o| o.net == Some(n) && o.pin.is_none()) {
        match (&o.kind, &o.shape) {
            (ObstacleKind::Track, Shape::Capsule { a, b, r }) => {
                for l in o.layers.iter() {
                    piece(&mut items, *a, *b, l, 2.0 * r, a.lerp(*b, 0.5));
                }
            }
            (ObstacleKind::Via, Shape::Circle { c, r }) => {
                let first = items.len();
                items.extend(o.layers.iter().map(|l| Item::Pad(*c, *r, l)));
                barrels.push((*c, 2.0 * o.hole.unwrap_or(0.0), (first..items.len()).collect()));
            }
            (ObstacleKind::Track | ObstacleKind::Zone, shape) => items.push(Item::Region(shape, o.layers)),
            _ => {}
        }
    }
    // copper that touches on a layer is one: a way through a pin has the width of its pad
    let pad = |p: usize, f: &dyn Fn(&Shape, LayerSet) -> bool| {
        board.pins[p]
            .obstacles
            .iter()
            .any(|&o| f(&board.obstacles[o].shape, board.obstacles[o].layers))
    };
    let touch = |x: &Item, y: &Item| -> bool {
        use Item::*;
        match (x, y) {
            (Pin(_), Pin(_)) | (Joint(_), _) | (_, Joint(_)) => false,
            (End(a, la), End(b, lb)) => la == lb && a.dist(*b) < near,
            (End(e, l), Pad(c, r, vl)) | (Pad(c, r, vl), End(e, l)) => vl == l && e.dist(*c) <= r + near,
            (End(e, l), Pin(p)) | (Pin(p), End(e, l)) => pad(*p, &|s, ls| ls.has(*l) && s.dist_to_point(*e) < near),
            (End(e, l), Region(s, ls)) | (Region(s, ls), End(e, l)) => ls.has(*l) && s.dist_to_point(*e) < near,
            (Pad(c, _, l), Pin(p)) | (Pin(p), Pad(c, _, l)) => {
                pad(*p, &|s, ls| ls.has(*l) && s.dist_to_point(*c) < near)
            }
            (Pad(c, _, l), Region(s, ls)) | (Region(s, ls), Pad(c, _, l)) => ls.has(*l) && s.dist_to_point(*c) < near,
            (Pad(a, ra, la), Pad(b, rb, lb)) => la == lb && a.dist(*b) < ra + rb,
            (Pin(p), Region(s, ls)) | (Region(s, ls), Pin(p)) => {
                pad(*p, &|ps, pls| pls.intersects(*ls) && ps.dist(s) < near)
            }
            (Region(s, ls), Region(t, lt)) => ls.intersects(*lt) && s.dist(t) < near,
        }
    };
    let mut adj: Vec<Vec<(usize, f64, Point)>> = vec![Vec::new(); items.len()];
    for &(a, b, w, at) in &edges {
        adj[a].push((b, w, at));
        adj[b].push((a, w, at));
    }
    // what touches, and which copper is one without the barrels
    let mut root: Vec<usize> = (0..items.len()).collect();
    fn find(root: &mut [usize], mut i: usize) -> usize {
        while root[i] != i {
            root[i] = root[root[i]];
            i = root[i];
        }
        i
    }
    for i in 0..items.len() {
        for j in i + 1..items.len() {
            if touch(&items[i], &items[j]) {
                adj[i].push((j, f64::INFINITY, Point::default()));
                adj[j].push((i, f64::INFINITY, Point::default()));
                let (a, b) = (find(&mut root, i), find(&mut root, j));
                root[a] = b;
            }
        }
    }
    // vias whose pads lie in the same copper on every layer where they touch any are in
    // parallel: one joint that carries what all their barrels carry
    let mut joints: Vec<Joint> = Vec::new();
    for (at, drill, pads) in &barrels {
        let key: Vec<Option<usize>> = pads
            .iter()
            .map(|&p| (!adj[p].is_empty()).then(|| find(&mut root, p)))
            .collect();
        let j = match joints.iter().position(|jt| jt.key == key) {
            Some(j) => j,
            None => {
                items.push(Item::Joint(*at));
                adj.push(Vec::new());
                joints.push(Joint {
                    key,
                    node: items.len() - 1,
                    unlimited: false,
                    drills: Vec::new(),
                });
                joints.len() - 1
            }
        };
        let node = joints[j].node;
        if *drill <= 0.0 {
            joints[j].unlimited = true;
        } else {
            joints[j].drills.push(*drill);
        }
        for &p in pads {
            adj[p].push((node, f64::INFINITY, Point::default()));
            adj[node].push((p, f64::INFINITY, Point::default()));
        }
    }
    let mut cap: Vec<f64> = items
        .iter()
        .map(|it| match it {
            Item::Pin(p) if board.narrow_pin(*p) => {
                let cls = board.class_of(n);
                board.pin_room(*p, cls.clearance.max(board.rules.min_clearance))
            }
            _ => f64::INFINITY,
        })
        .collect();
    for jt in joints.iter().filter(|jt| !jt.unlimited) {
        cap[jt.node] = board.via_current_width(jt.drills.iter().copied());
    }
    let pos = |i: usize| match items[i] {
        Item::Pin(p) => board.pins[p].pos,
        Item::End(a, _) | Item::Pad(a, _, _) | Item::Joint(a) => a,
        Item::Region(..) => Point::default(),
    };
    let mut width = vec![f64::NEG_INFINITY; items.len()];
    let mut at = vec![Point::default(); items.len()];
    width[from] = f64::INFINITY;
    at[from] = pos(from);
    let mut heap = std::collections::BinaryHeap::new();
    heap.push(Widest(f64::INFINITY, from));
    while let Some(Widest(w, u)) = heap.pop() {
        if w < width[u] {
            continue;
        }
        for &(v, ew, eat) in &adj[u] {
            let c = w.min(ew).min(cap[v]);
            if c > width[v] {
                width[v] = c;
                at[v] = if c == w {
                    at[u]
                } else if c == ew {
                    eat
                } else {
                    pos(v)
                };
                heap.push(Widest(c, v));
            }
        }
    }
    width.truncate(pins.len());
    at.truncate(pins.len());
    (width, at)
}

/// A node of the widest way search, widest first.
struct Widest(f64, usize);

/// Vias in parallel in the widest way search: the copper they join on each layer (its root, or
/// none where the pad touches nothing), their node, whether one of them is a branch point that
/// carries without limit, and the drills of the others.
struct Joint {
    key: Vec<Option<usize>>,
    node: usize,
    unlimited: bool,
    drills: Vec<f64>,
}

impl PartialEq for Widest {
    fn eq(&self, o: &Self) -> bool {
        self.cmp(o) == std::cmp::Ordering::Equal
    }
}
impl Eq for Widest {}
impl PartialOrd for Widest {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Widest {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        self.0.total_cmp(&o.0).then(o.1.cmp(&self.1))
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::geom::{Polygon, pt};
    use crate::realize::{Elem, Track};

    /// A board with one ruled net from a top pad at (2, 5) to a bottom pad at (8, 5), 2 oz copper
    /// on both sides.
    pub(crate) fn current_board() -> Board {
        let mut b = crate::topo::tests::two_pad_board();
        b.classes[0].track_width = 0.75;
        b.classes[0].min_width = Some(0.69);
        b.classes[0].via_diameter = 0.8;
        b.classes[0].via_drill = 0.4;
        b.obstacles[1].layers = crate::board::LayerSet::single(1);
        b.pins[1].layers = crate::board::LayerSet::single(1);
        b.stackup = [Some(0), None, Some(1)]
            .into_iter()
            .map(|copper| crate::board::StackItem {
                copper,
                thickness: if copper.is_some() { 0.07 } else { 1.5 },
                epsilon_r: 4.5,
            })
            .collect();
        b
    }

    pub(crate) fn line(a: (f64, f64), b: (f64, f64), layer: u8) -> Track {
        let elems = vec![Elem::Line(pt(a.0, a.1), pt(b.0, b.1))];
        Track {
            pts: crate::realize::flatten(&elems),
            elems,
            width: 0.75,
            layer,
            net: 0,
            necks: Vec::new(),
            neck_w: 0.75,
            origin: None,
            resting: false,
            fitted: Vec::new(),
            pattern: None,
            seg: u32::MAX,
            tap: None,
        }
    }

    pub(crate) fn via(x: f64) -> crate::realize::Via {
        crate::realize::Via {
            at: pt(x, 5.),
            diameter: 0.8,
            drill: 0.4,
            net: 0,
            layer: None,
            origin: None,
            resting: false,
            half: Default::default(),
        }
    }

    /// A via pair is a capsule: a track passes its long side closer than the circle around it,
    /// its ends count like round vias.
    #[test]
    fn a_via_pair_is_a_capsule() {
        let mut b = crate::topo::tests::two_pad_board();
        b.classes[0].clearance = 0.1;
        b.nets.push(crate::board::Net {
            name: "B".into(),
            class: 0,
            pins: Vec::new(),
            planes: crate::board::LayerSet::default(),
            priority: 0,
        });
        let track = |a: (f64, f64), c: (f64, f64)| Track {
            width: 0.2,
            neck_w: 0.2,
            net: 1,
            ..line(a, c, 0)
        };
        let pair = |half: Point| crate::realize::Via {
            at: pt(5., 7.5),
            diameter: 1.5,
            drill: 1.2,
            half,
            ..via(5.)
        };
        // 0.65 mm from the line of the vias: 0.3 mm copper, 0.1 mm clearance, 0.1 mm track
        let side = Geometry {
            tracks: vec![track((5.65, 6.5), (5.65, 8.5))],
            vias: vec![pair(pt(0., 0.45))],
        };
        assert!(check(&b, &side).is_empty(), "{:?}", check(&b, &side));
        let round = Geometry {
            vias: vec![pair(pt(0., 0.))],
            ..side.clone()
        };
        assert!(!check(&b, &round).is_empty());
        // past the end of the line it is as wide as a via
        let end = Geometry {
            tracks: vec![track((4., 8.45), (6., 8.45))],
            vias: vec![pair(pt(0., 0.45))],
        };
        assert!(!check(&b, &end).is_empty());
    }

    /// Hole to hole holds for the own net too: a routed via next to a locked via of its net, and
    /// two routed vias of one net; the board minimum width holds in exempt courtyards as well.
    #[test]
    fn same_net_holes_and_the_board_minimum_width() {
        let mut b = current_board();
        b.classes[0].min_width = None;
        b.obstacles.push(crate::board::Obstacle {
            shape: Shape::Circle {
                c: pt(5.0, 5.0),
                r: 0.4,
            },
            layers: b.all_layers(),
            net: Some(0),
            kind: ObstacleKind::Via,
            hole: Some(0.2),
            clearance: 0.2,
            pin: None,
            locked: true,
        });
        // 0.55 mm apart: the holes are 0.15 mm apart, 0.25 needed
        let near = |x: f64| Geometry {
            tracks: Vec::new(),
            vias: vec![via(x)],
        };
        let v = check(&b, &near(5.55));
        assert!(v.iter().any(|v| v.what.starts_with("hole_to_hole")), "{v:?}");
        assert!(
            check(&b, &near(6.0))
                .iter()
                .all(|v| !v.what.starts_with("hole_to_hole"))
        );
        // the kept via itself in the geometry is no violation
        assert!(
            check(&b, &near(5.0))
                .iter()
                .all(|v| !v.what.starts_with("hole_to_hole"))
        );
        let two = Geometry {
            tracks: Vec::new(),
            vias: vec![via(7.0), via(7.55)],
        };
        assert_eq!(
            check(&b, &two)
                .iter()
                .filter(|v| v.what.starts_with("hole_to_hole"))
                .count(),
            1
        );

        // a track tapered to 0.105 mm under a board minimum of 0.127 mm
        let mut t = line((1.0, 2.0), (9.0, 2.0), 0);
        t.width = 0.105;
        t.neck_w = 0.105;
        let geo = Geometry {
            tracks: vec![t],
            vias: Vec::new(),
        };
        assert!(check(&b, &geo).iter().all(|v| !v.what.contains("board minimum")));
        b.rules.kicad_min_width = 0.127;
        let v = check(&b, &geo);
        assert!(
            v.iter().any(|v| v.what.contains("board minimum") && v.is_width()),
            "{v:?}"
        );
    }

    /// IPC-2221 as the rules see it: a 0.4 mm via carries what 0.18 mm of 2 oz outer copper
    /// carry, three carry more than the 0.69 mm of a 4.1 A rule; an inner layer of 1 oz carries
    /// a fifth of the outer one.
    #[test]
    fn copper_carries_by_ipc_2221() {
        let mut b = current_board();
        assert_eq!(b.vias_for(0.69, 0.4), 3);
        assert_eq!(b.vias_for(1.44, 0.4), 5);
        assert!(
            (b.via_current_width([0.4]) - 0.183).abs() < 0.002,
            "{}",
            b.via_current_width([0.4])
        );
        assert!(b.via_current_width([0.4; 3]) > 0.69 && b.via_current_width([0.4; 2]) < 0.69);
        assert_eq!(b.current_width(1, 0.75), 0.75);
        b.layers.insert(1, b.layers[0].clone());
        b.layers.insert(1, b.layers[0].clone());
        b.stackup = [0.07, 0.035, 0.035, 0.07]
            .into_iter()
            .enumerate()
            .map(|(l, thickness)| crate::board::StackItem {
                copper: Some(l as u8),
                thickness,
                epsilon_r: 4.5,
            })
            .collect();
        assert!(
            (b.current_width(1, 0.75) - 0.144).abs() < 0.001,
            "{}",
            b.current_width(1, 0.75)
        );
        // without a stackup: 1 oz, two vias
        b.stackup.clear();
        assert_eq!(b.vias_for(0.69, 0.4), 2);
    }

    /// The current changes layer through the vias there: one via is a neck, three in a row
    /// under the track and the piece of the other layer over them are not.
    #[test]
    fn the_current_counts_its_vias() {
        let b = current_board();
        let current = |tracks: Vec<Track>, vias: Vec<crate::realize::Via>| -> Vec<Violation> {
            let geo = Geometry { tracks, vias };
            let all = check(&b, &geo);
            assert!(all.iter().all(|v| v.is_current()), "{all:?}");
            all
        };
        let one = current(
            vec![line((2., 5.), (5., 5.), 0), line((5., 5.), (8., 5.), 1)],
            vec![via(5.)],
        );
        assert_eq!(one.len(), 1, "{one:?}");
        assert!(
            one[0].at.dist(pt(5., 5.)) < 1e-9 && (one[0].actual - 0.183).abs() < 0.002,
            "{one:?}"
        );
        let two = current(
            vec![
                line((2., 5.), (5., 5.), 0),
                line((5., 5.), (8., 5.), 1),
                line((5., 5.), (4.33, 5.), 1),
            ],
            vec![via(5.), via(4.33)],
        );
        assert!(two.len() == 1 && two[0].actual < 0.69, "{two:?}");
        let three = current(
            vec![
                line((2., 5.), (5., 5.), 0),
                line((5., 5.), (8., 5.), 1),
                line((5., 5.), (3.66, 5.), 1),
            ],
            vec![via(5.), via(4.33), via(3.66)],
        );
        assert!(three.is_empty(), "{three:?}");
        // a via the bottom track only passes by is not one of them
        let apart = current(
            vec![line((2., 5.), (5., 5.), 0), line((5., 5.), (8., 5.), 1)],
            vec![via(5.), via(6.), via(7.)],
        );
        assert_eq!(apart.len(), 1, "{apart:?}");
    }

    /// Below the minimum width only where the piece touches the courtyard of a part the rule
    /// exempts, the way KiCad's rule reads.
    #[test]
    fn a_narrow_piece_counts_out_of_the_courtyard() {
        let mut b = current_board();
        b.classes[0].neck_refs = vec!["U1".into()];
        let mut yard = Rect::empty();
        yard.add(pt(4., 4.));
        yard.add(pt(6., 6.));
        b.courtyards.insert("U1".into(), yard);
        let mut zone = Rect::empty();
        zone.add(pt(4., 4.));
        zone.add(pt(9., 6.));
        let mut t = line((5., 5.), (6.5, 5.), 0);
        t.elems.push(Elem::Line(pt(6.5, 5.), pt(8., 5.)));
        t.pts = crate::realize::flatten(&t.elems);
        t.necks = vec![zone];
        t.neck_w = 0.25;
        let geo = Geometry {
            tracks: vec![t],
            vias: Vec::new(),
        };
        let v: Vec<Violation> = check(&b, &geo)
            .into_iter()
            .filter(|v| v.is_width() && !v.is_current())
            .collect();
        assert!(
            v.len() == 1 && v[0].at.dist(pt(7.25, 5.)) < 1e-9 && v[0].actual == 0.25,
            "{v:?}"
        );
    }

    /// The current between two wide pins may not pass the neck of a fine pitch pin; a branch to
    /// that pin may be as narrow as it likes.
    #[test]
    fn the_current_never_passes_a_neck() {
        let mut b = crate::topo::tests::two_pad_board();
        b.classes[0].track_width = 0.75;
        b.classes[0].min_width = Some(0.69);
        b.classes[0].neck_refs = vec!["U1".into()];
        let mut o = b.obstacles[0].clone();
        o.shape = Shape::Polygon(Polygon::new(vec![
            pt(4.875, 4.6),
            pt(5.125, 4.6),
            pt(5.125, 5.4),
            pt(4.875, 5.4),
        ]));
        o.pin = Some(2);
        b.obstacles.push(o);
        let mut u = b.pins[0].clone();
        u.name = "U1.1".into();
        u.footprint = "U1".into();
        u.pos = pt(5., 5.);
        u.obstacles = vec![2];
        u.size = (0.25, 0.8);
        b.pins.push(u);
        b.nets[0].pins = vec![0, 1, 2];
        let mut zone = Rect::empty();
        zone.add(pt(4.6, 4.5));
        zone.add(pt(5.4, 5.5));
        let track = |pts: &[(f64, f64)], width: f64| {
            let elems: Vec<Elem> = pts
                .windows(2)
                .map(|w| Elem::Line(pt(w[0].0, w[0].1), pt(w[1].0, w[1].1)))
                .collect();
            Track {
                pts: crate::realize::flatten(&elems),
                elems,
                width,
                layer: 0,
                net: 0,
                necks: vec![zone],
                neck_w: 0.25,
                origin: None,
                resting: false,
                fitted: Vec::new(),
                pattern: None,
                seg: u32::MAX,
                tap: None,
            }
        };
        let current = |tracks: Vec<Track>| -> Vec<Violation> {
            let geo = Geometry {
                tracks,
                vias: Vec::new(),
            };
            check(&b, &geo).into_iter().filter(|v| v.is_current()).collect()
        };
        // through the pin, with its neck on both sides
        let through = current(vec![
            track(&[(2., 5.), (4.5, 5.), (5., 5.)], 0.75),
            track(&[(5., 5.), (5.5, 5.), (8., 5.)], 0.75),
        ]);
        assert_eq!(through.len(), 1, "{through:?}");
        assert!(through[0].actual == 0.25 && through[0].required == 0.69 && through[0].at.dist(pt(5., 5.)) < 0.5);
        // wide tracks that end apart in the narrow pad: the pad is the neck
        let pad = current(vec![
            track(&[(2., 4.7), (4.5, 4.7), (5., 4.7)], 0.75),
            track(&[(5., 5.3), (5.5, 5.3), (8., 5.3)], 0.75),
        ]);
        assert!(pad.len() == 1 && pad[0].actual == 0.25, "{pad:?}");
        // around it at full width, and a narrow branch to the pin
        let around = current(vec![
            track(&[(2., 5.), (2., 7.), (8., 7.), (8., 5.)], 0.75),
            track(&[(5., 5.), (6., 6.), (8., 5.)], 0.25),
        ]);
        assert!(around.is_empty(), "{around:?}");
    }
}
