//! Clearance check of routed geometry against the board and itself.

use rayon::prelude::*;

use crate::board::{Board, NetId, ObstacleKind};
use crate::geom::{Rect, Shape, point_seg_dist, seg_seg_dist};
use crate::index::GridIndex;
use crate::realize::Geometry;

#[derive(Debug, Clone)]
pub struct Violation {
    pub what: String,
    pub at: crate::geom::Point,
    pub actual: f64,
    pub required: f64,
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
        net: NetId,
    },
}

pub fn check(board: &Board, geo: &Geometry) -> Vec<Violation> {
    let routing = board.routing_layers();
    let mut pieces = Vec::new();
    for t in &geo.tracks {
        for w in t.pts.windows(2) {
            pieces.push(Piece::Seg {
                a: w[0],
                b: w[1],
                r: t.width / 2.0,
                layer: t.layer,
                net: t.net,
            });
        }
    }
    for v in &geo.vias {
        pieces.push(Piece::Via {
            c: v.at,
            r: v.diameter / 2.0,
            net: v.net,
        });
    }
    let bbox = |p: &Piece| -> Rect {
        let mut r = Rect::empty();
        match *p {
            Piece::Seg { a, b, r: rr, .. } => {
                r.add(a);
                r.add(b);
                r.grow(rr)
            }
            Piece::Via { c, r: rr, .. } => {
                r.add(c);
                r.grow(rr)
            }
        }
    };
    let mut idx = GridIndex::new(board.bbox().grow(5.0), 1.0);
    for (i, p) in pieces.iter().enumerate() {
        idx.insert(i as u32, &bbox(p));
    }
    let mut oidx = GridIndex::new(board.bbox().grow(5.0), 1.0);
    for (i, o) in board.obstacles.iter().enumerate() {
        oidx.insert(i as u32, &o.shape.bbox());
    }
    let clr = |n: NetId| board.class_of(n).clearance.max(board.rules.min_clearance);
    pieces
        .par_iter()
        .enumerate()
        .flat_map_iter(|(i, p)| {
            let mut out = Vec::new();
            let (net, layer_has): (NetId, Box<dyn Fn(u8) -> bool>) = match *p {
                Piece::Seg { net, layer, .. } => (net, Box::new(move |l| l == layer)),
                Piece::Via { net, .. } => (net, Box::new(|_| true)),
            };
            let shape = match *p {
                Piece::Seg { a, b, r, .. } => Shape::Capsule { a, b, r },
                Piece::Via { c, r, .. } => Shape::Circle { c, r },
            };
            let at = match *p {
                Piece::Seg { a, b, .. } => a.lerp(b, 0.5),
                Piece::Via { c, .. } => c,
            };
            let bb = bbox(p).grow(1.0);
            // board edge
            let edge = match *p {
                Piece::Seg { a, b, r, .. } => {
                    board
                        .outline
                        .iter()
                        .map(|o| {
                            o.edges()
                                .map(|(c, d)| seg_seg_dist(a, b, c, d))
                                .fold(f64::INFINITY, f64::min)
                        })
                        .fold(f64::INFINITY, f64::min)
                        - r
                }
                Piece::Via { c, r, .. } => {
                    board
                        .outline
                        .iter()
                        .map(|o| o.boundary_dist(c))
                        .fold(f64::INFINITY, f64::min)
                        - r
                }
            };
            if edge < board.rules.edge_clearance - 1e-4 {
                out.push(Violation {
                    what: "edge".into(),
                    at,
                    actual: edge,
                    required: board.rules.edge_clearance,
                });
            }
            for j in oidx.query(&bb) {
                let o = &board.obstacles[j as usize];
                if o.net == Some(net) && o.kind != ObstacleKind::Hole {
                    continue;
                }
                if !(o.layers.iter().any(|l| routing.has(l) && layer_has(l))
                    || (o.hole.is_some() && matches!(p, Piece::Via { .. })))
                {
                    continue;
                }
                let need = match o.kind {
                    ObstacleKind::Keepout { tracks, vias } => {
                        let applies = match p {
                            Piece::Seg { .. } => tracks,
                            Piece::Via { .. } => vias,
                        };
                        if !applies {
                            continue;
                        }
                        0.0
                    }
                    ObstacleKind::Hole => board.rules.hole_clearance,
                    _ => o.clearance.max(clr(net)),
                };
                let d = o.shape.dist(&shape);
                if d < need - 1e-4 {
                    out.push(Violation {
                        what: format!(
                            "{:?} {}~{:?}",
                            o.kind,
                            board.nets[net as usize].name,
                            o.net.map(|n| board.nets[n as usize].name.clone())
                        ),
                        at,
                        actual: d,
                        required: need,
                    });
                }
            }
            for j in idx.query(&bb) {
                if j as usize <= i {
                    continue;
                }
                let q = &pieces[j as usize];
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
                    (Piece::Seg { a, b, r, .. }, Piece::Via { c, r: r2, net: n2 })
                    | (Piece::Via { c, r: r2, net: n2 }, Piece::Seg { a, b, r, .. }) => {
                        let n = if matches!(p, Piece::Via { .. }) {
                            match q {
                                Piece::Seg { net, .. } => *net,
                                _ => *n2,
                            }
                        } else {
                            *n2
                        };
                        (n, point_seg_dist(*c, *a, *b) - r - r2, true)
                    }
                    (Piece::Via { c, r, .. }, Piece::Via { c: c2, r: r2, net: n2 }) => {
                        (*n2, c.dist(*c2) - r - r2, true)
                    }
                };
                if qnet == net || !both_layers {
                    continue;
                }
                let need = clr(net).max(clr(qnet));
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
                        at,
                        actual: d,
                        required: need,
                    });
                }
            }
            out
        })
        .collect()
}
