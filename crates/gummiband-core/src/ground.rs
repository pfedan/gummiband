//! How whole the copper area of a net stays on a layer once the board is routed.
//!
//! A ground area does its work where it is one piece: the return current of a signal runs in
//! the area right under the track, and a track of another net that cuts the area sends it
//! around the cut. The fill model ([`crate::fill`]) lays the area on the copper as it is;
//! this counts what is left of it: into how many pieces it falls, how much of it the largest
//! piece holds, and how many pads of the net sit on a piece cut off from that one.

use crate::board::{Board, LayerId, NetId};
use crate::fill::{Area, NetFill, Options, fill_net};

/// The area of one net on one layer, counted by its pieces.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct Whole {
    pub layer: LayerId,
    /// Pieces of fill on the layer, floating ones included.
    pub pieces: usize,
    /// mm² of fill on the layer, and of its largest piece.
    pub area: f64,
    pub largest: f64,
    /// Pads of the net on pieces other than the largest.
    pub pads_apart: usize,
}

impl Whole {
    /// The share of the fill that lies in the largest piece: 1 for an area in one piece.
    pub fn share(&self) -> f64 {
        if self.area > 0.0 { self.largest / self.area } else { 0.0 }
    }
}

/// The pieces of a fill, layer by layer (layers without fill are left out).
pub fn wholeness(f: &NetFill) -> Vec<Whole> {
    let mut out: Vec<Whole> = Vec::new();
    for p in &f.pieces {
        let w = match out.iter_mut().find(|w| w.layer == p.layer) {
            Some(w) => w,
            None => {
                out.push(Whole {
                    layer: p.layer,
                    pieces: 0,
                    area: 0.0,
                    largest: 0.0,
                    pads_apart: 0,
                });
                out.last_mut().unwrap()
            }
        };
        w.pieces += 1;
        w.area += p.area;
        w.largest = w.largest.max(p.area);
    }
    for w in &mut out {
        w.pads_apart = f
            .pieces
            .iter()
            .filter(|p| p.layer == w.layer && p.area < w.largest)
            .map(|p| p.pins.len())
            .sum();
    }
    out.sort_by_key(|w| w.layer);
    out
}

/// A pour of `net` over the whole board on each of `layers`, laid on the copper of `board`
/// (pads, kept tracks and vias; [`crate::fill::with_copper`] puts a routed result there), and
/// how whole it is. Every piece counts, also one that holds nothing of the net.
pub fn pour_wholeness(board: &Board, net: NetId, layers: &[LayerId], o: &Options) -> Vec<Whole> {
    let areas: Vec<Area> = layers
        .iter()
        .map(|&l| {
            let mut a = Area::pour(net, l, board.outline.clone());
            a.island_min = 0.0;
            a
        })
        .collect();
    wholeness(&fill_net(board, &areas, net, o))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{LayerSet, Net, Obstacle, ObstacleKind};
    use crate::geom::{Polygon, Shape, pt};

    #[test]
    fn a_track_across_the_board_leaves_the_area_in_two_pieces() {
        let mut b = crate::topo::tests::two_pad_board();
        b.outline = vec![Polygon::new(vec![pt(0., 0.), pt(20., 0.), pt(20., 20.), pt(0., 20.)])];
        b.pins.clear();
        b.obstacles.clear();
        b.nets = ["GND", "S"]
            .iter()
            .map(|n| Net {
                name: n.to_string(),
                class: 0,
                pins: Vec::new(),
                planes: LayerSet::default(),
                priority: 0,
            })
            .collect();
        let o = Options::default();
        let whole = pour_wholeness(&b, 0, &[0], &o);
        assert_eq!(whole.len(), 1);
        assert_eq!((whole[0].pieces, whole[0].pads_apart), (1, 0));
        assert!((whole[0].share() - 1.0).abs() < 1e-9);
        // a track of another net from edge to edge, a quarter of the way in
        b.obstacles.push(Obstacle {
            shape: Shape::Capsule {
                a: pt(5., 0.),
                b: pt(5., 20.),
                r: 0.1,
            },
            layers: LayerSet::single(0),
            net: Some(1),
            kind: ObstacleKind::Track,
            hole: None,
            clearance: 0.2,
            pin: None,
            locked: false,
        });
        let whole = pour_wholeness(&b, 0, &[0, 1], &o);
        assert_eq!(whole.len(), 2);
        assert_eq!(whole[0].pieces, 2);
        assert!(whole[0].share() > 0.7 && whole[0].share() < 0.8, "{:?}", whole[0]);
        // the other layer is untouched
        assert_eq!(whole[1].pieces, 1);
    }
}
