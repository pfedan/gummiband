//! Return vias: a via of the ground net next to each via of a fast signal. Where a signal changes
//! layers its return current changes reference plane; a ground via close by gives it a short way
//! between the planes (layout guides ask for one within about 0.5 to 0.75 mm, for every via of a
//! DDR signal). Placed on the finished geometry, only where it keeps every clearance.

use crate::board::{Board, NetId};
use crate::check::Copper;
use crate::geom::{Point, pt};
use crate::realize::{Geometry, Via};

/// What [`return_vias`] did.
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct ReturnVias {
    /// The ground net the vias belong to (none: the board has no ground plane).
    pub ground: Option<String>,
    /// Signal vias that needed one.
    pub signal_vias: usize,
    /// Of those, the ones that had a ground via (or ground hole) in reach already.
    pub had_one: usize,
    /// Vias added.
    pub added: usize,
    /// Signal vias without room for one in reach: net and position.
    pub missing: Vec<(String, Point)>,
}

/// The ground net: the net of the largest plane whose name says ground (GND, VSS, ground), else
/// none.
pub fn ground_net(board: &Board) -> Option<NetId> {
    let ground = |n: NetId| {
        let name = board.nets[n as usize].name.to_ascii_uppercase();
        name.contains("GND") || name.contains("VSS") || name.contains("GROUND")
    };
    board
        .planes
        .iter()
        .filter(|p| ground(p.net))
        .max_by(|a, b| a.outline.signed_area().abs().total_cmp(&b.outline.signed_area().abs()))
        .map(|p| p.net)
}

/// The nets that get return vias by default: the nets of the length, skew and differential pair
/// rules, and the nets of differential pairs.
pub fn fast_nets(board: &Board) -> Vec<NetId> {
    let mut out: Vec<NetId> = board.match_rules.iter().flat_map(|r| r.nets.iter().copied()).collect();
    out.extend(board.diff_pairs().into_iter().flat_map(|(p, n)| [p, n]));
    out.sort_unstable();
    out.dedup();
    out
}

/// Give every via of `nets` a via of the ground net at most `reach` mm away (centre to centre),
/// the closest place that keeps the clearances of the check, the hole to hole distance and the
/// board edge. A signal via that has a ground via or a ground hole in reach already keeps it.
pub fn return_vias(board: &Board, geo: &mut Geometry, nets: &[NetId], reach: f64) -> ReturnVias {
    let mut rep = ReturnVias::default();
    let Some(gnd) = ground_net(board) else {
        return rep;
    };
    rep.ground = Some(board.nets[gnd as usize].name.clone());
    let cls = board.class_of(gnd);
    let (dia, drill) = (cls.via_diameter, cls.via_drill);
    let gap = board.rules.hole_to_hole;
    let signal: Vec<usize> = (0..geo.vias.len())
        .filter(|&i| geo.vias[i].drill > 0.0 && nets.contains(&geo.vias[i].net))
        .collect();
    // ground holes of the board (through hole pads of the net)
    let ground_holes: Vec<Point> = board
        .obstacles
        .iter()
        .filter(|o| o.net == Some(gnd) && o.hole.is_some())
        .map(|o| o.shape.bbox().center())
        .collect();
    let mut copper = Copper::new(board, geo);
    for vi in signal {
        let v = geo.vias[vi].clone();
        rep.signal_vias += 1;
        let near = |p: Point| p.dist(v.at) <= reach + 1e-9;
        let has = geo.vias.iter().any(|w| w.net == gnd && w.drill > 0.0 && near(w.at))
            || ground_holes.iter().any(|&h| near(h));
        if has {
            rep.had_one += 1;
            continue;
        }
        let hole_room = |at: Point, geo: &Geometry| {
            geo.vias
                .iter()
                .filter(|w| w.drill > 0.0)
                .all(|w| w.at.dist(at) >= (w.drill + drill) / 2.0 + gap - 1e-9)
                && board
                    .obstacles
                    .iter()
                    .filter(|o| o.hole.is_some())
                    .all(|o| o.shape.bbox().center().dist(at) >= o.hole.unwrap() + drill / 2.0 + gap - 1e-9)
        };
        // closest first: rings from where the two vias just clear each other out to the reach
        let inner = (v.diameter + dia) / 2.0 + cls.clearance.max(board.rules.min_clearance);
        let mut found = None;
        let steps = 6;
        'search: for k in 0..=steps {
            let r = inner + (reach - inner).max(0.0) * k as f64 / steps as f64;
            if r > reach + 1e-9 {
                break;
            }
            for i in 0..16 {
                let a = i as f64 * std::f64::consts::PI / 8.0;
                let at = v.at + pt(a.cos(), a.sin()) * r;
                if hole_room(at, geo) && copper.clear(gnd, &[], &[(at, dia)]) {
                    found = Some(at);
                    break 'search;
                }
            }
        }
        match found {
            Some(at) => {
                geo.vias.push(Via {
                    at,
                    diameter: dia,
                    drill,
                    net: gnd,
                    layer: None,
                    origin: None,
                    resting: false,
                    half: Point::default(),
                });
                rep.added += 1;
                copper = Copper::new(board, geo);
            }
            None => rep.missing.push((board.nets[v.net as usize].name.clone(), v.at)),
        }
    }
    rep
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{Net, Plane};
    use crate::check::tests::{line, via};
    use crate::geom::Polygon;

    /// A via of a fast net gets a ground via within reach that keeps the clearances; a second
    /// run adds nothing, and a board without a ground plane gets none.
    #[test]
    fn a_fast_via_gets_a_ground_via_next_to_it() {
        let mut b = crate::topo::tests::two_pad_board();
        b.classes[0].via_diameter = 0.6;
        b.classes[0].via_drill = 0.3;
        let geo = || Geometry {
            tracks: vec![line((2., 5.), (5., 5.), 0), line((5., 5.), (8., 5.), 1)],
            vias: vec![Via {
                diameter: 0.6,
                drill: 0.3,
                ..via(5.)
            }],
        };
        let mut g = geo();
        let none = return_vias(&b, &mut g, &[0], 1.0);
        assert!(none.ground.is_none());
        assert_eq!(g.vias.len(), 1);
        b.nets.push(Net {
            name: "GND".into(),
            class: 0,
            pins: Vec::new(),
            planes: Default::default(),
            priority: 0,
        });
        let all = Polygon::new(vec![pt(0., 0.), pt(10., 0.), pt(10., 10.), pt(0., 10.)]);
        b.planes.push(Plane::new(1, 1, all));
        let rep = return_vias(&b, &mut g, &[0], 1.0);
        assert_eq!((rep.signal_vias, rep.added, rep.missing.len()), (1, 1, 0));
        let gv = g.vias.iter().find(|v| v.net == 1).unwrap();
        assert!(gv.at.dist(pt(5., 5.)) <= 1.0 + 1e-9);
        let v = crate::check::check(&b, &g);
        assert!(v.is_empty(), "{v:?}");
        let again = return_vias(&b, &mut g, &[0], 1.0);
        assert_eq!((again.had_one, again.added), (1, 0));
        // out of reach: no room within 0.5 mm for two vias of 0.6 mm with 0.2 mm between them
        let mut g = geo();
        let rep = return_vias(&b, &mut g, &[0], 0.5);
        assert_eq!((rep.added, rep.missing.len()), (0, 1));
    }
}
