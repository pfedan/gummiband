//! Moving a part on a routed board the rubber band way: the copper around it goes along.
//!
//! All unlocked copper becomes soft copper (see [`Board::soften`]) and is taken as it is, no
//! routing. The part then moves in small steps, never farther per step than the smallest
//! clearance, so that no pad can jump over a track. After every step the track ends and vias on
//! its pads go along with the pads, and the relaxation lets the tracks give way: the tracks of
//! the part's own nets stretch, those of other nets are pushed aside within their clearances,
//! and they push their own neighbours in turn. Copper that nothing touches sleeps and stays as
//! the file has it. When a step leaves a clearance problem the bands cannot solve (a track
//! clamped between the moving pad and fixed copper, the board edge, or a track that would have
//! to cross a pad), or the courtyard runs into another part or off the board, the part stops at
//! the last step that was clean.

use std::collections::HashMap;

use crate::board::Board;
use crate::check;
use crate::geom::{Point, Rect};
use crate::import::SoftChanges;
use crate::realize::{Geometry, Track, Via};
use crate::relax::{self, RelaxConfig};
use crate::topo::{Substrate, SubstrateConfig};

/// What [`move_with_copper`] did.
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct FollowStats {
    /// The move asked for and the one reached, in mm.
    pub asked: (f64, f64),
    pub reached: (f64, f64),
    /// Steps taken, of how many.
    pub steps: usize,
    pub steps_planned: usize,
    /// Why the part stopped short, if it did.
    pub stopped: Option<String>,
    /// Clearance spots before and after (gummiband's own check).
    pub spots_before: usize,
    pub spots_after: usize,
    /// Tracks and vias that moved, and how far a track got from where it was at most.
    pub tracks_moved: usize,
    pub vias_moved: usize,
    pub nets_moved: usize,
    pub max_shift: f64,
}

/// The footprints that move with `fp`: the members of its block, or `fp` alone.
pub fn moving_refs(board: &Board, fp: &str) -> Vec<String> {
    board
        .blocks
        .iter()
        .find(|b| b.refs.iter().any(|r| r == fp))
        .map_or_else(|| vec![fp.to_string()], |b| b.refs.clone())
}

/// Move footprint `fp` (with its block) by `d` and let the copper around it follow. `board` is
/// read with all copper kept and gets softened here. Returns the geometry of all soft copper
/// (origins set, `resting` where it did not move), what to write again, and the offset reached.
pub fn move_with_copper(board: &mut Board, fp: &str, d: Point) -> (Geometry, SoftChanges, Point, FollowStats) {
    board.soften(&[]);
    let board = &*board;
    let refs = moving_refs(board, fp);
    let geo = soft_geometry(board);
    // no unlocked copper: nothing to follow, the part goes where it was asked to
    if geo.tracks.is_empty() && geo.vias.is_empty() {
        let st = FollowStats {
            asked: (d.x, d.y),
            reached: (d.x, d.y),
            ..Default::default()
        };
        return (geo, SoftChanges::default(), d, st);
    }
    let to: HashMap<String, Point> = refs.iter().map(|r| (r.clone(), d)).collect();
    let mut f = follow_steps(
        board,
        &HashMap::new(),
        &to,
        &geo,
        &SubstrateConfig::default(),
        // vias keep out of the edge and stay in their planes, tracks leave fine pitch pads straight
        &RelaxConfig {
            via_edge: true,
            ..Default::default()
        },
    );
    let reached = f.offsets.get(fp).copied().unwrap_or_default();
    let mut st = f.stats;
    st.asked = (d.x, d.y);
    st.reached = (reached.x, reached.y);
    // resting: exactly as the file has it
    for (i, t) in f.geo.tracks.iter_mut().enumerate() {
        let old = &board.soft.chains[i];
        t.resting = t.pts.len() == old.pts.len() && t.pts.iter().zip(&old.pts).all(|(a, b)| a.dist(*b) < 1e-6);
        if t.resting {
            t.elems = old.elems.clone();
            t.fitted.clear();
        } else if t.elems.is_empty() {
            t.elems = t
                .pts
                .windows(2)
                .map(|w| crate::realize::Elem::Line(w[0], w[1]))
                .collect();
        }
    }
    for (i, v) in f.geo.vias.iter_mut().enumerate() {
        v.resting = v.at.dist(board.soft.nodes[i].at) < 1e-6;
    }
    let changes = crate::import::soft_changes(board, &f.geo);
    st.tracks_moved = changes.chains.iter().filter(|c| **c).count();
    st.vias_moved = f.geo.vias.iter().filter(|v| !v.resting && v.drill > 0.0).count();
    st.nets_moved = changes.nets.len();
    st.max_shift = changes.shift;
    (f.geo, changes, reached, st)
}

/// What [`follow_steps`] reached.
#[derive(Debug, Clone)]
pub struct Followed {
    /// The geometry after the last clean step.
    pub geo: Geometry,
    /// The footprint offsets of that step (all of `from`, updated where parts moved).
    pub offsets: HashMap<String, Point>,
    pub stats: FollowStats,
}

/// Move the parts from the offsets `from` towards `to` in small steps; after every step the
/// copper of `geo` (on `board` with `from` applied) follows: track ends and vias on moved pads
/// go along, what the pads now press against is woken, and the relaxation lets it give way.
/// Copper that is asleep (`resting`) and not touched stays as it is. Stops before the first step
/// that leaves a clearance spot that was not there or got worse (see [`is_new`]), or where a
/// courtyard runs into another part or off the board.
pub fn follow_steps(
    board: &Board,
    from: &HashMap<String, Point>,
    to: &HashMap<String, Point>,
    geo: &Geometry,
    sub_cfg: &SubstrateConfig,
    cfg: &RelaxConfig,
) -> Followed {
    let at = |m: &HashMap<String, Point>, r: &str| m.get(r).copied().unwrap_or_default();
    let mut refs: Vec<String> = to
        .keys()
        .chain(from.keys())
        .filter(|r| at(from, r).dist(at(to, r)) > 1e-9)
        .cloned()
        .collect();
    refs.sort();
    refs.dedup();
    let longest = refs.iter().map(|r| at(from, r).dist(at(to, r))).fold(0.0, f64::max);
    let start = board.with_offsets(from);
    let base_spots = check::spots(&check::check(&start, geo));
    let base = base_spots.len();
    // never farther per step than the smallest clearance: a pad cannot pass a track in one step
    let step = board.rules.min_clearance.clamp(0.02, 0.1);
    let n = (longest / step).ceil() as usize;
    let mut st = FollowStats {
        steps_planned: n,
        spots_before: base,
        spots_after: base,
        ..Default::default()
    };
    let others: Vec<(&String, &Rect)> = start.courtyards.iter().filter(|(r, _)| !refs.contains(r)).collect();
    let overlapping = |cy: &HashMap<String, Rect>| -> Vec<String> {
        let mut v: Vec<String> = refs
            .iter()
            .filter_map(|r| cy.get(r))
            .flat_map(|c| {
                let c = c.grow(-0.01);
                others
                    .iter()
                    .filter(move |(_, o)| c.overlaps(o))
                    .map(|(r, _)| (*r).clone())
            })
            .collect();
        v.sort();
        v.dedup();
        v
    };
    let touching_before = overlapping(&start.courtyards);
    let off = |k: usize| -> HashMap<String, Point> {
        let mut m = from.clone();
        for r in &refs {
            let (a, b) = (at(from, r), at(to, r));
            m.insert(r.clone(), a + (b - a) * (k as f64 / n as f64));
        }
        m
    };
    let mut geo = geo.clone();
    let mut cur = start.clone();
    let mut offsets = from.clone();
    for k in 1..=n {
        let next_off = off(k);
        let next = board.with_offsets(&next_off);
        // courtyards: no new overlap, inside the board
        if let Some(r) = overlapping(&next.courtyards)
            .into_iter()
            .find(|r| !touching_before.contains(r))
        {
            st.stopped = Some(format!("courtyard runs into {r}"));
            break;
        }
        let outside = refs.iter().filter_map(|r| next.courtyards.get(r)).any(|c| {
            [
                c.min,
                c.max,
                crate::geom::pt(c.min.x, c.max.y),
                crate::geom::pt(c.max.x, c.min.y),
            ]
            .iter()
            .any(|&p| !next.inside(p))
        });
        if outside {
            st.stopped = Some("courtyard leaves the board".into());
            break;
        }
        let mut g = crate::follow_parts(&cur, &next, &geo);
        wake_near(&next, &refs, &mut g, &geo);
        let sub = Substrate::build(&next, sub_cfg);
        relax::relax(&next, &sub, &mut g, cfg);
        let spots = check::spots(&check::check(&next, &g));
        if let Some(s) = spots.iter().find(|s| is_new(s, &base_spots, longest)) {
            let name = |n: Option<u32>| n.map_or("fixed copper".to_string(), |n| board.nets[n as usize].name.clone());
            st.stopped = Some(format!(
                "no more room at ({:.2}, {:.2}): {} and {}, {:.3} mm missing",
                s.at.x,
                s.at.y,
                name(Some(s.nets.0)),
                name(s.nets.1),
                s.missing
            ));
            break;
        }
        st.spots_after = spots.len();
        st.steps = k;
        offsets = next_off;
        geo = g;
        cur = next;
    }
    Followed {
        geo,
        offsets,
        stats: st,
    }
}

/// A clearance spot that was not there before the move, or got worse: no spot of the same two
/// nets within reach (the moved distance plus the size of a spot), or one with less missing. A
/// problem that only moves from one pair of nets to another counts as new, one that only splits
/// into pieces does not.
fn is_new(s: &check::Spot, before: &[check::Spot], moved: f64) -> bool {
    // a micrometre is rounding, not a problem
    if s.missing < 0.001 && !s.short {
        return false;
    }
    let key = |n: (u32, Option<u32>)| {
        let b = n.1.unwrap_or(u32::MAX);
        (n.0.min(b), n.0.max(b))
    };
    !before.iter().any(|b| {
        key(b.nets) == key(s.nets)
            && b.at.dist(s.at) < 2.0 + moved
            && s.missing <= b.missing + 0.001
            && (b.short || !s.short)
    })
}

/// All soft copper as geometry, asleep, each item with its origin.
pub fn soft_geometry(board: &Board) -> Geometry {
    let mut geo = Geometry::default();
    for (i, ch) in board.soft.chains.iter().enumerate() {
        geo.tracks.push(Track {
            pts: ch.pts.clone(),
            elems: ch.elems.clone(),
            width: ch.width,
            layer: ch.layer,
            net: ch.net,
            necks: Vec::new(),
            neck_w: ch.width,
            origin: Some(i as u32),
            resting: true,
            fitted: Vec::new(),
            pattern: None,
            seg: u32::MAX,
            tap: None,
        });
    }
    for (i, n) in board.soft.nodes.iter().enumerate() {
        geo.vias.push(Via {
            at: n.at,
            diameter: n.r * 2.0,
            drill: n.drill,
            net: n.net,
            layer: n.layer,
            origin: Some(i as u32),
            resting: true,
            half: Default::default(),
        });
    }
    geo
}

/// Wake what the step touched: tracks and vias that went along with a pad, and copper of other
/// nets that the moved pads now come too near. What only stands close keeps sleeping, so that
/// it keeps its shape. The relaxation wakes what these touch in turn.
fn wake_near(next: &Board, refs: &[String], g: &mut Geometry, before: &Geometry) {
    let obs: Vec<usize> = refs
        .iter()
        .filter_map(|r| next.footprint_obstacles.get(r))
        .flatten()
        .copied()
        .collect();
    let reach =
        |net: u32, half: f64, o: &crate::board::Obstacle| half + o.clearance.max(next.class_of(net).clearance) - 0.001;
    for (t, old) in g.tracks.iter_mut().zip(&before.tracks) {
        if t.pts != old.pts {
            t.resting = false;
            continue;
        }
        if !t.resting {
            continue;
        }
        let near = obs.iter().map(|&i| &next.obstacles[i]).any(|o| {
            o.layers.has(t.layer)
                && o.net != Some(t.net)
                && t.pts.windows(2).any(|w| {
                    let r = reach(t.net, t.width / 2.0, o);
                    o.shape.dist_to_seg(w[0], w[1]) < r
                })
        });
        if near {
            t.resting = false;
        }
    }
    for (v, old) in g.vias.iter_mut().zip(&before.vias) {
        if v.at.dist(old.at) > 1e-12 {
            v.resting = false;
            continue;
        }
        if v.resting
            && obs
                .iter()
                .map(|&i| &next.obstacles[i])
                .any(|o| o.net != Some(v.net) && o.shape.dist_to_point(v.at) < reach(v.net, v.diameter / 2.0, o))
        {
            v.resting = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::*;
    use crate::geom::{Polygon, Shape, pt, roundrect_polygon};

    /// Net A runs as an unlocked track from the pad P0 at (2, 5) to P1 at (8, 5). Part Q (net B,
    /// one 0.6 mm pad) sits above it at (5, 3.5). `min_clearance` 0.1 gives steps of 0.1 mm.
    fn board() -> Board {
        let mut b = crate::topo::tests::two_pad_board();
        b.layers.truncate(1);
        b.rules.min_clearance = 0.1;
        b.nets.push(Net {
            name: "B".into(),
            class: 0,
            pins: vec![2],
            planes: LayerSet::default(),
            priority: 0,
        });
        let q = b.obstacles.len();
        b.obstacles.push(Obstacle {
            shape: Shape::Polygon(roundrect_polygon(pt(5.0, 3.5), 0.6, 0.6, 0.0, 0.0, 0.005)),
            layers: LayerSet::single(0),
            net: Some(1),
            kind: ObstacleKind::Pad,
            hole: None,
            clearance: 0.2,
            pin: Some(2),
            locked: true,
        });
        b.pins.push(Pin {
            name: "Q.1".into(),
            footprint: "Q".into(),
            net: 1,
            pos: pt(5.0, 3.5),
            layers: LayerSet::single(0),
            obstacles: vec![q],
            size: (0.6, 0.6),
        });
        b.footprint_obstacles.insert("Q".into(), vec![q]);
        b.courtyards.insert(
            "Q".into(),
            Rect {
                min: pt(4.6, 3.1),
                max: pt(5.4, 3.9),
            },
        );
        for (i, r) in ["P0", "P1"].iter().enumerate() {
            b.footprint_obstacles.insert(r.to_string(), vec![i]);
            let x = [2.0, 8.0][i];
            b.courtyards.insert(
                r.to_string(),
                Rect {
                    min: pt(x - 0.6, 4.4),
                    max: pt(x + 0.6, 5.6),
                },
            );
        }
        let first = b.obstacles.len() as u32;
        let (a, z) = (pt(2.0, 5.0), pt(8.0, 5.0));
        b.obstacles.push(Obstacle {
            shape: Shape::Capsule { a, b: z, r: 0.1 },
            layers: LayerSet::single(0),
            net: Some(0),
            kind: ObstacleKind::Track,
            hole: None,
            clearance: 0.2,
            pin: None,
            locked: false,
        });
        b.copper.push(CopperItem {
            kind: CopperKind::Line {
                a,
                b: z,
                width: 0.2,
                layer: 0,
            },
            net: Some(0),
            locked: false,
            source: 7,
            first,
            last: first + 1,
            pattern: false,
        });
        b
    }

    fn lowest(t: &Track) -> f64 {
        t.pts.iter().map(|p| p.y).fold(f64::MIN, f64::max)
    }

    #[test]
    fn a_track_of_another_net_gives_way_to_the_moving_part() {
        let mut b = board();
        // the pad ends where the track was: the track has to bend around below it
        let (geo, changes, reached, st) = move_with_copper(&mut b, "Q", pt(0.0, 1.5));
        assert!(st.stopped.is_none(), "{st:?}");
        assert!(reached.dist(pt(0.0, 1.5)) < 1e-9);
        assert_eq!(st.steps, st.steps_planned);
        let moved = b.with_offsets(&HashMap::from([("Q".to_string(), reached)]));
        assert!(check::check(&moved, &geo).is_empty());
        assert_eq!(changes.chains, vec![true]);
        assert_eq!(changes.strip, vec![7]);
        // pad bottom 5.3, clearance 0.2, half width 0.1: the track passes at 5.6 or below
        assert!(lowest(&geo.tracks[0]) > 5.59, "{:?}", geo.tracks[0].pts);
        // and it still ends on its pads
        let t = &geo.tracks[0];
        assert!(t.pts[0].dist(pt(2.0, 5.0)) < 1e-9 && t.pts.last().unwrap().dist(pt(8.0, 5.0)) < 1e-9);
    }

    #[test]
    fn the_own_tracks_go_along_with_the_pads() {
        let mut b = board();
        // P1 carries the end of the track; it moves down and to the left
        let (geo, _, reached, st) = move_with_copper(&mut b, "P1", pt(-0.5, 2.0));
        assert!(st.stopped.is_none(), "{st:?}");
        assert!(reached.dist(pt(-0.5, 2.0)) < 1e-9);
        assert!(
            geo.tracks[0].pts.last().unwrap().dist(pt(7.5, 7.0)) < 1e-9,
            "{:?}",
            geo.tracks[0].pts
        );
        assert!(geo.tracks[0].pts[0].dist(pt(2.0, 5.0)) < 1e-9);
    }

    #[test]
    fn the_part_stops_where_the_copper_cannot_give_way() {
        let mut b = board();
        // fixed copper right below the track: no room to give way
        b.obstacles.push(Obstacle {
            shape: Shape::Polygon(Polygon::new(vec![
                pt(3.0, 5.35),
                pt(7.0, 5.35),
                pt(7.0, 6.0),
                pt(3.0, 6.0),
            ])),
            layers: LayerSet::single(0),
            net: None,
            kind: ObstacleKind::Graphic,
            hole: None,
            clearance: 0.2,
            pin: None,
            locked: true,
        });
        let (geo, _, reached, st) = move_with_copper(&mut b, "Q", pt(0.0, 1.5));
        assert!(st.stopped.is_some(), "{st:?}");
        assert!(reached.y < 1.5 && reached.y > 0.3, "{reached:?}");
        let moved = b.with_offsets(&HashMap::from([("Q".to_string(), reached)]));
        assert!(check::check(&moved, &geo).is_empty());
    }

    #[test]
    fn courtyards_stop_the_part() {
        // down and left onto P0: the courtyards meet before the copper does
        let (_, _, reached, st) = move_with_copper(&mut board(), "Q", pt(-3.0, 1.2));
        assert!(st.stopped.as_deref().is_some_and(|s| s.contains("P0")), "{st:?}");
        assert!(reached.x > -3.0 && reached.x < -1.5, "{reached:?}");
    }

    #[test]
    fn a_problem_that_moves_to_other_nets_is_new_one_that_splits_is_not() {
        let spot = |x: f64, nets: (u32, Option<u32>), missing: f64| check::Spot {
            at: pt(x, 0.0),
            nets,
            missing,
            short: false,
        };
        let before = [spot(0.0, (1, Some(2)), 0.02)];
        // the same pair close by, in two pieces, no worse
        assert!(!is_new(&spot(0.5, (2, Some(1)), 0.02), &before, 0.1));
        assert!(!is_new(&spot(1.5, (1, Some(2)), 0.01), &before, 0.1));
        // worse, far away, or another pair
        assert!(is_new(&spot(0.5, (1, Some(2)), 0.03), &before, 0.1));
        assert!(is_new(&spot(5.0, (1, Some(2)), 0.01), &before, 0.1));
        assert!(is_new(&spot(0.5, (1, Some(3)), 0.01), &before, 0.1));
    }
}
