//! Finishing a net on a board with a pour of that net (a `gb:` zone): the pour is copper of the
//! net like a large pad. A track into it ends where it enters; it does not run on to a pad
//! under the pour as one straight line, across the holes of the pour and what lies in them.

use gummiband_app::{Hooks, RouteOptions, route_file};
use gummiband_core::geom::{pt, segs_intersect};

/// Four pads. GND has a pad under its pour and one outside it; the pour has a hole (its outline
/// goes in and out through a slit), and in the hole runs a track of net N from one of its pads
/// to the other. The straight line from the outer GND pad to the one under the pour crosses that
/// track.
const HOLE: &str = r#"(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (net 0 "") (net 1 "GND") (net 2 "N")
  (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (stroke (width 0.1) (type default)) (fill no))
  (footprint "R" (layer "F.Cu") (at 5 10) (property "Reference" "R1")
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "GND")))
  (footprint "R" (layer "F.Cu") (at 26 10) (property "Reference" "R2")
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "GND")))
  (footprint "R" (layer "F.Cu") (at 12 7.5) (property "Reference" "R3")
    (pad "1" smd rect (at 0 0) (size 0.4 0.4) (layers "F.Cu") (net 2 "N")))
  (footprint "R" (layer "F.Cu") (at 12 12.5) (property "Reference" "R4")
    (pad "1" smd rect (at 0 0) (size 0.4 0.4) (layers "F.Cu") (net 2 "N")))
  (segment (start 12 7.5) (end 12 12.5) (width 0.2) (layer "F.Cu") (net 2) (uuid "00000000-0000-4000-8000-000000000001"))
  (zone (net 1) (net_name "GND") (layer "F.Cu") (uuid "00000000-0000-4000-8000-000000000002") (name "gb:hole") (hatch edge 0.5) (priority 1)
    (connect_pads yes (clearance 0.2)) (min_thickness 0.2) (fill yes (thermal_gap 0.3) (thermal_bridge_width 0.3))
    (polygon (pts (xy 3 4) (xy 20 4) (xy 20 16) (xy 3 16) (xy 3 7) (xy 10 7) (xy 10 13) (xy 14 13) (xy 14 7) (xy 10 7) (xy 3 7))))
)"#;

/// Two GND pads, each under a pour of its own, and a short track of N between the pours.
const TWO: &str = r#"(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (net 0 "") (net 1 "GND") (net 2 "N")
  (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (stroke (width 0.1) (type default)) (fill no))
  (footprint "R" (layer "F.Cu") (at 5 10) (property "Reference" "R1")
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "GND")))
  (footprint "R" (layer "F.Cu") (at 26 10) (property "Reference" "R2")
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "GND")))
  (footprint "R" (layer "F.Cu") (at 15 8) (property "Reference" "R3")
    (pad "1" smd rect (at 0 0) (size 0.4 0.4) (layers "F.Cu") (net 2 "N")))
  (footprint "R" (layer "F.Cu") (at 15 12) (property "Reference" "R4")
    (pad "1" smd rect (at 0 0) (size 0.4 0.4) (layers "F.Cu") (net 2 "N")))
  (segment (start 15 8) (end 15 12) (width 0.2) (layer "F.Cu") (net 2) (uuid "00000000-0000-4000-8000-000000000001"))
  (zone (net 1) (net_name "GND") (layer "F.Cu") (uuid "00000000-0000-4000-8000-000000000002") (name "gb:left") (hatch edge 0.5) (priority 1)
    (connect_pads yes (clearance 0.2)) (min_thickness 0.2) (fill yes (thermal_gap 0.3) (thermal_bridge_width 0.3))
    (polygon (pts (xy 3 4) (xy 11 4) (xy 11 16) (xy 3 16))))
  (zone (net 1) (net_name "GND") (layer "F.Cu") (uuid "00000000-0000-4000-8000-000000000003") (name "gb:right") (hatch edge 0.5) (priority 1)
    (connect_pads yes (clearance 0.2)) (min_thickness 0.2) (fill yes (thermal_gap 0.3) (thermal_bridge_width 0.3))
    (polygon (pts (xy 19 4) (xy 28 4) (xy 28 16) (xy 19 16))))
)"#;

fn finish_gnd(name: &str, shove: bool) -> gummiband_app::RouteOutcome {
    finish(HOLE, name, shove)
}

fn finish(board: &str, name: &str, shove: bool) -> gummiband_app::RouteOutcome {
    let dir = std::env::temp_dir().join(format!("gummiband-app-pours-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let pcb = dir.join("hole.kicad_pcb");
    std::fs::write(&pcb, board).unwrap();
    std::fs::write(pcb.with_extension("kicad_pro"), "{}").unwrap();
    let opts = RouteOptions {
        runs: 4,
        generations: 1,
        geometry_generations: 1,
        keep_all: true,
        nets: vec!["GND".into()],
        shove: Some(shove),
        ..Default::default()
    };
    route_file(&pcb, &dir.join("out.kicad_pcb"), &opts, &Hooks::default()).unwrap()
}

/// The connection is routed, the last check has nothing to take out, and the new track ends in
/// the pour without touching the track in its hole.
fn assert_ends_in_the_pour(out: &gummiband_app::RouteOutcome) {
    let r = &out.routed;
    let st = &r.res.stats;
    assert_eq!((st.conns, st.routed), (1, 1), "{:?}", st.unrouted);
    assert!(
        r.legal.dropped.is_empty() && r.legal.open.is_empty() && r.legal.shorts_left == 0 && r.legal.trimmed == 0,
        "{:?}",
        r.legal
    );
    assert!(
        r.legal.dangling.is_empty() && r.legal.put_back.is_empty(),
        "{:?}",
        r.legal
    );
    assert!(r.violations.is_empty(), "{:?}", r.violations);
    let gnd = r.board.net_id("GND").unwrap();
    let new: Vec<_> = r
        .res
        .geometry
        .tracks
        .iter()
        .filter(|t| t.net == gnd && t.origin.is_none())
        .collect();
    assert_eq!(new.len(), 1, "one track from the pad outside to the pour");
    let t = new[0];
    // it does not cross the track of N in the hole ...
    let (a, b) = (pt(12.0, 7.5), pt(12.0, 12.5));
    assert!(
        t.pts.windows(2).all(|w| !segs_intersect(w[0], w[1], a, b)),
        "{:?}",
        t.pts
    );
    // ... because it ends just inside the outline on the side of its pad, not on the pad
    // under the pour
    let ends = [t.pts[0], *t.pts.last().unwrap()];
    let inside = ends.iter().find(|p| p.x < 25.0).expect("an end in the pour");
    assert!(inside.x > 19.0 && inside.x < 20.0, "{inside:?}");
    assert!(ends.iter().any(|p| p.dist(pt(26.0, 10.0)) < 0.5), "{ends:?}");
    let len: f64 = t.pts.windows(2).map(|w| w[0].dist(w[1])).sum();
    assert!(len < 8.0, "{len}");
}

#[test]
fn a_track_into_a_pour_of_its_net_ends_where_it_enters() {
    assert_ends_in_the_pour(&finish_gnd("fixed", false));
}

/// With shove the track of N is part of the topology and may give way. It has no reason to:
/// nothing moves, and nothing is put back.
#[test]
fn the_same_with_the_other_copper_giving_way() {
    let out = finish_gnd("shove", true);
    assert!(out.routed.shove);
    assert_ends_in_the_pour(&out);
    assert!(out.routed.soft.nets.is_empty(), "moved: {:?}", out.routed.soft.nets);
}

/// From one pour to another: the track is the piece between them, from where the way leaves the
/// first to where it enters the second, and it goes round the track of N that lies between.
#[test]
fn a_track_from_pour_to_pour_is_the_piece_between_them() {
    let out = finish(TWO, "two", false);
    let r = &out.routed;
    assert_eq!((r.res.stats.conns, r.res.stats.routed), (1, 1));
    assert!(r.legal.dropped.is_empty() && r.legal.open.is_empty() && r.legal.dangling.is_empty());
    assert!(r.violations.is_empty(), "{:?}", r.violations);
    let gnd = r.board.net_id("GND").unwrap();
    let new: Vec<_> = r.res.geometry.tracks.iter().filter(|t| t.net == gnd).collect();
    assert_eq!(new.len(), 1);
    let t = new[0];
    let (lo, hi) = t
        .pts
        .iter()
        .fold((f64::INFINITY, 0.0f64), |(lo, hi), p| (lo.min(p.x), hi.max(p.x)));
    assert!(lo > 10.0 && lo < 11.0 && hi > 19.0 && hi < 20.0, "{lo} {hi}");
    let (a, b) = (pt(15.0, 8.0), pt(15.0, 12.0));
    assert!(
        t.pts.windows(2).all(|w| !segs_intersect(w[0], w[1], a, b)),
        "{:?}",
        t.pts
    );
    let len: f64 = t.pts.windows(2).map(|w| w[0].dist(w[1])).sum();
    assert!(len < 12.0, "{len}");
}
