//! A track leaves a pad in a tight row by a fixed straight piece on the middle line of the pad
//! (`gummiband_core::exit`). In a routed result that piece is a track of its own: one line from
//! the pin to the exit point, and the board is the board of the file again.

use gummiband_app::{Hooks, RouteOptions, route_file};
use gummiband_core::exit;
use gummiband_core::realize::Elem;

fn quick(board: &str) -> gummiband_app::RouteOutcome {
    let opts = RouteOptions {
        runs: 4,
        generations: 1,
        geometry_generations: 0,
        move_parts: Some(false),
        ..Default::default()
    };
    let input = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../bench/boards")
        .join(board);
    let output = std::env::temp_dir().join(format!("gummiband-app-exits-{}-{board}", std::process::id()));
    route_file(&input, &output, &opts, &Hooks::default()).unwrap()
}

/// `bus` has two parts with pads at 0.5 mm pitch: 0.1 mm to spare beside a track on the middle
/// line, less than its clearance, so every pad of them gets an exit.
#[test]
fn the_tracks_of_a_tight_row_leave_by_one_line_on_the_middle_line() {
    let out = quick("bus.kicad_pcb");
    let r = &out.routed;
    let (b, geo) = (&r.board, &r.res.geometry);
    assert_eq!(r.res.stats.routed, r.res.stats.conns);
    // the result is plain copper on the pads of the file
    assert!(b.exits.is_empty() && b.bare_pads.is_empty());
    assert!(r.tight_pads().is_empty(), "the footprints of the board have room");
    // the pads the rule covers, with the exits it lays (the same on this board as when it
    // was routed: nothing of the route is an obstacle)
    let plans = exit::plan(b);
    assert!(plans.len() >= 30, "{} pads", plans.len());
    let mut pieces = 0;
    for p in &plans {
        let pin = &b.pins[p.pin];
        let side = pin.pos + p.frame.u.perp() * (p.frame.half + 0.05);
        let at_pin = |q: gummiband_core::geom::Point| q.dist(pin.pos) < 1e-9;
        for t in geo.tracks.iter().filter(|t| t.net == pin.net && t.pts.len() >= 2) {
            let (first, last) = (t.pts[0], t.pts[t.pts.len() - 1]);
            if !at_pin(first) && !at_pin(last) {
                // no other track of the net comes beside the pad: they begin at the exit point
                let near = t
                    .pts
                    .windows(2)
                    .map(|w| gummiband_core::geom::point_seg_dist(side, w[0], w[1]))
                    .fold(f64::INFINITY, f64::min);
                assert!(near > 0.05, "{}: a track beside the pad", pin.name);
                continue;
            }
            let far = if at_pin(first) { last } else { first };
            let e = p
                .exits
                .iter()
                .find(|e| e.at.dist(far) < 1e-9)
                .unwrap_or_else(|| panic!("{}: the track from the pin ends at {far:?}", pin.name));
            assert!(matches!(t.elems[..], [Elem::Line(..)]), "{}: one line", pin.name);
            assert!(
                (far - pin.pos).cross(e.dir).abs() < 1e-9,
                "{}: on the middle line",
                pin.name
            );
            assert!(t.width == e.width && pin.pos.dist(far) > 0.3);
            pieces += 1;
        }
    }
    assert!(pieces >= 20, "{pieces} exit pieces");
    // and nothing of it troubles the check
    assert!(r.violations.is_empty(), "{:?}", r.violations);
}
