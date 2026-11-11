//! No routed result holds a short between two nets, and what counts as routed is connected in
//! the copper. Two bench boards pin the flaws that used to end as shorts at quick settings, and
//! they must be fixed where they arise: the last check on the finished copper
//! (`gummiband_core::legal`) may not have to take anything out for them.

use gummiband_app::{Hooks, RouteOptions, route_file};

/// A fresh folder for the outputs of one test.
fn out_dir(name: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("gummiband-app-legal-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn bench(board: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../bench/boards")
        .join(board)
}

fn quick(board: &str) -> gummiband_app::RouteOutcome {
    let opts = RouteOptions {
        runs: Some(4),
        generations: Some(1),
        geometry_generations: Some(0),
        move_parts: Some(false),
        ..Default::default()
    };
    // nothing is written: the output only has to differ from the input
    let output = std::env::temp_dir().join(format!("gummiband-app-legal-{}-{board}", std::process::id()));
    route_file(&bench(board), &output, &opts, &Hooks::default()).unwrap()
}

/// The pairs of nets that touch in a written board file, by name.
fn shorts_in_file(path: &std::path::Path) -> Vec<(String, String)> {
    use gummiband_kicad::{KeepCopper, ReadOptions, read_board};
    let mut b = read_board(
        path,
        &ReadOptions {
            keep: KeepCopper::All,
            ..Default::default()
        },
    )
    .unwrap()
    .board;
    b.soften(&[]);
    let geo = gummiband_core::follow::soft_geometry(&b);
    gummiband_core::legal::shorts(&b, &geo)
        .into_iter()
        .map(|(x, y)| (b.nets[x as usize].name.clone(), b.nets[y as usize].name.clone()))
        .collect()
}

/// `geo` with a stub of one net laid onto a track of another net: a short that no search made.
fn with_a_short(geo: &gummiband_core::realize::Geometry) -> gummiband_core::realize::Geometry {
    use gummiband_core::geom::pt;
    use gummiband_core::realize::Elem;
    let a = geo.tracks.iter().find(|t| t.pts.len() >= 2).expect("a track");
    let b = geo
        .tracks
        .iter()
        .find(|t| t.net != a.net && t.layer == a.layer && t.pts.len() >= 2)
        .expect("a track of another net on the same layer");
    let at = b.pts[0].lerp(b.pts[1], 0.5);
    let end = pt(at.x + 0.05, at.y);
    let mut stub = a.clone();
    stub.pts = vec![at, end];
    stub.elems = vec![Elem::Line(at, end)];
    stub.necks.clear();
    stub.fitted.clear();
    (stub.origin, stub.resting, stub.pattern, stub.seg, stub.tap) = (None, false, None, u32::MAX, None);
    let mut out = geo.clone();
    out.tracks.push(stub);
    out
}

fn assert_legal(out: &gummiband_app::RouteOutcome) {
    let r = &out.routed;
    let st = &r.res.stats;
    assert_eq!(st.routed, st.conns, "open: {:?}", st.unrouted);
    assert!(
        r.legal.dropped.is_empty()
            && r.legal.open.is_empty()
            && r.legal.shorts_left == 0
            && r.legal.trimmed == 0
            && r.legal.dangling.is_empty(),
        "the last check had to step in: {:?}",
        r.legal
    );
    let shorts: Vec<&str> = r
        .violations
        .iter()
        .filter(|v| v.actual <= 0.0 && !v.is_width())
        .map(|v| v.what.as_str())
        .collect();
    assert!(shorts.is_empty(), "shorts: {shorts:?}");
}

/// B3_1 and B3_2 run side by side over long mesh edges with slack. The band of one took the
/// radius of a far portal at a pivot, swung wide and crossed the other twice (realize.rs,
/// `seg_path`): a band has to keep to its portals.
#[test]
fn a_band_keeps_to_its_portals() {
    assert_legal(&quick("bga_escape.kicad_pcb"));
}

/// One connection dives under a track through two new vias next to each other. The second via
/// split the triangle the first one had split, the track between them was threaded anew, and
/// the via ended up on the wrong side of it (GND and D1 crossed); a way that comes back over an
/// edge it crossed before the vias took the wrong slot there (+3V3 and USB_P). Both are caught
/// when the way is committed (route.rs, `commit`).
#[test]
fn two_vias_of_one_way_keep_the_topology_planar() {
    assert_legal(&quick("parts_hard.kicad_pcb"));
}

/// A track that runs over a pad of its own net crosses no mesh edge there. The check of the
/// router's state (`Router::validate`) took that for a broken chain of crossings and cried wolf
/// in half of all runs. What it reports now is real: chords that cross, or an edge that holds
/// more than fits.
#[test]
fn the_check_of_the_topology_does_not_cry_wolf() {
    use gummiband_core::route::{Router, RouterConfig};
    use gummiband_core::topo::{Substrate, SubstrateConfig};
    let pcb = bench("qfn_fanout.kicad_pcb");
    let opts = RouteOptions {
        move_parts: Some(false),
        ..Default::default()
    };
    let kb = gummiband_app::load_board(&pcb, &opts, &Hooks::default()).unwrap();
    let sub = Substrate::build(&kb.board, &SubstrateConfig::default());
    let mut r = Router::new(&kb.board, &sub, RouterConfig::default());
    r.plan();
    r.run();
    assert!(r.unrouted().is_empty());
    let found = r.validate();
    assert!(found.iter().all(|l| l.contains("over capacity")), "{found:?}");
}

/// The open end goes the same way as a plain run: its result and every alternative it leaves
/// pass the last check before they are written, and the last check finds nothing more to do on
/// them. And the writer looks once more at whatever it is handed: with a short put into the
/// result and into an alternative by hand, no written file holds it.
#[test]
fn an_open_end_result_and_its_alternatives_are_written_without_a_short() {
    use gummiband_core::legal;
    let dir = out_dir("open");
    let opts = RouteOptions {
        runs: Some(2),
        generations: Some(1),
        geometry_generations: Some(1),
        move_parts: Some(false),
        seed: Some(1),
        open_end_seconds: Some(3.0),
        ..Default::default()
    };
    assert!(opts.open_end(false));
    let output = dir.join("open.kicad_pcb");
    let mut out = route_file(&bench("medium.kicad_pcb"), &output, &opts, &Hooks::default()).unwrap();
    let r = &out.routed;
    assert_eq!(r.legal_alternatives.len(), r.res.alternatives.len());
    let results =
        std::iter::once((&r.res.stats, &r.res.geometry)).chain(r.res.alternatives.iter().map(|(s, g)| (s, g)));
    for (stats, geometry) in results {
        assert!(legal::shorts(&r.board, geometry).is_empty());
        let (mut again, mut counted) = (geometry.clone(), stats.clone());
        let found = legal::enforce(&r.board, &mut again, &mut counted);
        assert!(found.dropped.is_empty() && found.open.is_empty(), "{found:?}");
        assert_eq!(
            (counted.routed, again.tracks.len()),
            (stats.routed, geometry.tracks.len())
        );
    }
    // a short in the result and in an alternative, as if a step had slipped past the last check
    let bad = with_a_short(&out.routed.res.geometry);
    assert_eq!(legal::shorts(&out.routed.board, &bad).len(), 1);
    out.routed.res.geometry = bad.clone();
    if out.routed.res.alternatives.is_empty() {
        let stats = out.routed.res.stats.clone();
        out.routed.res.alternatives.push((stats, bad));
    } else {
        for (_, geometry) in &mut out.routed.res.alternatives {
            *geometry = with_a_short(geometry);
        }
    }
    let written = out.write().unwrap();
    assert!(!written.alternatives.is_empty());
    for path in std::iter::once(&written.output).chain(written.alternatives.iter().map(|a| &a.path)) {
        assert_eq!(shorts_in_file(path), Vec::new(), "{}", path.display());
    }
    let _ = std::fs::remove_dir_all(&dir);
}
