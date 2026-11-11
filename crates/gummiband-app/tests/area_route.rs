//! A route with `areas fill`: the nets with an area on a routed layer are left to its fill.
//! The boards are those of `bench/boards/areas`.

use gummiband_app::areas::route_plan;
use gummiband_app::{Areas, Hooks, RouteOptions, parse_pour, route_file};
use gummiband_kicad::{ReadOptions, read_board};

fn bench(board: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../bench/boards/areas")
        .join(format!("{board}.kicad_pcb"))
}

fn quick(areas: Areas, pour: &[&str]) -> RouteOptions {
    RouteOptions {
        runs: 4,
        generations: 1,
        geometry_generations: 1,
        move_parts: Some(false),
        areas,
        pour: pour.iter().map(|s| s.to_string()).collect(),
        ..Default::default()
    }
}

fn route(board: &str, name: &str, opts: &RouteOptions) -> gummiband_app::RouteOutcome {
    let dir = std::env::temp_dir().join(format!("gummiband-app-areas-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let out = route_file(&bench(board), &dir.join("out.kicad_pcb"), opts, &Hooks::default()).unwrap();
    std::fs::remove_dir_all(&dir).ok();
    out
}

fn tracks_of(out: &gummiband_app::RouteOutcome, net: &str) -> usize {
    let b = &out.routed.board;
    let id = b.net_id(net).unwrap();
    let geo = &out.routed.res.geometry;
    geo.tracks.iter().filter(|t| t.net == id).count() + geo.vias.iter().filter(|v| v.net == id).count()
}

#[test]
fn an_entry_of_pour_is_a_net_with_or_without_layers() {
    assert_eq!(parse_pour("GND"), Ok(("GND".to_string(), vec![])));
    assert_eq!(
        parse_pour("GND:F.Cu,B.Cu"),
        Ok(("GND".to_string(), vec!["F.Cu".to_string(), "B.Cu".to_string()]))
    );
    // a colon in the name of a net is not taken for layers
    assert_eq!(parse_pour("/power:GND"), Ok(("/power:GND".to_string(), vec![])));
    assert_eq!(
        parse_pour("/power:GND:In1.Cu"),
        Ok(("/power:GND".to_string(), vec!["In1.Cu".to_string()]))
    );
    assert!(parse_pour(":F.Cu").unwrap_err().contains("names no net"));
}

/// With `areas tracks` a net with a zone is laid like any net. With `areas fill` only the pads
/// its fill does not reach get copper, and only as far as the area: VBUS has its area on the
/// back and pads on the front, so a group of pads gets a piece of track to a via that lands
/// in the fill, or to a pad that hangs on the area already.
#[test]
fn a_net_with_an_area_is_left_to_its_fill() {
    let tracks = route("area_far", "tracks", &quick(Areas::Tracks, &[]));
    assert!(tracks_of(&tracks, "VBUS") > 0);
    assert!(tracks.routed.areas.is_empty());
    let fill = route("area_far", "fill", &quick(Areas::Fill, &[]));
    let (b, geo) = (&fill.routed.board, &fill.routed.res.geometry);
    let vbus = b.net_id("VBUS").unwrap();
    // GND has no area and is laid as before
    assert!(tracks_of(&fill, "GND") > 0);
    let [area] = fill.routed.areas.as_slice() else {
        panic!("one area net")
    };
    assert_eq!(area.net, "VBUS");
    assert_eq!(area.reached, area.pads, "{}", area.summary);
    assert!(fill.routed.res.stats.unrouted.is_empty());
    // its copper is vias into the area and what leads to them: less than its tracks were,
    // on a board whose pads lie as far from a spot for a via as they can
    let length = |out: &gummiband_app::RouteOutcome| -> f64 {
        let g = &out.routed.res.geometry;
        let of = g.tracks.iter().filter(|t| t.net == vbus);
        of.map(|t| t.pts.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>()).sum()
    };
    assert!(geo.vias.iter().any(|v| v.net == vbus));
    assert!(length(&fill) < length(&tracks), "{} {}", length(&fill), length(&tracks));
    // the last check takes none of it back: an end in the fill is held
    assert_eq!(fill.routed.legal.trimmed, 0);
    // what the fill joins stays in the count of connections, so that a share of them means
    // the same with tracks and with fill
    assert_eq!(fill.routed.res.stats.conns, tracks.routed.res.stats.conns);
    assert_eq!(fill.routed.res.stats.routed, fill.routed.res.stats.conns);
}

/// A pad the fill of a planned pour does not reach gets its piece of copper to the area.
#[test]
fn a_pad_the_pour_does_not_reach_is_joined_to_it() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../bench/boards");
    let run = |opts: &RouteOptions| {
        let tmp = std::env::temp_dir().join(format!(
            "gummiband-app-areas-medium-{}-{}",
            opts.areas.word(),
            std::process::id()
        ));
        std::fs::create_dir_all(&tmp).unwrap();
        let out = route_file(
            &dir.join("medium.kicad_pcb"),
            &tmp.join("out.kicad_pcb"),
            opts,
            &Hooks::default(),
        )
        .unwrap();
        std::fs::remove_dir_all(&tmp).ok();
        out
    };
    let fill = run(&quick(Areas::Fill, &["GND"]));
    let [gnd] = fill.routed.areas.as_slice() else {
        panic!("one area net")
    };
    assert_eq!(gnd.reached, gnd.pads, "{}", gnd.summary);
    assert!(gnd.islands.is_empty());
    assert!(fill.routed.res.stats.unrouted.is_empty());
    assert!(fill.routed.violations.is_empty());
}

/// Two of the pens of `area_strips` leave their pad on an island of fill with room for a via.
/// Such an island gets that via on the finished copper and no track; with `area_stitch` off
/// the pad gets a connection like any pad the fill does not reach.
#[test]
fn an_island_of_fill_gets_one_via_and_no_track() {
    let locked = |stitch: Option<bool>| RouteOptions {
        keep_locked: true,
        area_stitch: stitch,
        ..quick(Areas::Fill, &[])
    };
    let out = route("area_strips", "stitch", &locked(None));
    let [gnd] = out.routed.areas.as_slice() else {
        panic!("one area net")
    };
    assert_eq!((gnd.pads, gnd.reached), (10, 10), "{}", gnd.summary);
    assert_eq!(out.routed.area_vias, 2);
    let geo = &out.routed.res.geometry;
    let net = out.routed.board.net_id("GND").unwrap();
    assert_eq!(geo.vias.iter().filter(|v| v.net == net).count(), 2);
    assert_eq!(
        geo.tracks.iter().filter(|t| t.net == net && t.origin.is_none()).count(),
        0
    );
    assert_eq!(out.routed.res.stats.vias, 2);
    assert!(out.routed.violations.is_empty());
    // without the vias the search joins the pads itself
    let off = route("area_strips", "nostitch", &locked(Some(false)));
    assert_eq!(off.routed.area_vias, 0);
    assert_eq!(off.routed.areas[0].reached, 10, "{}", off.routed.areas[0].summary);
    assert!(tracks_of(&off, "GND") > 0);
}

/// `area_island`: two islands of fill on the front get a via each into the fill of the back,
/// and a pad without a spoke gets a piece of track into the fill of the front. Between the
/// islands lie strips of fill that hold nothing and that only the fine raster of the last
/// check sees. That check must still know which cells are fill of the area: it once took the
/// track back as a stub and the via of an island with it, and counted both pads open.
#[test]
fn the_last_check_keeps_what_ends_in_the_fill_where_pieces_of_fill_are_left_out() {
    let opts = RouteOptions {
        keep_locked: true,
        ..quick(Areas::Fill, &[])
    };
    let out = route("area_island", "island", &opts);
    let [gnd] = out.routed.areas.as_slice() else {
        panic!("one area net")
    };
    assert_eq!((gnd.pads, gnd.reached), (6, 6), "{}", gnd.summary);
    assert_eq!(out.routed.area_vias, 2);
    assert_eq!(out.routed.legal.trimmed, 0);
    assert!(
        out.routed.res.stats.unrouted.is_empty(),
        "{:?}",
        out.routed.res.stats.unrouted
    );
    assert_eq!(out.routed.res.stats.routed, out.routed.res.stats.conns);
    let geo = &out.routed.res.geometry;
    let net = out.routed.board.net_id("GND").unwrap();
    assert_eq!(geo.vias.iter().filter(|v| v.net == net).count(), 2);
    // the piece of track of the pad without a spoke
    assert!(geo.tracks.iter().any(|t| t.net == net && t.origin.is_none()));
    assert!(out.routed.violations.is_empty());
}

/// A planned pour makes its net an area net, on the outer layers where none are named.
#[test]
fn a_planned_pour_is_an_area_of_its_net() {
    let kb = read_board(&bench("area_halves"), &ReadOptions::default()).unwrap();
    let b = &kb.board;
    assert!(route_plan(&kb, &[], &[], true).unwrap().plan.nets.is_empty());
    let p = route_plan(&kb, &["GND".into(), "GNDR".into()], &[], true).unwrap().plan;
    let (gnd, gndr) = (b.net_id("GND").unwrap(), b.net_id("GNDR").unwrap());
    assert_eq!(p.nets, [gnd.min(gndr), gnd.max(gndr)]);
    // each net on both outer layers, and the two share the board
    for net in [gnd, gndr] {
        let mut layers: Vec<u8> = p.areas.iter().filter(|a| a.net == net).map(|a| a.layer).collect();
        layers.sort_unstable();
        layers.dedup();
        assert_eq!(layers.len(), 2);
    }
    let one = route_plan(&kb, &["GND:F.Cu".into()], &[], true).unwrap().plan;
    assert!(
        one.areas
            .iter()
            .all(|a| a.net == gnd && b.layers[a.layer as usize].name == "F.Cu")
    );
}

#[test]
fn a_plan_that_cannot_work_is_refused_in_plain_words() {
    let refused = |board: &str, pour: &[&str], nets: &[&str], word: &str| {
        let kb = read_board(&bench(board), &ReadOptions::default()).unwrap();
        let s = |v: &[&str]| v.iter().map(|x| x.to_string()).collect::<Vec<_>>();
        let e = match route_plan(&kb, &s(pour), &s(nets), true) {
            Ok(_) => panic!("{word}"),
            Err(e) => e.to_string(),
        };
        assert!(e.contains(word), "{e}");
    };
    refused("area_halves", &["NOPE"], &[], "no net named");
    refused("area_halves", &["GND:Top.Cu"], &[], "no copper layer");
    refused("area_halves", &["GND:F.Cu", "GNDR"], &[], "same layers");
    // the zone of the file is the area already
    refused("area_far", &["VBUS"], &[], "already has a zone");
    // nothing would join the pads the fill leaves
    refused("area_far", &[], &["S1"], "nets leaves it out");
}
