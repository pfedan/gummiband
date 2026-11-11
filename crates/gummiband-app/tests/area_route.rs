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

/// With `areas tracks` a net with a zone is laid like any net; with `areas fill` it gets no
/// copper of its own, and the report says what its fill reaches.
#[test]
fn a_net_with_an_area_is_left_to_its_fill() {
    let tracks = route("area_far", "tracks", &quick(Areas::Tracks, &[]));
    assert!(tracks_of(&tracks, "VBUS") > 0);
    assert!(tracks.routed.areas.is_empty());
    let fill = route("area_far", "fill", &quick(Areas::Fill, &[]));
    assert_eq!(tracks_of(&fill, "VBUS"), 0);
    // GND has no area and is laid as before
    assert!(tracks_of(&fill, "GND") > 0);
    let [area] = fill.routed.areas.as_slice() else {
        panic!("one area net")
    };
    assert_eq!(area.net, "VBUS");
    assert!(area.pads >= 2 && area.reached <= area.pads, "{}", area.summary);
    // the pads the fill leaves are not counted among the open connections of the tracks
    assert!(fill.routed.res.stats.unrouted.iter().all(|u| !u.starts_with("VBUS ")));
}

/// A planned pour makes its net an area net, on the outer layers where none are named.
#[test]
fn a_planned_pour_is_an_area_of_its_net() {
    let kb = read_board(&bench("area_halves"), &ReadOptions::default()).unwrap();
    let b = &kb.board;
    assert!(route_plan(&kb, &[], &[]).unwrap().plan.nets.is_empty());
    let p = route_plan(&kb, &["GND".into(), "GNDR".into()], &[]).unwrap().plan;
    let (gnd, gndr) = (b.net_id("GND").unwrap(), b.net_id("GNDR").unwrap());
    assert_eq!(p.nets, [gnd.min(gndr), gnd.max(gndr)]);
    // each net on both outer layers, and the two share the board
    for net in [gnd, gndr] {
        let mut layers: Vec<u8> = p.areas.iter().filter(|a| a.net == net).map(|a| a.layer).collect();
        layers.sort_unstable();
        layers.dedup();
        assert_eq!(layers.len(), 2);
    }
    let one = route_plan(&kb, &["GND:F.Cu".into()], &[]).unwrap().plan;
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
        let e = match route_plan(&kb, &s(pour), &s(nets)) {
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
