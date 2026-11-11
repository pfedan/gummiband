//! What the fill model says about the bench boards for areas (`bench/boards/areas`, written by
//! `bench/gen_boards.py`). The boards are fixtures: each holds one situation of a pour or a
//! plane, and these tests pin what the model finds there. Whether KiCad's fill agrees is
//! measured with `bench/areas-kicad.sh` where KiCad is installed.

use gummiband_app::areas::model;
use gummiband_app::{AreaOptions, AreaReport, area_report};
use gummiband_kicad::{KeepCopper, ReadOptions, read_board};

fn bench(board: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../bench/boards/areas")
        .join(format!("{board}.kicad_pcb"))
}

fn report(board: &str, o: &AreaOptions) -> AreaReport {
    area_report(&bench(board), o).unwrap()
}

fn names(v: &[&str]) -> Vec<String> {
    v.iter().map(|s| s.to_string()).collect()
}

/// Six pens open through strips of fill 0.10, 0.15, 0.20, 0.25, 0.30 and 0.40 mm wide, the
/// minimum width of the pour is 0.2 mm: the fill passes a strip that is wider and not one that
/// is narrower, and the strip of just the minimum width is the doubtful one.
#[test]
fn the_fill_passes_a_strip_wider_than_the_minimum_width_and_no_narrower_one() {
    let r = report("area_strips", &AreaOptions::default());
    let gnd = &r.nets[0];
    assert_eq!(
        (gnd.net.as_str(), gnd.pads, gnd.reached),
        ("GND", 10, 7),
        "{}",
        gnd.summary
    );
    let unreached: Vec<&str> = gnd.unreached.iter().map(|u| u.pad.as_str()).collect();
    assert_eq!(unreached, ["TP1.1", "TP2.1", "TP3.1"]);
    assert!(gnd.unreached.iter().all(|u| u.reason == "island"));
    // every closed pen is an island with room for one via to the back
    assert_eq!(gnd.islands.len(), 3);
    for i in &gnd.islands {
        let via = i.via.as_ref().expect("a via site in the pen");
        assert_eq!((via.from.as_str(), via.to.as_str()), ("F.Cu", "B.Cu"));
    }
    // 0.10 and 0.15 are shut and 0.25 and wider are open at any reading of the raster
    let doubtful: Vec<&str> = gnd.doubtful.as_ref().unwrap().iter().map(|d| d.pad.as_str()).collect();
    assert_eq!(doubtful, ["TP3.1"]);
}

/// Two ground nets on two halves and no zone in the file: the pour of both shares the board
/// between them, each area reaches all pads of its net and the two never overlap.
#[test]
fn a_pour_of_two_nets_on_two_halves_reaches_both_without_overlap() {
    let o = AreaOptions {
        nets: names(&["GND", "GNDR"]),
        pour: names(&["F.Cu", "B.Cu"]),
        ..Default::default()
    };
    let r = report("area_halves", &o);
    for n in &r.nets {
        assert_eq!((n.pads, n.reached), (6, 6), "{}", n.summary);
    }
    let kb = read_board(
        &bench("area_halves"),
        &ReadOptions {
            keep: KeepCopper::All,
            ..Default::default()
        },
    )
    .unwrap();
    let m = model(&kb, &o).unwrap();
    assert!(m.alone.is_empty());
    assert_eq!(m.poured.len(), 4, "an area per net and layer");
    let net = |name: &str| kb.board.nets.iter().position(|n| n.name == name).unwrap() as u32;
    let (gnd, gndr) = (net("GND"), net("GNDR"));
    // which of the two nets has an area at a spot of the front layer
    let at = |x: f64, y: f64| {
        let mut found = Vec::new();
        for a in m.poured.iter().map(|&i| &m.areas[i]) {
            let p = gummiband_core::geom::pt(x, y);
            if a.layer == 0 && a.rings.iter().filter(|r| r.contains(p)).count() % 2 == 1 {
                found.push(u32::from(a.net));
            }
        }
        found
    };
    // the board is 50 x 30 mm at 100, 100: no spot lies in two areas, and all but the corridor
    // between the two nets and the rim lies in one
    let (mut both, mut one, mut none) = (0, 0, 0);
    for i in 0..200 {
        for k in 0..120 {
            match at(100.125 + 0.25 * i as f64, 100.125 + 0.25 * k as f64).len() {
                0 => none += 1,
                1 => one += 1,
                _ => both += 1,
            }
        }
    }
    assert_eq!(both, 0);
    assert!(none * 20 < one, "{none} spots in no area, {one} in one");
    // GND has the left of the board and GNDR the right
    assert_eq!(at(105.0, 115.0), [gnd]);
    assert_eq!(at(145.0, 115.0), [gndr]);
}

/// The area of VBUS lies at the right on the back; most of its pads are far to the left. The
/// report names the way from a pad to the nearest spot where a via lands in the area, and that
/// way is not cut short by the edge of the board.
#[test]
fn the_way_from_a_pad_to_an_area_far_away_is_measured_in_full() {
    let r = report("area_far", &AreaOptions::default());
    let vbus = &r.nets[0];
    assert_eq!(
        (vbus.net.as_str(), vbus.pads, vbus.reached),
        ("VBUS", 8, 2),
        "{}",
        vbus.summary
    );
    let far = vbus.site_distance.as_ref().unwrap();
    // J1.1 is 3 mm from the left edge of the board, the area begins 31 mm from it
    assert_eq!(far.farthest, "J1.1");
    assert!(far.max > 27.0 && far.max < 29.0, "{}", far.max);
    let of = |pad: &str| vbus.unreached.iter().find(|u| u.pad == pad).unwrap();
    assert_eq!(of("J1.1").reason, "outside");
    assert_eq!(of("U1.8").reason, "other_layer");
    assert!(of("U1.8").site_distance.unwrap() > 13.0);
    // the pad above the area on the front is a via away
    assert!(of("C3.1").site_distance.unwrap() < 1.5);
}

/// A plane layer split between two nets: each half reaches the through-hole pins in it, and a
/// pin of the net in the other half lies outside its outline.
#[test]
fn a_split_plane_holds_the_pins_in_its_own_half_only() {
    let r = report("area_split", &AreaOptions::default());
    let net = |name: &str| r.nets.iter().find(|n| n.net == name).unwrap();
    for (name, outside) in [("+3V3", "J4.2"), ("+5V", "J3.1")] {
        let n = net(name);
        assert_eq!((n.pads, n.reached, n.joins), (6, 2, 2), "{}", n.summary);
        assert_eq!(n.layers.len(), 1);
        let out: Vec<&str> = n
            .unreached
            .iter()
            .filter(|u| u.reason == "outside")
            .map(|u| u.pad.as_str())
            .collect();
        assert_eq!(out, [outside]);
        // the pin over the other half is the one with the longest way to a via site
        assert!(n.site_distance.as_ref().unwrap().max > 15.0);
    }
    // the two halves are the same size and do not touch
    assert!((net("+3V3").area - net("+5V").area).abs() < 1.0);
    assert_eq!(net("GND").reached, 4);
}
