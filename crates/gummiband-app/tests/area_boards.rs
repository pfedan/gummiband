//! What the fill model says about the bench boards for areas (`bench/boards/areas`, written by
//! `bench/gen_boards.py`). The boards are fixtures: each holds one situation of a pour or a
//! plane, and these tests pin what the model finds there. Whether KiCad's fill agrees is
//! measured with `bench/areas-kicad.sh` where KiCad is installed.
//!
//! The boards of `bench/boards/areas/kicad` are routed bench boards with a GND pour, kept as
//! KiCad 10 got them when it was held against the model: at each of them the two disagreed.
//! Their tests pin what KiCad's fill does there.

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
/// minimum width of the pour is 0.2 mm: the fill passes a strip of the minimum width and wider
/// and not one that is narrower. That is KiCad's answer (3 pieces of fill on the front, 2
/// unconnected); the strip of just the minimum width stays the doubtful one, half a cell
/// decides it.
#[test]
fn the_fill_passes_a_strip_of_the_minimum_width_and_no_narrower_one() {
    let r = report("area_strips", &AreaOptions::default());
    let gnd = &r.nets[0];
    assert_eq!(
        (gnd.net.as_str(), gnd.pads, gnd.reached),
        ("GND", 10, 8),
        "{}",
        gnd.summary
    );
    let unreached: Vec<&str> = gnd.unreached.iter().map(|u| u.pad.as_str()).collect();
    assert_eq!(unreached, ["TP1.1", "TP2.1"]);
    assert!(gnd.unreached.iter().all(|u| u.reason == "island"));
    assert_eq!(gnd.expected_unconnected, 2);
    assert_eq!((gnd.layers[0].layer.as_str(), gnd.layers[0].pieces), ("F.Cu", 3));
    // every closed pen is an island with room for one via to the back
    assert_eq!(gnd.islands.len(), 2);
    for i in &gnd.islands {
        let via = i.via.as_ref().expect("a via site in the pen");
        assert_eq!((via.from.as_str(), via.to.as_str()), ("F.Cu", "B.Cu"));
    }
    // 0.10 and 0.15 are shut and 0.25 and wider are open at any reading of the raster
    let doubtful: Vec<&str> = gnd.doubtful.as_ref().unwrap().iter().map(|d| d.pad.as_str()).collect();
    assert_eq!(doubtful, ["TP3.1"]);
    assert!(gnd.doubtful.as_ref().unwrap()[0].reached);
}

/// From the top of the board down: a region and a band of fill with one GND pad each, cut off
/// by locked tracks from the rest (two islands, each with room for a via to the back), between
/// them strips of fill that hold nothing, and a pad whose spokes are all cut by a pen of
/// track.
#[test]
fn two_islands_with_a_via_site_each_and_a_pad_without_a_spoke() {
    let r = report("area_island", &AreaOptions::default());
    let gnd = &r.nets[0];
    assert_eq!((gnd.pads, gnd.reached), (6, 3), "{}", gnd.summary);
    let unreached: Vec<(&str, &str)> = gnd.unreached.iter().map(|u| (u.pad.as_str(), u.reason)).collect();
    assert_eq!(
        unreached,
        [("TP1.1", "island"), ("TP2.1", "island"), ("G3.1", "no_spoke")]
    );
    for i in &gnd.islands {
        let via = i.via.as_ref().expect("a via site on the island");
        assert_eq!((via.from.as_str(), via.to.as_str()), ("F.Cu", "B.Cu"));
        assert!(via.second.is_none());
    }
    // the strips are pieces of their own that hold nothing, and are left out
    let front = &gnd.layers[0];
    assert_eq!((front.of_the_area, front.floating, front.isolated), (1, 4, 0));
    assert!(gnd.doubtful.as_ref().unwrap().is_empty());
}

/// The report of GND on a board of `bench/boards/areas/kicad`.
fn as_kicad_got_it(board: &str) -> gummiband_app::areas::NetReport {
    let r = report(&format!("kicad/{board}"), &AreaOptions::default());
    let [gnd] = r.nets.as_slice() else {
        panic!("one net with an area")
    };
    assert_eq!(gnd.net, "GND");
    gnd.clone()
}

/// `fine_pitch_mixed`, seed 2: six resistor pads in a row at a pitch of 1.6 mm, whose thermal
/// gaps meet, so that no spoke between two of them ends in fill. KiCad joins all six all the
/// same: a spoke also stays where it and a spoke of another pad reach each other's ends. The
/// one pad KiCad leaves unconnected is the exposed pad of U1.
#[test]
fn pads_in_a_row_whose_reliefs_meet_are_joined_by_their_spokes() {
    let gnd = as_kicad_got_it("fine_pitch_mixed.s2");
    let unreached: Vec<&str> = gnd.unreached.iter().map(|u| u.pad.as_str()).collect();
    assert_eq!(unreached, ["U1.57"], "{}", gnd.summary);
    assert_eq!(gnd.expected_unconnected, 1);
    let doubtful = gnd.doubtful.as_ref().unwrap();
    for pad in ["R50.2", "R51.2", "R52.2", "R53.2", "R54.2", "R56.2"] {
        assert!(doubtful.iter().all(|d| d.pad != pad), "{pad} is doubtful");
    }
}

/// `fine_pitch_mixed`, seed 3: the exposed pad of U1 hangs by its spokes on a piece of fill of
/// one square millimetre between the tracks that leave the chip. KiCad has that piece (two
/// pieces of fill on the front: the area and this one) and counts the pad as unconnected. On
/// the back nothing of GND holds the pour: KiCad keeps all its pieces as isolated copper.
#[test]
fn a_pad_whose_spokes_end_in_a_small_piece_of_fill_is_on_an_island() {
    let gnd = as_kicad_got_it("fine_pitch_mixed.s3");
    let [u] = gnd.unreached.as_slice() else {
        panic!("{}", gnd.summary)
    };
    assert_eq!((u.pad.as_str(), u.reason), ("U1.57", "island"));
    assert_eq!(gnd.expected_unconnected, 1);
    let [island] = gnd.islands.as_slice() else {
        panic!("one island")
    };
    assert!(
        island.via.is_none() && island.filled > 0.5 && island.filled < 1.5,
        "{}",
        island.filled
    );
    let (front, back) = (&gnd.layers[0], &gnd.layers[1]);
    assert_eq!((front.layer.as_str(), back.layer.as_str()), ("F.Cu", "B.Cu"));
    // what stays of the front: the area and the island
    assert_eq!(front.of_the_area, 1);
    assert_eq!(front.pieces - front.floating + front.isolated, 2);
    assert_eq!((back.of_the_area, back.isolated), (0, back.pieces));
    assert!(
        back.pieces > 30 && back.filled > 1390.0 && back.filled < 1420.0,
        "{}",
        back.filled
    );
}

/// `parts_medium`, seed 1: every GND pad is on the front and nothing joins the pour of the
/// back to the net. KiCad keeps that copper all the same (one piece of 1760 mm2) and warns of
/// isolated copper: it takes no piece out of a zone in which every piece holds nothing. U2.4
/// is reached in the model and not in KiCad, and the model names it as doubtful.
#[test]
fn a_pour_that_nothing_holds_stays_as_isolated_copper() {
    let gnd = as_kicad_got_it("parts_medium.s1");
    let back = &gnd.layers[1];
    assert_eq!(back.layer, "B.Cu");
    assert_eq!(
        (back.pieces, back.of_the_area, back.floating, back.isolated),
        (1, 0, 1, 1)
    );
    assert!(back.filled > 1750.0 && back.filled < 1770.0, "{}", back.filled);
    // on the front the pieces that hold nothing are left out: other pieces hold pads there
    assert!(gnd.layers[0].floating > 0 && gnd.layers[0].isolated == 0);
    let doubtful: Vec<(&str, bool)> = gnd
        .doubtful
        .as_ref()
        .unwrap()
        .iter()
        .map(|d| (d.pad.as_str(), d.reached))
        .collect();
    assert_eq!(doubtful, [("U2.4", true)]);
}

/// `matching`, seed 3: J1.16, the last pad of a connector with a pitch of 0.5 mm. Its spoke
/// away from the row ends 0.05 mm from the clearance of a track, in a pocket of fill between
/// two tracks. KiCad joins the pad. The model does not: with the clearances as they are the
/// end of the spoke lies one cell outside the fill, with half a cell less the spoke stays and
/// the pocket is an island. That is within the raster, and the model says so: the pad is
/// doubtful.
#[test]
fn a_pad_whose_spoke_ends_within_a_cell_of_the_fill_is_doubtful() {
    let gnd = as_kicad_got_it("matching.s3");
    let unreached: Vec<&str> = gnd.unreached.iter().map(|u| u.pad.as_str()).collect();
    assert_eq!(unreached, ["J1.16"], "{}", gnd.summary);
    let [d] = gnd.doubtful.as_ref().unwrap().as_slice() else {
        panic!("one doubtful pad")
    };
    assert_eq!((d.pad.as_str(), d.reached, d.spoke), ("J1.16", false, true));
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
