//! The shared route pipeline on a board of two resistors: defaults, a route from file to file,
//! and finishing a net on the result.

use gummiband_app::{Event, Hooks, RouteOptions, defaults, route_file};

fn pad(num: u32, x: f64, net: &str) -> String {
    format!(
        "(pad \"{num}\" smd roundrect (at {x} 0) (size 0.8 0.95) (layers \"F.Cu\" \"F.Mask\" \"F.Paste\") \
         (roundrect_rratio 0.25) (net \"{net}\"))"
    )
}

fn resistor(refdes: &str, x: f64, y: f64, n1: &str, n2: &str) -> String {
    format!(
        "(footprint \"Test:R_0603\" (layer \"F.Cu\") (at {x} {y}) \
         (property \"Reference\" \"{refdes}\" (at 0 -1.13) (layer \"F.Fab\")) (attr smd) {} {})",
        pad(1, -0.825, n1),
        pad(2, 0.825, n2)
    )
}

/// Two resistors, nets A and B between them, 20 x 15 mm outline.
fn tiny_board() -> String {
    format!(
        "(kicad_pcb (version 20260206) (generator \"gummiband_test\") (generator_version \"10.0\")\n\
         (general (thickness 1.6)) (paper \"A4\")\n\
         (layers (0 \"F.Cu\" signal) (2 \"B.Cu\" signal) (1 \"F.Mask\" user) (3 \"B.Mask\" user) \
         (13 \"F.Paste\" user) (15 \"B.Paste\" user) (25 \"Edge.Cuts\" user) (35 \"F.Fab\" user))\n\
         (setup (pad_to_mask_clearance 0))\n{}\n{}\n\
         (gr_rect (start 100 100) (end 120 115) (stroke (width 0.1) (type default)) (fill no) (layer \"Edge.Cuts\"))\n)\n",
        resistor("R1", 105.0, 105.0, "A", "B"),
        resistor("R2", 113.0, 110.0, "A", "B"),
    )
}

#[test]
fn options_follow_from_each_other() {
    let d = RouteOptions::default();
    assert_eq!(d.runs, defaults::RUNS);
    assert!(d.tune && d.move_parts() && !d.shove() && !d.finishing());
    assert_eq!(d.give(), 0.0);
    assert_eq!((d.air(false), d.air(true)), (0.0, defaults::OPEN_END_AIR));

    let finishing = RouteOptions {
        keep_all: true,
        nets: vec!["A".into()],
        ..Default::default()
    };
    assert!(finishing.finishing() && finishing.shove() && !finishing.move_parts());
    assert_eq!(finishing.give(), defaults::GIVE);
    assert_eq!(finishing.air(true), 0.0);
    // parts that move leave the other tracks as obstacles; what is asked for counts
    let moving = RouteOptions {
        move_parts: Some(true),
        ..finishing.clone()
    };
    assert!(!moving.shove());
    let fixed = RouteOptions {
        shove: Some(false),
        give: Some(2.0),
        air: Some(0.01),
        ..finishing
    };
    assert!(!fixed.shove());
    assert_eq!((fixed.give(), fixed.air(true)), (2.0, 0.01));
}

/// Options that contradict each other are refused with a message that says what to change;
/// options that were left out never contradict.
#[test]
fn contradictions_are_errors() {
    let d = RouteOptions::default();
    assert_eq!(d.validate(false), Ok(()));
    assert_eq!(d.validate(true), Ok(()));
    let finishing = RouteOptions {
        keep_all: true,
        nets: vec!["A".into()],
        ..Default::default()
    };
    assert_eq!(finishing.validate(true), Ok(()));
    let refused = |o: RouteOptions, open_end: bool, word: &str| {
        let e = o.validate(open_end).expect_err(word);
        assert!(e.contains(word), "{e}");
    };
    refused(RouteOptions { runs: 0, ..d.clone() }, false, "runs must be at least 1");
    refused(
        RouteOptions {
            pairs: true,
            ..d.clone()
        },
        true,
        "pairs",
    );
    assert_eq!(
        RouteOptions {
            pairs: true,
            ..d.clone()
        }
        .validate(false),
        Ok(())
    );
    refused(
        RouteOptions {
            reroute: true,
            ..d.clone()
        },
        false,
        "reroute needs keep_all and nets",
    );
    refused(
        RouteOptions {
            shove: Some(true),
            ..d.clone()
        },
        false,
        "shove needs keep_all and nets",
    );
    refused(
        RouteOptions {
            give: Some(4.0),
            ..d.clone()
        },
        false,
        "give needs keep_all and nets",
    );
    // switching off what is off anyway is no contradiction
    let off = RouteOptions {
        shove: Some(false),
        give: Some(0.0),
        ..d.clone()
    };
    assert_eq!(off.validate(false), Ok(()));
    // parts that move and tracks that shift: one of them has to be asked off
    let moving = RouteOptions {
        move_parts: Some(true),
        ..finishing.clone()
    };
    refused(moving.clone(), false, "switch shove off, or leave move_parts out");
    refused(
        RouteOptions {
            shove: Some(true),
            ..moving.clone()
        },
        false,
        "move_parts does not work with shove",
    );
    assert_eq!(
        RouteOptions {
            shove: Some(false),
            ..moving
        }
        .validate(false),
        Ok(())
    );
    refused(
        RouteOptions {
            pair_all: true,
            ..d.clone()
        },
        false,
        "pair_all needs pairs",
    );
    refused(
        RouteOptions {
            pair_zdiff: Some(90.0),
            ..d.clone()
        },
        false,
        "pair_zdiff needs pairs",
    );
    refused(
        RouteOptions {
            placements: 2,
            ..finishing.clone()
        },
        false,
        "placements routes the whole board",
    );
    refused(
        RouteOptions {
            clear_corridors: true,
            ..finishing
        },
        false,
        "clear_corridors",
    );
}

#[test]
fn route_a_file_then_finish_a_net() {
    let dir = std::env::temp_dir().join(format!("gummiband-app-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let pcb = dir.join("tiny.kicad_pcb");
    std::fs::write(&pcb, tiny_board()).unwrap();

    // net A alone, parts fixed
    let opts = RouteOptions {
        runs: 1,
        generations: 0,
        geometry_generations: 0,
        nets: vec!["A".into()],
        move_parts: Some(false),
        ..Default::default()
    };
    let movable = std::cell::Cell::new(None);
    let event = |e: Event<'_>| {
        if let Event::MovableParts { count } = e {
            movable.set(Some(count));
        }
    };
    let hooks = Hooks {
        event: Some(&event),
        ..Default::default()
    };
    let out = route_file(&pcb, &opts, &hooks).unwrap();
    assert_eq!(movable.get(), None, "parts are fixed");
    let st = &out.routed.res.stats;
    assert_eq!((st.conns, st.routed), (1, 1));
    assert!(out.routed.violations.is_empty() && out.routed.angles_45.is_none() && out.routed.tuned.is_none());
    let first = dir.join("a.kicad_pcb");
    let items = out.write(&out.routed.res.geometry, &first).unwrap();
    assert!(items >= 1);
    assert_eq!(
        std::fs::read_to_string(&first).unwrap().matches("(segment").count(),
        items
    );

    // net B joins on the routed board, as 45 degree tracks; the track of A stays
    let opts = RouteOptions {
        keep_all: true,
        nets: vec!["B".into()],
        angles: gummiband_app::Angles::Deg45,
        ..opts
    };
    let out = route_file(&first, &opts, &Hooks::default()).unwrap();
    assert!(out.routed.shove);
    assert_eq!(out.routed.res.stats.routed, 1);
    assert!(out.routed.angles_45.is_some());
    let second = dir.join("ab.kicad_pcb");
    let added = out.write(&out.routed.res.geometry, &second).unwrap();
    let text = std::fs::read_to_string(&second).unwrap();
    assert!(
        added >= 1 && text.matches("(segment").count() >= items + added,
        "{items} {added}"
    );

    // a net the board lacks, and a hint that does not exist
    let bad = RouteOptions {
        nets: vec!["nope".into()],
        ..Default::default()
    };
    assert!(route_file(&pcb, &bad, &Hooks::default()).is_err());
    let bad = RouteOptions {
        patterns: "bogus".into(),
        ..Default::default()
    };
    assert!(route_file(&pcb, &bad, &Hooks::default()).is_err());
    // the pipeline checks the options itself, whoever calls it
    let bad = RouteOptions {
        runs: 0,
        ..Default::default()
    };
    let e = route_file(&pcb, &bad, &Hooks::default()).err().unwrap();
    assert_eq!(e.to_string(), "runs must be at least 1");
    let _ = std::fs::remove_dir_all(&dir);
}
