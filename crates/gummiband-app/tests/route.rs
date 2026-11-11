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
    // a pour is only looked at when the fill connects
    let pour = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<String>>();
    refused(
        RouteOptions {
            pour: pour(&["GND"]),
            ..d.clone()
        },
        false,
        "pour needs areas fill",
    );
    let fill = RouteOptions {
        areas: gummiband_app::Areas::Fill,
        pour: pour(&["GND:F.Cu,B.Cu", "VCC"]),
        ..d.clone()
    };
    assert_eq!(fill.validate(false), Ok(()));
    refused(
        RouteOptions {
            pour: pour(&["GND", "GND:F.Cu"]),
            ..fill.clone()
        },
        false,
        "names GND twice",
    );
    refused(
        RouteOptions {
            nets: vec!["VCC".into(), "A".into()],
            ..fill.clone()
        },
        false,
        "pour names GND, which nets leaves out",
    );
    refused(
        RouteOptions {
            pairs: true,
            ..fill.clone()
        },
        false,
        "areas fill does not work with pairs yet",
    );
    refused(
        fill.clone(),
        true,
        "areas fill does not work with an open ended run yet",
    );
    refused(
        RouteOptions {
            area_stitch: Some(true),
            ..d.clone()
        },
        false,
        "area_stitch needs areas fill",
    );
    assert!(fill.area_stitch() && !d.area_stitch());
    // finishing nets shoves by default
    refused(
        RouteOptions {
            keep_all: true,
            nets: vec!["GND".into(), "VCC".into()],
            ..fill.clone()
        },
        false,
        "areas fill does not work with shove yet",
    );
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
    std::fs::write(pcb.with_extension("kicad_pro"), "{}").unwrap();
    std::fs::write(pcb.with_extension("kicad_dru"), "(version 1)\n").unwrap();

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
    let first = dir.join("a.kicad_pcb");
    let out = route_file(&pcb, &first, &opts, &hooks).unwrap();
    assert_eq!(movable.get(), None, "parts are fixed");
    let st = &out.routed.res.stats;
    assert_eq!((st.conns, st.routed), (1, 1));
    assert!(out.routed.violations.is_empty() && out.routed.angles_45.is_none() && out.routed.tuned.is_none());
    assert!(out.routed.seconds >= out.routed.search_seconds && out.routed.search_seconds > 0.0);
    assert_eq!(out.routed.seed, 0x5eed, "the seed of the core when none is given");
    let written = out.write().unwrap();
    let items = written.items;
    assert!(items >= 1);
    assert!(written.alternatives.is_empty(), "no open end, nothing to choose from");
    assert_eq!(
        written.copied,
        vec![first.with_extension("kicad_pro"), first.with_extension("kicad_dru")],
        "the project files of the input go next to the output"
    );
    // a second write leaves the project files that are there alone
    assert!(out.write().unwrap().copied.is_empty());
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
    let second = dir.join("ab.kicad_pcb");
    let out = route_file(&first, &second, &opts, &Hooks::default()).unwrap();
    assert!(out.routed.shove);
    assert_eq!(out.routed.res.stats.routed, 1);
    assert!(out.routed.angles_45.is_some());
    let added = out.write().unwrap().items;
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
    assert!(route_file(&pcb, &second, &bad, &Hooks::default()).is_err());
    let bad = RouteOptions {
        patterns: "bogus".into(),
        ..Default::default()
    };
    assert!(route_file(&pcb, &second, &bad, &Hooks::default()).is_err());
    // the pipeline checks the options itself, whoever calls it
    let bad = RouteOptions {
        runs: 0,
        ..Default::default()
    };
    let e = route_file(&pcb, &second, &bad, &Hooks::default()).err().unwrap();
    assert_eq!(e.to_string(), "runs must be at least 1");
    // the input is never the output, however the path is spelled
    let same = dir.join(".").join("tiny.kicad_pcb");
    let e = route_file(&pcb, &same, &RouteOptions::default(), &Hooks::default())
        .err()
        .unwrap();
    assert!(
        e.to_string().contains("the output must differ from the input file"),
        "{e}"
    );
    let _ = std::fs::remove_dir_all(&dir);
}

/// The options per net are one thing, whichever way they come: the file of `net_widths` in its
/// two forms, and the map a front end holds itself.
#[test]
fn net_options_from_a_file_and_from_the_front_end_agree() {
    use gummiband_app::NetOptions;
    let dir = std::env::temp_dir().join(format!("gummiband-app-nets-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let pcb = dir.join("tiny.kicad_pcb");
    std::fs::write(&pcb, tiny_board()).unwrap();
    let file = dir.join("nets.json");
    std::fs::write(
        &file,
        r#"{"A": 0.4, "B": {"width": 0.3, "min_width": 0.3, "priority": 0}, "not on the board": 1}"#,
    )
    .unwrap();
    let base = RouteOptions {
        runs: 1,
        generations: 0,
        geometry_generations: 0,
        move_parts: Some(false),
        ..Default::default()
    };
    let counted = std::cell::Cell::new(None);
    let event = |e: Event<'_>| {
        if let Event::NetWidths { nets } = e {
            counted.set(Some(nets));
        }
    };
    let hooks = Hooks {
        event: Some(&event),
        ..Default::default()
    };
    let from_file = RouteOptions {
        net_widths: Some(file.clone()),
        ..base.clone()
    };
    let routed = dir.join("routed.kicad_pcb");
    let out = route_file(&pcb, &routed, &from_file, &hooks).unwrap();
    assert_eq!(counted.get(), Some(2), "the nets of the file that the board has");
    let (text, _) = out.routed.board_text(&out.kb, &out.routed.res.geometry).unwrap();
    assert!(text.contains("(width 0.4)") && text.contains("(width 0.3)"), "{text}");
    let b = &out.kb.board;
    let class = |net: &str| &b.classes[b.nets[b.net_id(net).unwrap() as usize].class as usize];
    assert_eq!((class("A").track_width, class("A").min_width), (0.4, None));
    assert_eq!((class("B").track_width, class("B").min_width), (0.3, Some(0.3)));
    assert_eq!(b.nets[b.net_id("B").unwrap() as usize].priority, 0);

    let held = RouteOptions {
        net_options: [
            (
                "A".to_string(),
                NetOptions {
                    width: Some(0.4),
                    ..Default::default()
                },
            ),
            (
                "B".to_string(),
                NetOptions {
                    width: Some(0.3),
                    min_width: Some(0.3),
                    priority: Some(0),
                    ..Default::default()
                },
            ),
        ]
        .into(),
        ..base.clone()
    };
    let again = route_file(&pcb, &routed, &held, &Hooks::default()).unwrap();
    let (same, _) = again.routed.board_text(&again.kb, &again.routed.res.geometry).unwrap();
    assert_eq!(text, same);

    // a skipped net is left out and named; what the front end holds wins over the file
    let skipping = RouteOptions {
        net_options: [(
            "A".to_string(),
            NetOptions {
                skip: true,
                ..Default::default()
            },
        )]
        .into(),
        ..from_file
    };
    let out = route_file(&pcb, &routed, &skipping, &Hooks::default()).unwrap();
    assert_eq!(out.skipped_nets, vec!["A".to_string()]);
    assert_eq!(out.routed.res.stats.conns, 1);
    let _ = std::fs::remove_dir_all(&dir);
}

/// The open end with a time budget ends by itself, and its clock starts with the open end: a
/// budget shorter than the search before it still leaves that search whole.
#[test]
fn the_time_budget_starts_with_the_open_end() {
    let dir = std::env::temp_dir().join(format!("gummiband-app-open-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let pcb = dir.join("tiny.kicad_pcb");
    std::fs::write(&pcb, tiny_board()).unwrap();
    let phases = std::sync::Arc::new(std::sync::Mutex::new(Vec::<String>::new()));
    let seen = phases.clone();
    let progress = move |prefix: &str| {
        let (seen, prefix) = (seen.clone(), prefix.to_string());
        gummiband_core::ProgressFn(std::sync::Arc::new(move |p: &gummiband_core::Progress| {
            seen.lock().unwrap().push(format!("{prefix}{}", p.phase));
        }))
    };
    let hooks = Hooks {
        progress: Some(&progress),
        ..Default::default()
    };
    let opts = RouteOptions {
        runs: 2,
        generations: 1,
        geometry_generations: 1,
        move_parts: Some(false),
        seed: Some(7),
        open_end_seconds: 0.001,
        ..Default::default()
    };
    assert!(opts.open_end(false));
    assert_eq!(opts.air(opts.open_end(false)), defaults::OPEN_END_AIR);
    let out = route_file(&pcb, &dir.join("open.kicad_pcb"), &opts, &hooks).unwrap();
    assert_eq!(out.routed.seed, 7);
    assert_eq!(out.routed.res.stats.routed, 2);
    let phases = phases.lock().unwrap();
    let open = phases.iter().position(|p| p == gummiband_core::OPEN_END_PHASE);
    let geometry = phases.iter().position(|p| p == "geometry");
    assert!(
        geometry.is_some(),
        "the budget ran out before the search and cut it: {phases:?}"
    );
    assert!(open.is_some() && geometry < open, "{phases:?}");
    let _ = std::fs::remove_dir_all(&dir);
}
