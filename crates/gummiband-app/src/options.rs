//! The options of a route as a table: name, kind of value, default and one sentence each. The
//! CLI offers every entry as `--kebab-case`, the MCP tool `route` as an argument of the same
//! name in `snake_case`; tests in both front ends fail when an entry is missing there, or when
//! one of them offers an option of the pipeline that the table lacks. The options with a fixed
//! list of words ([`Kind::Choice`]) get their help text and their schema from this table
//! ([`OptionDoc::describe`]); the other help texts are to follow.

/// What a user gives for an option.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// Off unless given (CLI: a flag, MCP: true).
    Switch,
    /// On or off, with a default that may follow from other options (CLI: `--name` and
    /// `--no-name`, MCP: true or false).
    OnOff,
    /// A number.
    Number,
    /// One word of a fixed list, see [`OptionDoc::choices`].
    Choice,
    /// A comma separated list of words with a fixed vocabulary.
    Text,
    /// Names: nets, references, layers (CLI: comma separated, MCP: a list).
    List,
    /// A path.
    Path,
}

/// One option of a pipeline: of [`RouteOptions`](crate::RouteOptions) in [`OPTIONS`], of
/// [`AreaOptions`](crate::AreaOptions) in [`AREA_OPTIONS`](crate::AREA_OPTIONS).
#[derive(Debug, Clone, Copy)]
pub struct OptionDoc {
    /// The field of the options and the MCP argument; the CLI flag is this in kebab-case.
    pub name: &'static str,
    pub kind: Kind,
    /// The default in words, the same in every front end. For a plain number it is the number
    /// as Rust prints it.
    pub default: &'static str,
    /// What the option does, in one sentence.
    pub help: &'static str,
    /// [`Kind::Choice`]: the words to choose from, in the order a help text lists them.
    pub choices: &'static [Choice],
}

/// One word of an option of [`Kind::Choice`].
#[derive(Debug, Clone, Copy)]
pub struct Choice {
    /// The word, the same in every front end.
    pub value: &'static str,
    /// What it chooses and when to pick it, in one sentence.
    pub help: &'static str,
}

impl OptionDoc {
    /// The CLI flag without its dashes.
    pub fn flag(&self) -> String {
        self.name.replace('_', "-")
    }

    /// The words of a choice, as a help text lists them: `off, pull or band`.
    pub fn words(&self) -> String {
        let values: Vec<&str> = self.choices.iter().map(|c| c.value).collect();
        match values.split_last() {
            Some((last, rest)) if !rest.is_empty() => format!("{} or {last}", rest.join(", ")),
            _ => values.concat(),
        }
    }

    /// The word of a choice as the table spells it. A front end may get it with dashes in place
    /// of underscores. The error names the option and its words.
    pub fn choice(&self, word: &str) -> Result<&'static str, String> {
        let plain = word.replace('-', "_");
        self.choices
            .iter()
            .map(|c| c.value)
            .find(|v| *v == plain)
            .ok_or_else(|| format!("{} must be {}, not {word:?}", self.name, self.words()))
    }

    /// The whole text of the option for a help or a schema: what it does, then one line per
    /// word of a choice, then the default.
    pub fn describe(&self) -> String {
        self.describe_with(self.default)
    }

    /// As [`OptionDoc::describe`], for a front end whose default depends on something the table
    /// does not know (the MCP session and the copper its board was loaded with).
    pub fn describe_with(&self, default: &str) -> String {
        let mut text = self.help.to_string();
        for c in self.choices {
            text.push_str(&format!("\n{}: {}", c.value, c.help));
        }
        text.push_str(&format!("\nDefault: {default}."));
        text
    }
}

pub(crate) const fn opt(name: &'static str, kind: Kind, default: &'static str, help: &'static str) -> OptionDoc {
    OptionDoc {
        name,
        kind,
        default,
        help,
        choices: &[],
    }
}

const fn choice(
    name: &'static str,
    default: &'static str,
    help: &'static str,
    choices: &'static [Choice],
) -> OptionDoc {
    OptionDoc {
        name,
        kind: Kind::Choice,
        default,
        help,
        choices,
    }
}

const fn word(value: &'static str, help: &'static str) -> Choice {
    Choice { value, help }
}

/// Every option of a route that a user can set, in the order of `RouteOptions`.
pub const OPTIONS: &[OptionDoc] = &[
    choice(
        "scope",
        "fresh",
        "What of the copper on the board stays, and what is laid.",
        &[
            word(
                "fresh",
                "No copper stays: every track and via goes, locked ones too, and the board is laid anew.",
            ),
            word(
                "keep_locked",
                "Locked tracks and vias stay as fixed copper that the router connects to, all unlocked copper goes and is laid anew: for a board on which the copper that must stay is locked.",
            ),
            word(
                "finish",
                "All copper stays, locked or not, and only what is still open is laid, with nets only the open connections of those nets: for \"I laid these nets by hand, finish the rest\" and for a track that was started.",
            ),
            word(
                "reroute",
                "Needs nets: the unlocked copper of those nets is taken out and they are laid anew, everything else stays: for \"lay net X again\".",
            ),
        ],
    ),
    opt(
        "nets",
        Kind::List,
        "all nets",
        "Lay only these nets: with scope finish and reroute the others stay as they are, with fresh and keep_locked they stay unrouted.",
    ),
    opt(
        "shove",
        Kind::OnOff,
        "on with scope reroute and with scope finish and nets, else off",
        "Single nets on kept copper: the unlocked tracks of the other nets shift aside to let a connection pass.",
    ),
    opt(
        "give",
        Kind::Number,
        "8 with scope reroute and with scope finish and nets, else 0",
        "Single nets on kept copper: radius in mm within which the unlocked tracks of other nets give way and are routed again when a connection finds no way.",
    ),
    opt(
        "move_parts",
        Kind::OnOff,
        "on with scope fresh and keep_locked, off with finish and reroute",
        "Let parts next to clearance problems and open connections shift a little while the copper follows.",
    ),
    opt(
        "fixed",
        Kind::List,
        "none",
        "References that never move under move_parts, placements and clear_corridors.",
    ),
    opt(
        "plane_layers",
        Kind::List,
        "none",
        "Copper layers treated as planes besides those of KiCad type \"power\": not routed, their zones stay.",
    ),
    opt(
        "areas",
        Kind::Text,
        "tracks",
        "How nets with an area (a zone of the file or a pour of pour) on a routed layer are connected: tracks (like any net, the area is not looked at) or fill (they are not laid as tracks, the fill connects them, and the report says which pads it does not reach).",
    ),
    opt(
        "pour",
        Kind::List,
        "none",
        "With areas fill: nets that get a pour after the route, each NET or NET:LAYER,LAYER (the outer layers where none is given), which the route takes as an area of the net.",
    ),
    opt(
        "area_stitch",
        Kind::OnOff,
        "on with areas fill",
        "With areas fill: an island of fill that holds pads gets one via where it joins the area, set on the finished copper where the fill model finds the spot.",
    ),
    choice(
        "preset",
        "standard",
        "How much search the route gets: a preset sets runs, generations, geometry_generations and open_end_seconds, and each of the four wins over it when given.",
        &[
            word(
                "draft",
                "runs 4, generations 1, geometry_generations 0, open_end_seconds 0: the quickest look at whether the board routes and where it is tight, as the benchmark suite routes; leaves many clearance violations.",
            ),
            word(
                "quick",
                "runs 14, generations 3, geometry_generations 0, open_end_seconds 0: the whole order search without the rounds that select by the clearance check, in about a third of the time of standard and with many more clearance violations: for a first result on a real board.",
            ),
            word(
                "standard",
                "runs 14, generations 3, geometry_generations 3, open_end_seconds 0: the defaults, a usable board in a few minutes.",
            ),
            word(
                "final",
                "runs 14, generations 3, geometry_generations 3, open_end_seconds 600: standard, then the open end improves the result for 10 minutes (fewer vias, fewer violations, sometimes the last connections), for the last run of a board and not with pairs band.",
            ),
        ],
    ),
    opt(
        "runs",
        Kind::Number,
        "what the preset says: 14 with standard",
        "Portfolio size: router runs with different orders in every generation.",
    ),
    opt("via_pitch", Kind::Number, "0.5", "Via site grid pitch in mm."),
    opt("via_cost", Kind::Number, "3", "Via cost in mm of track."),
    opt(
        "layer_dirs",
        Kind::Number,
        "0.5",
        "Weight of the preferred layer directions in every third run (0: off).",
    ),
    opt(
        "patterns",
        Kind::Text,
        "none",
        "Hints from local patterns of the board, comma separated: shares, decaps, rows, pairs, groups, groups-short, all or none.",
    ),
    opt(
        "via_pass",
        Kind::Number,
        "4",
        "Factor on the via cost in the pass that routes connections with vias again and keeps what has fewer vias (0: off).",
    ),
    choice(
        "pairs",
        "off",
        "How the two nets of a differential pair are laid.",
        &[
            word("off", "Each net on its own, like any other net."),
            word(
                "pull",
                "Two nets, the second pulled to the way of the first with the weight pair_pull, and the finished tracks close up to the pair gap where nothing lies between: for every pair recognised by name, the gentle way on a board that is not dense.",
            ),
            word(
                "band",
                "Experimental: one wide track per pair that is split into two parallel tracks, for the pairs that a diff_pair_gap rule or a net class of their own marks (pair_all: every pair), which keeps them coupled, costs connections on a dense board and does not work with an open end.",
            ),
        ],
    ),
    opt(
        "pair_pull",
        Kind::Number,
        "1 with pairs pull, 0 with pairs band",
        "With pairs pull: weight of the pull between the two nets (1 is a good start, up to about 4); with pairs band: the same pull for the pairs that get no wide track.",
    ),
    opt(
        "pair_skew",
        Kind::Number,
        "0.1",
        "With pairs band: length difference left within a pair in mm.",
    ),
    opt(
        "pair_zdiff",
        Kind::Number,
        "none: the widths of the net classes and rules",
        "With pairs band: target differential impedance in ohm, which sets the track width per layer from the stackup.",
    ),
    opt(
        "pair_all",
        Kind::Switch,
        "off",
        "With pairs band: a wide track for every pair recognised by name, not only those a rule or a net class of their own marks.",
    ),
    opt(
        "breakouts",
        Kind::Switch,
        "off",
        "Experimental, with pairs band: the wide track ends just outside the pads and two single tracks connect them.",
    ),
    opt(
        "via_pairs",
        Kind::Switch,
        "off",
        "Experimental, with pairs band: two vias side by side are one via pair that turns with the wide track.",
    ),
    opt(
        "negotiate",
        Kind::Number,
        "0",
        "Rounds of negotiated routing from the start, in a second search next to the plain one (0: only when the plain search leaves a few connections open).",
    ),
    opt(
        "generations",
        Kind::Number,
        "what the preset says: 3 with standard",
        "Generations of the evolutionary order search after the portfolio.",
    ),
    opt(
        "geometry_generations",
        Kind::Number,
        "what the preset says: 3 with standard",
        "Generations that select by clearance violations after the geometry step (0: fastest, many more violations).",
    ),
    opt(
        "seed",
        Kind::Number,
        "GB_EVO_SEED, else a constant",
        "Seed of the search: the same seed gives the same result, another one a different try.",
    ),
    opt(
        "frames",
        Kind::Path,
        "none",
        "Folder for one SVG of the best candidate per generation, with an animation if ImageMagick is there.",
    ),
    opt(
        "open_end_seconds",
        Kind::Number,
        "what the preset says: 0 with standard",
        "Open end with a time budget: after the normal search keep improving for this many seconds, then finish the best result (0: off).",
    ),
    opt(
        "open_via_cost",
        Kind::Number,
        "20",
        "In the open end: what a via is worth in mm of track when its results are judged.",
    ),
    opt(
        "air",
        Kind::Number,
        "0.03 in an open ended run, but 0 with scope reroute and with scope finish and nets, else 0",
        "Reserve in mm on top of the clearances that the finished copper gets where there is room.",
    ),
    opt(
        "prefer_width",
        Kind::Number,
        "0",
        "Width in mm that tracks of the default net class grow to at the end where there is room (0: off).",
    ),
    choice(
        "angles",
        "any",
        "Track angles of the result.",
        &[
            word("any", "Any-angle tracks with arcs, as the rubber bands lie."),
            word(
                "45",
                "Conventional 45 degree routing as a last step, which never adds a clearance violation.",
            ),
        ],
    ),
    opt(
        "tune",
        Kind::OnOff,
        "on",
        "Meanders for the length and skew rules of the .kicad_dru.",
    ),
    opt("tune_amplitude", Kind::Number, "1", "Highest meander bump in mm."),
    opt(
        "tune_spacing",
        Kind::Number,
        "2",
        "Air between neighbouring meander legs in track widths.",
    ),
    opt(
        "placements",
        Kind::Number,
        "0",
        "Try this many placements from the netlist, route each and go on with the best (0: the placement stays).",
    ),
    opt(
        "placement_seed",
        Kind::Number,
        "1",
        "First seed of placements, and the seed of clear_corridors.",
    ),
    opt(
        "clear_corridors",
        Kind::Switch,
        "off",
        "Experimental: before routing, parts in the band of a group of tracks between two parts get new positions.",
    ),
    opt(
        "net_widths",
        Kind::Path,
        "none",
        "JSON file with options per net: a track width in mm, or an object with width, min_width, neck_refs, priority and skip.",
    ),
    opt(
        "net_priorities",
        Kind::Path,
        "none",
        "JSON file with a routing priority per net, lower routes first.",
    ),
    opt(
        "teardrops",
        Kind::Switch,
        "off",
        "Switch on KiCad's teardrops in the written board and its project.",
    ),
];

/// The entry of an option.
pub fn option_doc(name: &str) -> Option<&'static OptionDoc> {
    OPTIONS.iter().find(|o| o.name == name)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::RouteOptions;

    /// The names of the fields of `RouteOptions`. A field that is missing here does not compile.
    macro_rules! fields {
        ($($f:ident),* $(,)?) => {{
            let RouteOptions { $($f: _),* } = RouteOptions::default();
            vec![$(stringify!($f)),*]
        }};
    }

    #[test]
    fn the_table_has_every_option_of_the_pipeline_once() {
        let fields = fields!(
            scope,
            nets,
            shove,
            give,
            move_parts,
            fixed,
            plane_layers,
            areas,
            pour,
            area_stitch,
            preset,
            runs,
            via_pitch,
            via_cost,
            layer_dirs,
            patterns,
            via_pass,
            pairs,
            pair_pull,
            pair_skew,
            pair_zdiff,
            pair_all,
            breakouts,
            via_pairs,
            negotiate,
            generations,
            geometry_generations,
            seed,
            frames,
            open_end_seconds,
            open_via_cost,
            air,
            prefer_width,
            angles,
            tune,
            tune_amplitude,
            tune_spacing,
            placements,
            placement_seed,
            clear_corridors,
            net_widths,
            net_priorities,
            net_options,
            teardrops,
        );
        // net_options is what a front end holds per net itself (the session of the MCP server,
        // filled by set_net_options); a user reaches the same through the file of net_widths
        let offered: Vec<&str> = fields.into_iter().filter(|f| *f != "net_options").collect();
        let table: Vec<&str> = OPTIONS.iter().map(|o| o.name).collect();
        assert_eq!(table, offered, "OPTIONS and the fields of RouteOptions, in order");
        for o in OPTIONS {
            assert!(
                o.help.ends_with('.') && !o.help.trim_end_matches('.').contains(". "),
                "{}: one sentence",
                o.name
            );
            assert!(!o.default.is_empty(), "{}", o.name);
            // a choice has its words, each with one sentence, and its default is one of them
            assert_eq!(o.kind == Kind::Choice, !o.choices.is_empty(), "{}", o.name);
            for c in o.choices {
                assert!(
                    c.help.ends_with('.') && !c.help.trim_end_matches('.').contains(". "),
                    "{} {}: one sentence",
                    o.name,
                    c.value
                );
                assert_eq!(o.choice(c.value), Ok(c.value));
            }
            if o.kind == Kind::Choice {
                assert!(
                    o.choice(o.default.split([' ', ',']).next().unwrap()).is_ok(),
                    "{}",
                    o.name
                );
            }
        }
    }

    #[test]
    fn a_choice_takes_its_words_and_names_them_otherwise() {
        let pairs = option_doc("pairs").unwrap();
        assert_eq!(pairs.words(), "off, pull or band");
        assert_eq!(pairs.choice("band"), Ok("band"));
        assert_eq!(
            pairs.choice("wide"),
            Err("pairs must be off, pull or band, not \"wide\"".to_string())
        );
        let text = pairs.describe();
        assert!(
            text.starts_with(pairs.help) && text.ends_with("Default: off."),
            "{text}"
        );
        assert!(text.contains("\npull: ") && text.contains("\nband: "), "{text}");
        // every word of the table is a value of its type, and the other way round
        for c in pairs.choices {
            assert_eq!(crate::Pairs::parse(c.value).unwrap().word(), c.value);
        }
        assert_eq!(pairs.choices.len(), crate::Pairs::ALL.len());
        let scope = option_doc("scope").unwrap();
        for c in scope.choices {
            assert_eq!(crate::Scope::parse(c.value).unwrap().word(), c.value);
        }
        assert_eq!(scope.choices.len(), crate::Scope::ALL.len());
        // a front end may spell a word with a dash
        assert_eq!(crate::Scope::parse("keep-locked"), Ok(crate::Scope::KeepLocked));
        // the sentences an agent picks a scope by
        let said = |word: &str| scope.choices.iter().find(|c| c.value == word).unwrap().help;
        assert!(said("finish").contains("\"I laid these nets by hand, finish the rest\""));
        assert!(said("reroute").contains("\"lay net X again\"") && said("reroute").starts_with("Needs nets"));
    }

    #[test]
    fn the_defaults_in_words_are_the_defaults() {
        let d = RouteOptions::default();
        let on_off = |b: bool| if b { "on" } else { "off" }.to_string();
        let plain = [
            ("scope", d.scope.word().to_string()),
            ("preset", d.preset.word().to_string()),
            ("via_pitch", d.via_pitch.to_string()),
            ("via_cost", d.via_cost.to_string()),
            ("layer_dirs", d.layer_dirs.to_string()),
            ("patterns", d.patterns.clone()),
            ("via_pass", d.via_pass.to_string()),
            ("pairs", d.pairs.word().to_string()),
            ("pair_skew", d.pair_skew.to_string()),
            ("pair_all", on_off(d.pair_all)),
            ("breakouts", on_off(d.breakouts)),
            ("via_pairs", on_off(d.via_pairs)),
            ("negotiate", d.negotiate.to_string()),
            ("open_via_cost", d.open_via_cost.to_string()),
            ("prefer_width", d.prefer_width.to_string()),
            ("tune", on_off(d.tune)),
            ("tune_amplitude", d.tune_amplitude.to_string()),
            ("tune_spacing", d.tune_spacing.to_string()),
            ("placements", d.placements.to_string()),
            ("placement_seed", d.placement_seed.to_string()),
            ("clear_corridors", on_off(d.clear_corridors)),
            ("teardrops", on_off(d.teardrops)),
        ];
        for (name, value) in plain {
            assert_eq!(option_doc(name).unwrap().default, value, "{name}");
        }
        // the defaults that follow from other options name their numbers
        let text = |name: &str| option_doc(name).unwrap().default;
        assert!(text("give").starts_with(&crate::defaults::GIVE.to_string()));
        assert!(text("pair_pull").starts_with(&format!("{} with pairs pull", crate::defaults::PAIR_PULL)));
        assert_eq!(d.pair_pull(), 0.0);
        assert!(text("air").starts_with(&crate::defaults::OPEN_END_AIR.to_string()));
        assert!(d.move_parts() && text("move_parts").starts_with("on with scope fresh"));
        // the defaults that hang on the scope say so in one line
        for name in ["shove", "give", "move_parts", "air"] {
            assert!(text(name).contains("scope") && !text(name).contains('\n'), "{name}");
        }
        assert!(!d.shove() && text("shove").ends_with("else off"));
        assert_eq!(text("angles"), "any");
        assert_eq!(d.angles, crate::Angles::Any);
        assert_eq!(text("areas"), d.areas.word());
        assert!(d.pour.is_empty() && text("pour") == "none");
        assert!(!d.area_stitch() && text("area_stitch") == "on with areas fill");
        // the four that a preset sets say so, with the number of the default preset
        let standard = crate::Preset::Standard.values();
        let said = |n: String| format!("what the preset says: {n} with standard");
        assert_eq!(text("runs"), said(standard.runs.to_string()));
        assert_eq!(text("generations"), said(standard.generations.to_string()));
        assert_eq!(
            text("geometry_generations"),
            said(standard.geometry_generations.to_string())
        );
        assert_eq!(text("open_end_seconds"), said(standard.open_end_seconds.to_string()));
    }

    /// A preset is a word of the table, and its sentence starts with the four values it sets:
    /// what the help says is what the route gets.
    #[test]
    fn a_preset_says_what_it_sets() {
        use crate::Preset;
        let doc = option_doc("preset").unwrap();
        assert_eq!(doc.choices.len(), Preset::ALL.len());
        for (c, p) in doc.choices.iter().zip(Preset::ALL) {
            assert_eq!(c.value, p.word());
            assert_eq!(Preset::parse(c.value), Ok(p));
            let v = p.values();
            let sets = format!(
                "runs {}, generations {}, geometry_generations {}, open_end_seconds {}: ",
                v.runs, v.generations, v.geometry_generations, v.open_end_seconds
            );
            assert!(c.help.starts_with(&sets), "{}: {}", c.value, c.help);
        }
        // the default preset is the defaults of old, so a route without a preset is unchanged
        let d = RouteOptions::default();
        assert_eq!(d.preset, Preset::Standard);
        assert_eq!(
            (
                d.runs(),
                d.generations(),
                d.geometry_generations(),
                d.open_end_seconds()
            ),
            (
                crate::defaults::RUNS,
                crate::defaults::GENERATIONS,
                crate::defaults::GEOMETRY_GENERATIONS,
                crate::defaults::OPEN_END_SECONDS
            )
        );
        // an option that is given wins over the preset, each on its own
        let o = RouteOptions {
            preset: Preset::Draft,
            geometry_generations: Some(2),
            ..RouteOptions::default()
        };
        assert_eq!((o.runs(), o.generations(), o.geometry_generations()), (4, 1, 2));
        let o = RouteOptions {
            preset: Preset::Final,
            open_end_seconds: Some(0.0),
            ..RouteOptions::default()
        };
        assert!(!o.open_end(false) && o.runs() == crate::defaults::RUNS);
        assert!(
            RouteOptions {
                preset: Preset::Final,
                ..RouteOptions::default()
            }
            .open_end(false)
        );
        // final is an open end, which the wide track of pairs does not have: said, not dropped
        let band = RouteOptions {
            preset: Preset::Final,
            pairs: crate::Pairs::Band,
            ..RouteOptions::default()
        };
        assert!(band.validate(false).unwrap_err().contains("preset final"));
        assert_eq!(
            Preset::parse("fast").unwrap_err(),
            "preset must be draft, quick, standard or final, not \"fast\""
        );
    }
}
