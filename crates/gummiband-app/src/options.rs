//! The options of a route as a table: name, kind of value, default and one sentence each. The
//! CLI offers every entry as `--kebab-case`, the MCP tool `route` as an argument of the same
//! name in `snake_case`; tests in both front ends fail when an entry is missing there, or when
//! one of them offers an option of the pipeline that the table lacks. Help texts are to be
//! generated from this table.

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
    /// A word or a comma separated list of words with a fixed vocabulary.
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
}

impl OptionDoc {
    /// The CLI flag without its dashes.
    pub fn flag(&self) -> String {
        self.name.replace('_', "-")
    }
}

pub(crate) const fn opt(name: &'static str, kind: Kind, default: &'static str, help: &'static str) -> OptionDoc {
    OptionDoc {
        name,
        kind,
        default,
        help,
    }
}

/// Every option of a route that a user can set, in the order of `RouteOptions`.
pub const OPTIONS: &[OptionDoc] = &[
    opt(
        "keep_locked",
        Kind::Switch,
        "off",
        "Keep locked tracks and vias as fixed copper and route around them.",
    ),
    opt(
        "keep_all",
        Kind::Switch,
        "off",
        "Keep all tracks and vias as fixed copper and add to them.",
    ),
    opt("nets", Kind::List, "all nets", "Route only these nets."),
    opt(
        "reroute",
        Kind::Switch,
        "off",
        "With keep_all and nets: remove the unlocked copper of those nets first and route them anew.",
    ),
    opt(
        "shove",
        Kind::OnOff,
        "on when finishing nets (keep_all and nets), else off",
        "Finishing nets: the unlocked tracks of the other nets shift aside to let a connection pass.",
    ),
    opt(
        "give",
        Kind::Number,
        "8 when finishing nets (keep_all and nets), else 0",
        "Finishing nets: radius in mm within which the unlocked tracks of other nets give way and are routed again when a connection finds no way.",
    ),
    opt(
        "move_parts",
        Kind::OnOff,
        "on, off with keep_all",
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
        "runs",
        Kind::Number,
        "14",
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
    opt(
        "pair_pull",
        Kind::Number,
        "0",
        "Weight of the pull between the nets of differential pairs (0: off).",
    ),
    opt(
        "pairs",
        Kind::Switch,
        "off",
        "Experimental: route differential pairs as one wide track and split it into two parallel tracks.",
    ),
    opt(
        "pair_skew",
        Kind::Number,
        "0.1",
        "With pairs: length difference left within a pair in mm.",
    ),
    opt(
        "pair_zdiff",
        Kind::Number,
        "none: the widths of the net classes and rules",
        "With pairs: target differential impedance in ohm, which sets the track width per layer from the stackup.",
    ),
    opt(
        "pair_all",
        Kind::Switch,
        "off",
        "With pairs: couple every pair recognised by name, not only those a rule or a net class of their own marks.",
    ),
    opt(
        "breakouts",
        Kind::Switch,
        "off",
        "Experimental, with pairs: the wide track ends just outside the pads and two single tracks connect them.",
    ),
    opt(
        "via_pairs",
        Kind::Switch,
        "off",
        "Experimental, with pairs: two vias side by side are one via pair that turns with the wide track.",
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
        "3",
        "Generations of the evolutionary order search after the portfolio.",
    ),
    opt(
        "geometry_generations",
        Kind::Number,
        "3",
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
        "0",
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
        "0.03 in an open ended run of the whole board, else 0",
        "Reserve in mm on top of the clearances that the finished copper gets where there is room.",
    ),
    opt(
        "prefer_width",
        Kind::Number,
        "0",
        "Width in mm that tracks of the default net class grow to at the end where there is room (0: off).",
    ),
    opt(
        "angles",
        Kind::Text,
        "any",
        "Track angles of the result: any (any-angle tracks with arcs) or 45 (conventional 45 degree routing as a last step).",
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
            keep_locked,
            keep_all,
            nets,
            reroute,
            shove,
            give,
            move_parts,
            fixed,
            plane_layers,
            areas,
            pour,
            runs,
            via_pitch,
            via_cost,
            layer_dirs,
            patterns,
            via_pass,
            pair_pull,
            pairs,
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
        }
    }

    #[test]
    fn the_defaults_in_words_are_the_defaults() {
        let d = RouteOptions::default();
        let on_off = |b: bool| if b { "on" } else { "off" }.to_string();
        let plain = [
            ("keep_locked", on_off(d.keep_locked)),
            ("keep_all", on_off(d.keep_all)),
            ("reroute", on_off(d.reroute)),
            ("runs", d.runs.to_string()),
            ("via_pitch", d.via_pitch.to_string()),
            ("via_cost", d.via_cost.to_string()),
            ("layer_dirs", d.layer_dirs.to_string()),
            ("patterns", d.patterns.clone()),
            ("via_pass", d.via_pass.to_string()),
            ("pair_pull", d.pair_pull.to_string()),
            ("pairs", on_off(d.pairs)),
            ("pair_skew", d.pair_skew.to_string()),
            ("pair_all", on_off(d.pair_all)),
            ("breakouts", on_off(d.breakouts)),
            ("via_pairs", on_off(d.via_pairs)),
            ("negotiate", d.negotiate.to_string()),
            ("generations", d.generations.to_string()),
            ("geometry_generations", d.geometry_generations.to_string()),
            ("open_end_seconds", d.open_end_seconds.to_string()),
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
        assert!(text("air").starts_with(&crate::defaults::OPEN_END_AIR.to_string()));
        assert!(d.move_parts() && text("move_parts").starts_with("on"));
        assert!(!d.shove() && text("shove").ends_with("else off"));
        assert_eq!(text("angles"), "any");
        assert_eq!(d.angles, crate::Angles::Any);
        assert_eq!(text("areas"), d.areas.word());
        assert!(d.pour.is_empty() && text("pour") == "none");
    }
}
