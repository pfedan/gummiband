//! Patch a KiCad board: drop existing copper items and append routed ones. The rest of the file
//! is kept byte for byte.

use std::fmt::Write as _;
use std::ops::Range;

use gummiband_core::board::{LayerId, NetId};
use gummiband_core::geom::Point;

use crate::read::KicadBoard;
use crate::sexpr::quote;

#[derive(Debug, Clone)]
pub enum Item {
    Track {
        a: Point,
        b: Point,
        width: f64,
        layer: LayerId,
        net: NetId,
    },
    Arc {
        a: Point,
        mid: Point,
        b: Point,
        width: f64,
        layer: LayerId,
        net: NetId,
    },
    Via {
        at: Point,
        diameter: f64,
        drill: f64,
        net: NetId,
    },
    /// A KiCad length tuning pattern (`generated` group of type `tuning_pattern`) over items of
    /// the same list, by their index.
    Pattern(Box<PatternItem>),
}

#[derive(Debug, Clone)]
pub struct PatternItem {
    pub layer: LayerId,
    /// The net (P of a pair).
    pub net: NetId,
    /// The base line the meander replaces, and that of the N track for a pair.
    pub base: (Point, Point),
    pub base_coupled: Option<(Point, Point)>,
    pub rounded: bool,
    pub width: f64,
    pub gap: f64,
    pub amplitude: f64,
    pub spacing: f64,
    pub target: f64,
    /// Length of the net with the meander.
    pub length: f64,
    pub members: Vec<usize>,
}

#[derive(Debug, Clone, Default)]
pub struct StripOptions {
    /// Remove unlocked tracks, arcs and vias.
    pub unlocked: bool,
    /// Remove locked ones too.
    pub locked: bool,
    /// Remove copper zones (not keepouts) on these layers.
    pub zone_layers: Vec<String>,
    /// Remove the unlocked tracks, arcs and vias of these nets in any case.
    pub nets: Vec<String>,
    /// Remove these items, by their place in the file.
    pub items: Vec<u32>,
}

/// Byte ranges of root items removed by `opts`.
pub fn strip_ranges(kb: &KicadBoard, opts: &StripOptions) -> Vec<Range<usize>> {
    let mut out = Vec::new();
    for (i, n) in kb.root.args().iter().enumerate() {
        match n.head() {
            Some("segment") | Some("arc") | Some("via") => {
                let locked = n.flag("locked");
                let net_listed = !locked && !opts.nets.is_empty() && {
                    let net = n.child("net").and_then(|c| c.args().first());
                    let name = net.and_then(|x| x.as_str()).map(str::to_string).or_else(|| {
                        let id = net.and_then(|x| x.as_f64())? as i64;
                        let table = kb.net_numbers.as_ref()?;
                        table.iter().find(|(_, v)| **v == id).map(|(k, _)| k.clone())
                    });
                    name.is_some_and(|x| opts.nets.contains(&x))
                };
                if (locked && opts.locked)
                    || (!locked && opts.unlocked)
                    || net_listed
                    || opts.items.contains(&(i as u32))
                {
                    out.push(n.span());
                }
            }
            Some("zone") if !opts.zone_layers.is_empty() && n.child("keepout").is_none() => {
                let mut layers: Vec<&str> = Vec::new();
                for h in ["layer", "layers"] {
                    if let Some(l) = n.child(h) {
                        layers.extend(l.args().iter().filter_map(|x| x.as_str()));
                    }
                }
                if !layers.is_empty() && layers.iter().all(|l| opts.zone_layers.iter().any(|z| z == l)) {
                    out.push(n.span());
                }
            }
            _ => {}
        }
    }
    out
}

/// New file text: original minus `remove`, plus `items` before the closing parenthesis.
pub fn patch(kb: &KicadBoard, remove: &[Range<usize>], items: &[Item]) -> String {
    let text = &kb.text;
    let mut ranges: Vec<Range<usize>> = remove.to_vec();
    ranges.sort_by_key(|r| r.start);
    let mut out = String::with_capacity(text.len() + items.len() * 200);
    let mut pos = 0;
    for r in ranges {
        if r.start < pos {
            continue;
        }
        out.push_str(&text[pos..r.start]);
        // swallow the line break and indentation in front of the item
        while out.ends_with('\t') || out.ends_with(' ') {
            out.pop();
        }
        if out.ends_with('\n') {
            out.pop();
        }
        pos = r.end;
    }
    let close = kb.root.span().end - 1;
    out.push_str(&text[pos..close]);
    while out.ends_with(char::is_whitespace) {
        out.pop();
    }
    out.push('\n');
    let n_items = items.iter().filter(|it| !matches!(it, Item::Pattern(_))).count();
    let mut ids = UuidGen::new(text.len() as u64 ^ n_items as u64);
    let mut uuids: Vec<String> = Vec::with_capacity(items.len());
    for it in items {
        if matches!(it, Item::Pattern(_)) || no_length(it) {
            uuids.push(String::new());
            continue;
        }
        let id = ids.next();
        write_item(kb, it, &id, &mut out);
        uuids.push(id);
    }
    for it in items {
        if let Item::Pattern(p) = it {
            write_pattern(kb, p, &ids.next(), &uuids, &mut out);
        }
    }
    out.push_str(")\n");
    out
}

/// A tuning pattern as KiCad writes one: its properties in alphabetical order, the members by
/// UUID, sorted.
fn write_pattern(kb: &KicadBoard, p: &PatternItem, id: &str, uuids: &[String], out: &mut String) {
    let xy = |q: Point| format!("(xy {} {})", f(q.x), f(q.y));
    let yn = |b: bool| if b { "yes" } else { "no" };
    let mut members: Vec<&str> = p
        .members
        .iter()
        .filter_map(|&i| uuids.get(i))
        .filter(|u| !u.is_empty())
        .map(String::as_str)
        .collect();
    members.sort();
    let mut props = vec![format!("(base_line (pts {} {}))", xy(p.base.0), xy(p.base.1))];
    if let Some((a, b)) = p.base_coupled {
        props.push(format!("(base_line_coupled (pts {} {}))", xy(a), xy(b)));
    }
    let status = if p.length < p.target - 0.05 {
        "too_short"
    } else if p.length > p.target + 0.05 {
        "too_long"
    } else {
        "tuned"
    };
    props.extend([
        "(corner_radius_percent 100)".to_string(),
        format!("(end {})", xy(p.base.1)),
        "(initial_side \"default\")".to_string(),
        "(is_time_domain no)".to_string(),
        format!("(last_diff_pair_gap {})", f(p.gap)),
        format!("(last_netname {})", quote(&kb.board.nets[p.net as usize].name)),
        format!("(last_status \"{status}\")"),
        format!("(last_track_width {})", f(p.width)),
        format!("(last_tuning_length {})", f(p.length)),
        format!("(max_amplitude {})", f(p.amplitude)),
        format!("(min_amplitude {})", f(p.spacing.min(p.amplitude))),
        format!("(min_spacing {})", f(p.spacing)),
        format!("(origin {})", xy(p.base.0)),
        "(override_custom_rules yes)".to_string(),
        format!("(rounded {})", yn(p.rounded)),
        "(single_sided no)".to_string(),
        "(target_delay 0)".to_string(),
        "(target_delay_max 0)".to_string(),
        "(target_delay_min 0)".to_string(),
        format!("(target_length {})", f(p.target)),
        format!("(target_length_max {})", f(p.target + 0.05)),
        format!("(target_length_min {})", f(p.target - 0.05)),
        "(target_skew 0)".to_string(),
        "(target_skew_max 0)".to_string(),
        "(target_skew_min 0)".to_string(),
        format!(
            "(tuning_mode \"{}\")",
            if p.base_coupled.is_some() {
                "diff_pair"
            } else {
                "single"
            }
        ),
        "(update_order 0)".to_string(),
    ]);
    let _ = writeln!(
        out,
        "\t(generated\n\t\t(uuid \"{id}\")\n\t\t(type tuning_pattern)\n\t\t(name \"Tuning Pattern\")\n\t\t(layer {})\n\t\t{}\n\t\t(members {})\n\t)",
        quote(&kb.layer_names[p.layer as usize]),
        props.join("\n\t\t"),
        members.iter().map(|m| format!("\"{m}\"")).collect::<Vec<_>>().join(" ")
    );
}

/// A track or arc whose ends fall on the same nanometre of the file. It has no copper of its own
/// (what ends there meets at that point anyway), and KiCad's DRC takes such a dot for a crossing
/// of the tracks around it, so it is not written.
fn no_length(it: &Item) -> bool {
    let nm = |p: Point| ((p.x * 1e6).round(), (p.y * 1e6).round());
    match *it {
        Item::Track { a, b, .. } => nm(a) == nm(b),
        Item::Arc { a, mid, b, .. } => nm(a) == nm(b) && nm(a) == nm(mid),
        _ => false,
    }
}

fn net_ref(kb: &KicadBoard, net: NetId) -> String {
    let name = &kb.board.nets[net as usize].name;
    match &kb.net_numbers {
        Some(t) => format!("(net {})", t.get(name).copied().unwrap_or(0)),
        None => format!("(net {})", quote(name)),
    }
}

fn f(v: f64) -> String {
    // KiCad works in nanometres; six decimals in mm.
    let s = format!("{:.6}", (v * 1e6).round() / 1e6);
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.to_string() }
}

fn write_item(kb: &KicadBoard, it: &Item, id: &str, out: &mut String) {
    let layer = |l: LayerId| quote(&kb.layer_names[l as usize]);
    match *it {
        Item::Track {
            a,
            b,
            width,
            layer: l,
            net,
        } => {
            let _ = writeln!(
                out,
                "\t(segment\n\t\t(start {} {})\n\t\t(end {} {})\n\t\t(width {})\n\t\t(layer {})\n\t\t{}\n\t\t(uuid \"{}\")\n\t)",
                f(a.x),
                f(a.y),
                f(b.x),
                f(b.y),
                f(width),
                layer(l),
                net_ref(kb, net),
                id
            );
        }
        Item::Arc {
            a,
            mid,
            b,
            width,
            layer: l,
            net,
        } => {
            let _ = writeln!(
                out,
                "\t(arc\n\t\t(start {} {})\n\t\t(mid {} {})\n\t\t(end {} {})\n\t\t(width {})\n\t\t(layer {})\n\t\t{}\n\t\t(uuid \"{}\")\n\t)",
                f(a.x),
                f(a.y),
                f(mid.x),
                f(mid.y),
                f(b.x),
                f(b.y),
                f(width),
                layer(l),
                net_ref(kb, net),
                id
            );
        }
        Item::Pattern(_) => {}
        Item::Via {
            at,
            diameter,
            drill,
            net,
        } => {
            let last = kb.layer_names.len() as LayerId - 1;
            let _ = writeln!(
                out,
                "\t(via\n\t\t(at {} {})\n\t\t(size {})\n\t\t(drill {})\n\t\t(layers {} {})\n\t\t{}\n\t\t(uuid \"{}\")\n\t)",
                f(at.x),
                f(at.y),
                f(diameter),
                f(drill),
                layer(0),
                layer(last),
                net_ref(kb, net),
                id
            );
        }
    }
}

/// Deterministic version 4 style UUIDs (splitmix64).
pub(crate) struct UuidGen(u64);

impl UuidGen {
    pub(crate) fn new(seed: u64) -> Self {
        UuidGen(seed ^ 0x9e37_79b9_7f4a_7c15)
    }
    fn word(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    pub(crate) fn next(&mut self) -> String {
        let a = self.word();
        let b = self.word();
        format!(
            "{:08x}-{:04x}-4{:03x}-{:04x}-{:012x}",
            a >> 32,
            (a >> 16) & 0xffff,
            a & 0xfff,
            ((b >> 48) & 0x3fff) | 0x8000,
            b & 0xffff_ffff_ffff
        )
    }
}

/// Shift footprints (by reference) in a board file: the footprint's own `(at x y [r])` changes,
/// pads and graphics are relative to it; its zones are in board coordinates and shift as well.
pub fn move_footprints(
    text: &str,
    offsets: &std::collections::HashMap<String, Point>,
    blocks: &[gummiband_core::board::Block],
) -> anyhow::Result<String> {
    // tracks and vias of a block go with it
    let mut copper: std::collections::HashMap<&str, Point> = std::collections::HashMap::new();
    for b in blocks {
        if let Some(d) = b.refs.iter().find_map(|r| offsets.get(r)) {
            copper.extend(b.uuids.iter().map(|u| (u.as_str(), *d)));
        }
    }
    let root = crate::sexpr::parse(text).map_err(|e| anyhow::anyhow!("parse error: {e}"))?;
    let mut edits: Vec<(std::ops::Range<usize>, String)> = Vec::new();
    for fp in root
        .args()
        .iter()
        .filter(|n| matches!(n.head(), Some("footprint") | Some("module")))
    {
        let reference = fp
            .children("property")
            .find(|p| p.args().first().and_then(|x| x.as_str()) == Some("Reference"))
            .and_then(|p| p.args().get(1)?.as_str())
            .or_else(|| {
                fp.children("fp_text")
                    .find(|t| t.args().first().and_then(|x| x.as_str()) == Some("reference"))
                    .and_then(|t| t.args().get(1)?.as_str())
            });
        let (Some(reference), Some(at)) = (reference, fp.child("at")) else {
            continue;
        };
        let Some(d) = offsets.get(reference) else {
            continue;
        };
        let v = at.args().iter().filter_map(|x| x.as_f64()).collect::<Vec<_>>();
        if v.len() < 2 {
            continue;
        }
        let mut new = format!("(at {} {}", f(v[0] + d.x), f(v[1] + d.y));
        for r in &v[2..] {
            new.push_str(&format!(" {}", f(*r)));
        }
        new.push(')');
        edits.push((at.span(), new));
        for z in fp.children("zone") {
            for c in z.args() {
                match c.head() {
                    Some("polygon") => {
                        for xy in c.child("pts").into_iter().flat_map(|p| p.children("xy")) {
                            let v = xy.args().iter().filter_map(|x| x.as_f64()).collect::<Vec<_>>();
                            if v.len() >= 2 {
                                edits.push((xy.span(), format!("(xy {} {})", f(v[0] + d.x), f(v[1] + d.y))));
                            }
                        }
                    }
                    Some("filled_polygon") | Some("fill_segments") => edits.push((c.span(), String::new())),
                    _ => {}
                }
            }
        }
    }
    for n in root
        .args()
        .iter()
        .filter(|n| matches!(n.head(), Some("segment") | Some("arc") | Some("via")))
    {
        let id = n.child_str("uuid").or_else(|| n.child_str("tstamp"));
        let Some(d) = id.and_then(|i| copper.get(i)) else {
            continue;
        };
        for key in ["start", "mid", "end", "at"] {
            if let Some(p) = n.child(key) {
                let v = p.args().iter().filter_map(|x| x.as_f64()).collect::<Vec<_>>();
                if v.len() >= 2 {
                    edits.push((p.span(), format!("({key} {} {})", f(v[0] + d.x), f(v[1] + d.y))));
                }
            }
        }
    }
    edits.sort_by_key(|e| e.0.start);
    let mut out = String::with_capacity(text.len());
    let mut pos = 0;
    for (r, new) in edits {
        out.push_str(&text[pos..r.start]);
        out.push_str(&new);
        pos = r.end;
    }
    out.push_str(&text[pos..]);
    Ok(out)
}

/// Switch on KiCad's teardrops for all vias and connected copper pads of a board file. The
/// file only carries the settings: KiCad builds the shapes when it fills the zones, and leaves
/// out what does not fit.
pub fn enable_teardrops(text: &str) -> anyhow::Result<String> {
    const TD: &str = "(teardrops (best_length_ratio 0.5) (max_length 1) (best_width_ratio 1) \
                      (max_width 2) (curved_edges yes) (filter_ratio 0.9) (enabled yes) \
                      (allow_two_segments yes) (prefer_zone_connections yes))";
    let root = crate::sexpr::parse(text).map_err(|e| anyhow::anyhow!("parse error: {e}"))?;
    let mut at: Vec<usize> = Vec::new();
    for n in root.args() {
        match n.head() {
            Some("via") if n.child("teardrops").is_none() => at.push(n.span().end - 1),
            Some("footprint") | Some("module") => {
                for pad in n.children("pad") {
                    let kind = pad.args().get(1).and_then(|x| x.as_str());
                    if matches!(kind, Some("smd") | Some("thru_hole"))
                        && pad.child("net").is_some()
                        && pad.child("teardrops").is_none()
                    {
                        at.push(pad.span().end - 1);
                    }
                }
            }
            _ => {}
        }
    }
    at.sort_unstable();
    let mut out = String::with_capacity(text.len() + at.len() * (TD.len() + 1));
    let mut pos = 0;
    for a in at {
        out.push_str(&text[pos..a]);
        out.push(' ');
        out.push_str(TD);
        pos = a;
    }
    out.push_str(&text[pos..]);
    Ok(out)
}

/// Add items to a board as locked copper (a pre-routed layout the router keeps): the board text
/// with the items appended, each with `(locked yes)`.
pub fn add_locked(kb: &KicadBoard, items: &[Item]) -> String {
    add_locked_removing(kb, &[], items)
}

/// [`add_locked`] that also removes the root items at the byte ranges `remove` (see
/// [`strip_ranges`]).
pub fn add_locked_removing(kb: &KicadBoard, remove: &[Range<usize>], items: &[Item]) -> String {
    // patch writes the kept text, a line break, the new items and the closing paren
    let start = patch(kb, remove, &[]).len() - ")\n".len();
    let text = patch(kb, remove, items);
    if items.is_empty() {
        return text;
    }
    let (head, tail) = text.split_at(start);
    let tail = tail
        .replace("\t(segment\n", "\t(segment\n\t\t(locked yes)\n")
        .replace("\t(arc\n", "\t(arc\n\t\t(locked yes)\n")
        .replace("\t(via\n", "\t(via\n\t\t(locked yes)\n");
    format!("{head}{tail}")
}

/// Set a footprint's position (and rotation, if given) in a board file. Pads, texts and the
/// footprint's zones follow, see `place_write::Patch::move_footprint`.
pub fn place_footprint(text: &str, reference: &str, x: f64, y: f64, rotation: Option<f64>) -> anyhow::Result<String> {
    let root = crate::sexpr::parse(text).map_err(|e| anyhow::anyhow!("parse error: {e}"))?;
    for fp in root
        .args()
        .iter()
        .filter(|n| matches!(n.head(), Some("footprint") | Some("module")))
    {
        let name = fp
            .children("property")
            .find(|p| p.args().first().and_then(|x| x.as_str()) == Some("Reference"))
            .and_then(|p| p.args().get(1)?.as_str())
            .or_else(|| {
                fp.children("fp_text")
                    .find(|t| t.args().first().and_then(|x| x.as_str()) == Some("reference"))
                    .and_then(|t| t.args().get(1)?.as_str())
            });
        if name != Some(reference) {
            continue;
        }
        let Some(at) = fp.child("at") else { continue };
        let old = at.args().iter().filter_map(|x| x.as_f64()).nth(2).unwrap_or(0.0);
        let mut patch = crate::place_write::Patch::new(0);
        patch.move_footprint(fp, Point { x, y }, rotation.unwrap_or(old));
        return Ok(patch.apply(text, &root));
    }
    anyhow::bail!("no footprint {reference:?}")
}

/// A copper zone on one layer, appended to the board text. KiCad fills it (press B, or the
/// DRC with --refill-zones).
pub fn add_zone(
    kb: &KicadBoard,
    text: &str,
    net: NetId,
    layer: LayerId,
    outline: &[Point],
    priority: u32,
    ids: u64,
) -> String {
    add_zone_named(
        kb,
        text,
        &ZoneSpec {
            net,
            layer,
            priority,
            name: "",
            connect: PadConnect::Thermal,
            spoke: DEFAULT_SPOKE,
        },
        outline,
        ids,
    )
}

/// Thermal spoke width (KiCad's thermal_bridge_width) of the zones the tools write, mm.
pub const DEFAULT_SPOKE: f64 = 0.3;

/// How the pads of the zone's net join the zone.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PadConnect {
    /// Thermal reliefs on every pad.
    #[default]
    Thermal,
    /// Solid copper on every pad.
    Solid,
    /// Thermal reliefs on through hole pads, SMD pads solid (KiCad "thru_hole_only").
    ThermalTht,
}

impl PadConnect {
    /// From the MCP names "thermal", "solid" and "thermal_tht".
    pub fn parse(s: &str) -> Option<PadConnect> {
        match s {
            "thermal" => Some(PadConnect::Thermal),
            "solid" => Some(PadConnect::Solid),
            "thermal_tht" | "thru_hole_only" => Some(PadConnect::ThermalTht),
            _ => None,
        }
    }
}

/// What [`add_zone_named`] writes besides the outline.
pub struct ZoneSpec<'a> {
    pub net: NetId,
    pub layer: LayerId,
    pub priority: u32,
    /// Zone name; "gb:..." keeps the zone an obstacle for the router (see `read::GB_ZONE_PREFIX`).
    pub name: &'a str,
    /// How pads join the zone (power pours solid).
    pub connect: PadConnect,
    /// Thermal spoke width in mm.
    pub spoke: f64,
}

/// [`add_zone`] with a name and the pad connection.
pub fn add_zone_named(kb: &KicadBoard, text: &str, z: &ZoneSpec, outline: &[Point], ids: u64) -> String {
    let name = &kb.board.nets[z.net as usize].name;
    let net_item = match &kb.net_numbers {
        Some(t) => format!("(net {}) (net_name {})", t.get(name).copied().unwrap_or(0), quote(name)),
        None => format!("(net {})", quote(name)),
    };
    let mut gen_id = UuidGen::new(ids ^ text.len() as u64);
    let mut pts = String::new();
    for p in outline {
        pts.push_str(&format!("(xy {} {}) ", f(p.x), f(p.y)));
    }
    let zname = if z.name.is_empty() {
        String::new()
    } else {
        format!("(name {}) ", quote(z.name))
    };
    let connect = match z.connect {
        PadConnect::Solid => "(connect_pads yes (clearance 0.2))",
        PadConnect::ThermalTht => "(connect_pads thru_hole_only (clearance 0.2))",
        PadConnect::Thermal => "(connect_pads (clearance 0.2))",
    };
    let zone = format!(
        "\t(zone {net_item} (layer {}) (uuid \"{}\") {zname}(hatch edge 0.5) (priority {}) \
         {connect} (min_thickness 0.2) (fill yes (thermal_gap 0.3) \
         (thermal_bridge_width {})) (polygon (pts {pts})))\n",
        quote(&kb.layer_names[z.layer as usize]),
        gen_id.next(),
        z.priority,
        f(z.spoke.max(0.1)),
    );
    let end = text.rfind(')').unwrap_or(text.len());
    format!("{}{}{}", &text[..end], zone, &text[end..])
}

/// KiCad copper layer types.
pub const LAYER_TYPES: [&str; 4] = ["signal", "power", "mixed", "jumper"];

/// Set the KiCad type of copper layers (`(layers (4 "In1.Cu" signal) ...)`): `changes` holds
/// layer names with one of [`LAYER_TYPES`]. Returns the new text and per change the old type.
pub fn set_layer_types(text: &str, changes: &[(String, String)]) -> anyhow::Result<(String, Vec<String>)> {
    let root = crate::sexpr::parse(text).map_err(|e| anyhow::anyhow!("parse error: {e}"))?;
    let layers = root
        .child("layers")
        .ok_or_else(|| anyhow::anyhow!("the board has no layer table"))?;
    let mut edits: Vec<(std::ops::Range<usize>, String)> = Vec::new();
    let mut old = Vec::new();
    for (name, kind) in changes {
        if !LAYER_TYPES.contains(&kind.as_str()) {
            anyhow::bail!("layer type {kind:?}: use one of {}", LAYER_TYPES.join(", "));
        }
        let Some(l) = layers
            .args()
            .iter()
            .find(|l| l.items().get(1).and_then(|x| x.as_str()) == Some(name.as_str()))
        else {
            anyhow::bail!("no layer {name:?} in the layer table");
        };
        if !name.ends_with(".Cu") {
            anyhow::bail!("{name} is not a copper layer");
        }
        match l.items().get(2) {
            Some(t @ crate::sexpr::Node::Atom(..)) => {
                old.push(t.as_str().unwrap_or("signal").to_string());
                edits.push((t.span(), kind.clone()));
            }
            _ => {
                // no type given: KiCad reads signal; the type follows the name
                let at = l.items()[1].span().end;
                old.push("signal".into());
                edits.push((at..at, format!(" {kind}")));
            }
        }
    }
    edits.sort_by_key(|e| std::cmp::Reverse(e.0.start));
    let mut out = text.to_string();
    for (r, t) in edits {
        out.replace_range(r, &t);
    }
    Ok((out, old))
}

/// What [`set_locked`] changed.
#[derive(Debug, Clone, Default)]
pub struct LockCounts {
    /// References whose footprint changed its lock state.
    pub parts: Vec<String>,
    /// Tracks and arcs that changed.
    pub tracks: usize,
    pub vias: usize,
    /// Named references with no footprint on the board.
    pub unknown_parts: Vec<String>,
    /// Named nets without a track, arc or via on the board.
    pub nets_without_copper: Vec<String>,
}

/// Reference of a footprint node (KiCad 8+ property or the older fp_text).
fn footprint_reference(fp: &crate::sexpr::Node) -> Option<&str> {
    fp.children("property")
        .find(|p| p.args().first().and_then(|x| x.as_str()) == Some("Reference"))
        .and_then(|p| p.args().get(1)?.as_str())
        .or_else(|| {
            fp.children("fp_text")
                .find(|t| t.args().first().and_then(|x| x.as_str()) == Some("reference"))
                .and_then(|t| t.args().get(1)?.as_str())
        })
}

/// Net name of a track, arc or via (by name, or by number through the net table).
fn item_net_name(kb: &KicadBoard, n: &crate::sexpr::Node) -> Option<String> {
    let net = n.child("net").and_then(|c| c.args().first());
    net.and_then(|x| x.as_str())
        .filter(|s| s.parse::<f64>().is_err())
        .map(str::to_string)
        .or_else(|| {
            let id = net.and_then(|x| x.as_f64())? as i64;
            let table = kb.net_numbers.as_ref()?;
            table.iter().find(|(_, v)| **v == id).map(|(k, _)| k.clone())
        })
}

/// Lock (or with `lock` false unlock) footprints by reference and the tracks, arcs and vias of
/// nets, in place: only the `locked` flag of each item changes. `all_copper` takes every track,
/// arc and via of the board. Locked footprints stay where they are in place and route, locked
/// copper is kept by route (keep_locked) and never shoved.
pub fn set_locked(
    kb: &KicadBoard,
    parts: &[String],
    nets: &[String],
    all_copper: bool,
    lock: bool,
) -> (String, LockCounts) {
    use crate::sexpr::{Atom, Node};
    let text = &kb.text;
    let mut edits: Vec<(Range<usize>, String)> = Vec::new();
    let mut counts = LockCounts::default();
    let mut seen_parts: Vec<&str> = Vec::new();
    let mut seen_nets: Vec<String> = Vec::new();
    // the existing flag: `(locked yes|no)` or a bare `locked`
    let flag_node = |n: &Node| -> Option<Range<usize>> {
        n.args().iter().find_map(|a| match a {
            Node::Atom(Atom::Symbol(s), r) if s == "locked" => Some(r.clone()),
            Node::List(..) if a.head() == Some("locked") => Some(a.span()),
            _ => None,
        })
    };
    let toggle = |n: &Node, after: Range<usize>, edits: &mut Vec<(Range<usize>, String)>| -> bool {
        if n.flag("locked") == lock {
            return false;
        }
        match (flag_node(n), lock) {
            (Some(r), true) => edits.push((r, "(locked yes)".into())),
            (Some(r), false) => {
                // drop the flag with the white space in front of it
                let start = text[..r.start].trim_end().len();
                edits.push((start..r.end, String::new()));
            }
            (None, true) => {
                let rest = &text[after.end..];
                let sep = if rest.starts_with('\n') || rest.starts_with("\r\n") {
                    let line_start = text[..n.span().start].rfind('\n').map_or(0, |i| i + 1);
                    let indent: String = text[line_start..n.span().start]
                        .chars()
                        .take_while(|c| c.is_whitespace())
                        .collect();
                    format!("\n{indent}\t")
                } else {
                    " ".into()
                };
                edits.push((after.end..after.end, format!("{sep}(locked yes)")));
            }
            (None, false) => return false,
        }
        true
    };
    for n in kb.root.args() {
        match n.head() {
            Some("footprint") | Some("module") => {
                let Some(r) = footprint_reference(n) else {
                    continue;
                };
                if !parts.iter().any(|p| p == r) {
                    continue;
                }
                seen_parts.push(r);
                // after the library name, or the head when there is none
                let items = n.items();
                let after = match items.get(1) {
                    Some(a @ Node::Atom(Atom::Str(_), _)) => a.span(),
                    _ => items[0].span(),
                };
                if toggle(n, after, &mut edits) {
                    counts.parts.push(r.to_string());
                }
            }
            Some(h @ ("segment" | "arc" | "via")) => {
                let net = item_net_name(kb, n);
                let listed = net.as_ref().is_some_and(|x| nets.contains(x));
                if listed {
                    seen_nets.push(net.unwrap());
                }
                if !(listed || all_copper) {
                    continue;
                }
                if toggle(n, n.items()[0].span(), &mut edits) {
                    if h == "via" {
                        counts.vias += 1;
                    } else {
                        counts.tracks += 1;
                    }
                }
            }
            _ => {}
        }
    }
    counts.unknown_parts = parts
        .iter()
        .filter(|p| !seen_parts.contains(&p.as_str()))
        .cloned()
        .collect();
    counts.nets_without_copper = nets.iter().filter(|x| !seen_nets.contains(x)).cloned().collect();
    edits.sort_by_key(|e| std::cmp::Reverse(e.0.start));
    let mut out = text.to_string();
    for (r, t) in edits {
        out.replace_range(r, &t);
    }
    (out, counts)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layer_types_change_in_place() {
        let text = "(kicad_pcb (layers (0 \"F.Cu\" signal) (4 \"In1.Cu\" signal) (6 \"In2.Cu\" jumper) (2 \"B.Cu\" signal) (25 \"Edge.Cuts\" user))\n)\n";
        let (out, old) = set_layer_types(
            text,
            &[("In2.Cu".into(), "power".into()), ("In1.Cu".into(), "power".into())],
        )
        .unwrap();
        assert_eq!(old, vec!["jumper".to_string(), "signal".to_string()]);
        assert!(
            out.contains("(4 \"In1.Cu\" power) (6 \"In2.Cu\" power) (2 \"B.Cu\" signal)"),
            "{out}"
        );
        assert!(set_layer_types(text, &[("In3.Cu".into(), "power".into())]).is_err());
        assert!(set_layer_types(text, &[("In1.Cu".into(), "plane".into())]).is_err());
        assert!(set_layer_types(text, &[("Edge.Cuts".into(), "power".into())]).is_err());
    }

    const BOARD: &str = r#"(kicad_pcb
	(footprint "R"
		(at 10 20 90)
		(property "Reference" "R1" (at 0 -1 90))
		(pad "1" smd rect (at -1 0 90) (size 1 2) (layers "F.Cu"))
		(pad "2" smd rect (at 1 0 90) (size 1 2) (layers "F.Cu"))
	)
)
"#;

    #[test]
    fn rotating_a_footprint_turns_its_pads() {
        let out = place_footprint(BOARD, "R1", 12.5, 20.0, Some(-90.0)).unwrap();
        assert!(out.contains("(at 12.5 20 -90)"), "{out}");
        assert!(out.contains("(at -1 0 -90)"), "{out}");
        assert!(out.contains("(at 1 0 -90)"), "{out}");
        assert!(out.contains("(at 0 -1 -90)"), "{out}");
    }

    const KEEPOUT: &str = r#"(kicad_pcb
	(footprint "U"
		(at 10 20)
		(property "Reference" "U1" (at 0 0 0))
		(zone (layers "F.Cu") (keepout (tracks not_allowed))
			(polygon (pts (xy 8 18) (xy 12 18) (xy 12 22)))
		)
	)
)
"#;

    #[test]
    fn a_footprint_zone_goes_with_the_footprint() {
        let out = place_footprint(KEEPOUT, "U1", 15.0, 20.0, None).unwrap();
        assert!(out.contains("(xy 13 18) (xy 17 18) (xy 17 22)"), "{out}");
        let offsets = [("U1".to_string(), Point { x: 5.0, y: 0.0 })].into_iter().collect();
        assert_eq!(move_footprints(KEEPOUT, &offsets, &[]).unwrap(), out);
    }

    #[test]
    fn moving_without_rotation_keeps_all_angles() {
        let out = place_footprint(BOARD, "R1", 12.5, 20.0, None).unwrap();
        assert_eq!(out, BOARD.replace("(at 10 20 90)", "(at 12.5 20 90)"));
    }

    #[test]
    fn an_unknown_reference_is_an_error() {
        assert!(place_footprint(BOARD, "R2", 0.0, 0.0, None).is_err());
    }

    /// A track of no length (in the open end one can end up on a via, both ends on it) is left
    /// out: KiCad reported such a dot as a crossing with a track that passes the via.
    #[test]
    fn a_track_of_no_length_is_not_written() {
        let text = r#"(kicad_pcb (version 20240108) (generator "test")
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
  (net 0 "") (net 1 "A")
  (footprint "R" (at 10 20) (property "Reference" "R1" (at 0 0 0))
    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "A")))
)"#;
        let kb = crate::read::parse_board(text.to_string(), None, &crate::read::ReadOptions::default()).unwrap();
        let net = kb.board.net_id("A").unwrap();
        let p = gummiband_core::geom::pt(10.0, 20.0);
        let track = |b: Point| Item::Track {
            a: p,
            b,
            width: 0.25,
            layer: 0,
            net,
        };
        let items = [
            track(p),
            track(p + gummiband_core::geom::pt(2e-7, 0.0)),
            track(gummiband_core::geom::pt(12.0, 20.0)),
        ];
        let out = patch(&kb, &[], &items);
        assert_eq!(out.matches("(segment").count(), 1, "{out}");
        assert!(out.contains("(end 12 20)"), "{out}");
    }
    #[test]
    fn lock_and_unlock_in_place() {
        let text = "(kicad_pcb (version 20240108) (generator \"test\")\n\t(layers (0 \"F.Cu\" signal) (31 \"B.Cu\" signal))\n\t(net 0 \"\") (net 1 \"A\") (net 2 \"B\")\n\t(footprint \"R\"\n\t\t(layer \"F.Cu\")\n\t\t(at 10 20)\n\t\t(property \"Reference\" \"R1\" (at 0 0 0))\n\t\t(pad \"1\" smd rect (at 0 0) (size 1 1) (layers \"F.Cu\") (net 1 \"A\"))\n\t)\n\t(footprint \"R\" locked (at 20 20) (property \"Reference\" \"R2\" (at 0 0 0)))\n\t(segment (start 10 20) (end 12 20) (width 0.25) (layer \"F.Cu\") (net 1))\n\t(segment (start 12 20) (end 14 20) (width 0.25) (layer \"F.Cu\") (net 2))\n\t(via (at 12 20) (size 0.6) (drill 0.3) (layers \"F.Cu\" \"B.Cu\") (net 1))\n)\n";
        let read =
            |t: &str| crate::read::parse_board(t.to_string(), None, &crate::read::ReadOptions::default()).unwrap();
        let kb = read(text);
        let parts = vec!["R1".to_string(), "R2".to_string(), "R9".to_string()];
        let (out, c) = set_locked(&kb, &parts, &["A".into(), "C".into()], false, true);
        assert_eq!(c.parts, vec!["R1".to_string()], "R2 was locked already");
        assert_eq!((c.tracks, c.vias), (1, 1));
        assert_eq!(c.unknown_parts, vec!["R9".to_string()]);
        assert_eq!(c.nets_without_copper, vec!["C".to_string()]);
        assert!(out.contains("(footprint \"R\"\n\t\t(locked yes)\n\t\t(layer"), "{out}");
        assert!(out.contains("(segment (locked yes) (start 10 20)"), "{out}");
        assert!(out.contains("(segment (start 12 20)"), "net B stays unlocked: {out}");
        assert!(out.contains("(via (locked yes) (at 12 20)"), "{out}");
        let kb = read(&out);
        let (back, c) = set_locked(&kb, &parts, &[], true, false);
        assert_eq!(c.parts, vec!["R1".to_string(), "R2".to_string()]);
        assert_eq!((c.tracks, c.vias), (1, 1));
        assert!(!back.contains("locked"), "{back}");
        assert_eq!(
            back.replace("(footprint \"R\" (at 20 20)", "(footprint \"R\" locked (at 20 20)"),
            text
        );
    }
}
