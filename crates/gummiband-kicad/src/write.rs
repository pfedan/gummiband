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
    let mut ids = UuidGen::new(text.len() as u64 ^ items.len() as u64);
    for it in items {
        write_item(kb, it, &mut ids, &mut out);
    }
    out.push_str(")\n");
    out
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

fn write_item(kb: &KicadBoard, it: &Item, ids: &mut UuidGen, out: &mut String) {
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
                ids.next()
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
                ids.next()
            );
        }
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
                ids.next()
            );
        }
    }
}

/// Deterministic version 4 style UUIDs (splitmix64).
struct UuidGen(u64);

impl UuidGen {
    fn new(seed: u64) -> Self {
        UuidGen(seed ^ 0x9e37_79b9_7f4a_7c15)
    }
    fn word(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    fn next(&mut self) -> String {
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

/// Shift footprints (by reference) in a board file: only the footprint's own `(at x y [r])`
/// changes, pads and graphics are relative to it.
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
    let text = patch(kb, &[], items);
    let close = kb.root.span().end - 1;
    // patch keeps everything up to the root's closing paren, the new items follow it
    let start = text[..close.min(text.len())].rfind('\n').unwrap_or(0);
    let (head, tail) = text.split_at(start);
    let tail = tail
        .replace("\t(segment\n", "\t(segment\n\t\t(locked yes)\n")
        .replace("\t(arc\n", "\t(arc\n\t\t(locked yes)\n")
        .replace("\t(via\n", "\t(via\n\t\t(locked yes)\n");
    format!("{head}{tail}")
}

/// Set a footprint's position (and rotation, if given) in a board file.
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
        let old = at.args().iter().filter_map(|x| x.as_f64()).nth(2);
        let mut new = format!("(at {} {}", f(x), f(y));
        if let Some(r) = rotation.or(old) {
            new.push_str(&format!(" {}", f(r)));
        }
        new.push(')');
        let span = at.span();
        return Ok(format!("{}{}{}", &text[..span.start], new, &text[span.end..]));
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
    let name = &kb.board.nets[net as usize].name;
    let net_item = match &kb.net_numbers {
        Some(t) => format!("(net {}) (net_name {})", t.get(name).copied().unwrap_or(0), quote(name)),
        None => format!("(net {})", quote(name)),
    };
    let mut gen_id = UuidGen::new(ids ^ text.len() as u64);
    let mut pts = String::new();
    for p in outline {
        pts.push_str(&format!("(xy {} {}) ", f(p.x), f(p.y)));
    }
    let zone = format!(
        "\t(zone {net_item} (layer {}) (uuid \"{}\") (hatch edge 0.5) (priority {priority}) \
         (connect_pads (clearance 0.2)) (min_thickness 0.2) (fill yes (thermal_gap 0.3) \
         (thermal_bridge_width 0.3)) (polygon (pts {pts})))\n",
        quote(&kb.layer_names[layer as usize]),
        gen_id.next(),
    );
    let end = text.rfind(')').unwrap_or(text.len());
    format!("{}{}{}", &text[..end], zone, &text[end..])
}
