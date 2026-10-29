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
}

/// Byte ranges of root items removed by `opts`.
pub fn strip_ranges(kb: &KicadBoard, opts: &StripOptions) -> Vec<Range<usize>> {
    let mut out = Vec::new();
    for n in kb.root.args() {
        match n.head() {
            Some("segment") | Some("arc") | Some("via") => {
                let locked = n.flag("locked");
                if (locked && opts.locked) || (!locked && opts.unlocked) {
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
                if !layers.is_empty()
                    && layers
                        .iter()
                        .all(|l| opts.zone_layers.iter().any(|z| z == l))
                {
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
