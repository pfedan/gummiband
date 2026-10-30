//! Patch the placement of a KiCad board: move and rotate footprints, transform or replace zones,
//! remove root items and append new ones. Everything else is kept byte for byte.
//!
//! KiCad stores pad and text positions relative to the footprint in its unrotated frame, but pad
//! and text angles as absolute angles, and zones inside a footprint in board coordinates. A
//! footprint rotation therefore also adds the rotation delta to every pad and text angle and
//! transforms the footprint's own zones.

use std::fmt::Write as _;
use std::ops::Range;

use gummiband_core::geom::{Point, pt};

use crate::sexpr::{Node, quote};

/// KiCad rotates counter-clockwise on screen with y pointing down.
pub fn kicad_rot(p: Point, deg: f64) -> Point {
    p.rotate(-deg.to_radians())
}

/// Number in KiCad's style (nanometre grid, no trailing zeros).
pub fn fmt_mm(v: f64) -> String {
    let s = format!("{:.6}", (v * 1e6).round() / 1e6);
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.to_string() }
}

/// Angle in (-180, 180].
pub fn norm_angle(a: f64) -> f64 {
    let mut a = a % 360.0;
    if a <= -180.0 + 1e-9 {
        a += 360.0;
    }
    if a > 180.0 + 1e-9 {
        a -= 360.0;
    }
    if a.abs() < 1e-9 { 0.0 } else { a }
}

/// Text edits on the original file.
#[derive(Default)]
pub struct Patch {
    edits: Vec<(Range<usize>, String)>,
    removes: Vec<Range<usize>>,
    append: Vec<String>,
    uuid: u64,
}

fn xy_of(n: &Node) -> Option<Point> {
    let a = n.args();
    Some(pt(a.first()?.as_f64()?, a.get(1)?.as_f64()?))
}

impl Patch {
    pub fn new(seed: u64) -> Patch {
        Patch {
            uuid: seed ^ 0x5851_f42d_4c95_7f2d,
            ..Default::default()
        }
    }

    /// Deterministic version 4 style UUID.
    pub fn uuid(&mut self) -> String {
        let mut w = || {
            self.uuid = self.uuid.wrapping_add(0x9e37_79b9_7f4a_7c15);
            let mut z = self.uuid;
            z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
            z ^ (z >> 31)
        };
        let (a, b) = (w(), w());
        format!(
            "{:08x}-{:04x}-4{:03x}-{:04x}-{:012x}",
            a >> 32,
            (a >> 16) & 0xffff,
            a & 0xfff,
            ((b >> 48) & 0x3fff) | 0x8000,
            b & 0xffff_ffff_ffff
        )
    }

    fn replace(&mut self, n: &Node, s: String) {
        self.edits.push((n.span(), s));
    }

    /// Remove a node (with the line break and indentation in front of it).
    pub fn remove(&mut self, n: &Node) {
        self.removes.push(n.span());
    }

    /// Append an item (one or more lines, without the leading tab) before the closing parenthesis.
    pub fn append(&mut self, item: String) {
        self.append.push(item);
    }

    /// Add `delta` degrees to the angle of an `(at x y [angle])` node, keep its position.
    fn rotate_at(&mut self, at: &Node, delta: f64) {
        if delta.abs() < 1e-9 {
            return;
        }
        let a = at.args();
        let (Some(x), Some(y)) = (a.first().and_then(|v| v.as_f64()), a.get(1).and_then(|v| v.as_f64())) else {
            return;
        };
        let old = a.get(2).and_then(|v| v.as_f64()).unwrap_or(0.0);
        // keep flags such as `unlocked` that some versions put inside (at ...)
        let extra: Vec<&str> = a.iter().skip(3).filter_map(|v| v.as_str()).collect();
        let ang = norm_angle(old + delta);
        let mut s = format!("(at {} {}", fmt_mm(x), fmt_mm(y));
        if ang != 0.0 {
            let _ = write!(s, " {}", fmt_mm(ang));
        }
        for e in extra {
            let _ = write!(s, " {e}");
        }
        s.push(')');
        self.replace(at, s);
    }

    /// Move a footprint to `at` with rotation `rot` (degrees). Pads, texts and the footprint's
    /// zones follow.
    pub fn move_footprint(&mut self, fp: &Node, at: Point, rot: f64) {
        let Some(at_node) = fp.child("at") else { return };
        let a = at_node.args();
        let old_at = pt(
            a.first().and_then(|v| v.as_f64()).unwrap_or(0.0),
            a.get(1).and_then(|v| v.as_f64()).unwrap_or(0.0),
        );
        let old_rot = a.get(2).and_then(|v| v.as_f64()).unwrap_or(0.0);
        let rot = norm_angle(rot);
        let delta = rot - old_rot;
        let mut s = format!("(at {} {}", fmt_mm(at.x), fmt_mm(at.y));
        if rot != 0.0 {
            let _ = write!(s, " {}", fmt_mm(rot));
        }
        s.push(')');
        self.replace(at_node, s);
        for c in fp.args() {
            match c.head() {
                Some("pad") | Some("property") | Some("fp_text") => {
                    if let Some(an) = c.child("at") {
                        self.rotate_at(an, delta);
                    }
                }
                Some("zone") => {
                    let f = |p: Point| at + kicad_rot(p - old_at, delta);
                    self.transform_zone(c, &f);
                }
                _ => {}
            }
        }
    }

    /// Transform every point of a zone outline; filled polygons are dropped (refill in KiCad).
    pub fn transform_zone(&mut self, z: &Node, f: &dyn Fn(Point) -> Point) {
        for c in z.args() {
            match c.head() {
                Some("polygon") => {
                    if let Some(pts) = c.child("pts") {
                        for xy in pts.children("xy") {
                            if let Some(p) = xy_of(xy) {
                                let q = f(p);
                                self.replace(xy, format!("(xy {} {})", fmt_mm(q.x), fmt_mm(q.y)));
                            }
                        }
                    }
                }
                Some("filled_polygon") | Some("fill_segments") => self.remove(c),
                _ => {}
            }
        }
    }

    /// Replace the outline of a zone by one polygon; filled polygons are dropped.
    pub fn replace_zone_outline(&mut self, z: &Node, outline: &[Point]) {
        let mut first = true;
        for c in z.args() {
            match c.head() {
                Some("polygon") if first => {
                    first = false;
                    self.replace(c, polygon_text(outline, 2));
                }
                Some("polygon") | Some("filled_polygon") | Some("fill_segments") => self.remove(c),
                _ => {}
            }
        }
    }

    /// Move a root text (gr_text): position through `f`, angle plus `delta`.
    pub fn transform_text(&mut self, t: &Node, f: &dyn Fn(Point) -> Point, delta: f64) {
        let Some(at) = t.child("at") else { return };
        let a = at.args();
        let (Some(x), Some(y)) = (a.first().and_then(|v| v.as_f64()), a.get(1).and_then(|v| v.as_f64())) else {
            return;
        };
        let q = f(pt(x, y));
        let ang = norm_angle(a.get(2).and_then(|v| v.as_f64()).unwrap_or(0.0) + delta);
        let mut s = format!("(at {} {}", fmt_mm(q.x), fmt_mm(q.y));
        if ang != 0.0 {
            let _ = write!(s, " {}", fmt_mm(ang));
        }
        s.push(')');
        self.replace(at, s);
    }

    /// New file text.
    pub fn apply(&self, text: &str, root: &Node) -> String {
        enum E<'a> {
            Rep(&'a str),
            Del,
        }
        let mut all: Vec<(Range<usize>, E)> = self.edits.iter().map(|(r, s)| (r.clone(), E::Rep(s))).collect();
        all.extend(self.removes.iter().map(|r| (r.clone(), E::Del)));
        all.sort_by_key(|(r, _)| (r.start, std::cmp::Reverse(r.end)));
        let mut out = String::with_capacity(text.len());
        let mut pos = 0;
        for (r, e) in all {
            if r.start < pos {
                continue; // nested in an earlier edit
            }
            out.push_str(&text[pos..r.start]);
            match e {
                E::Rep(s) => out.push_str(s),
                E::Del => {
                    while out.ends_with('\t') || out.ends_with(' ') {
                        out.pop();
                    }
                    if out.ends_with('\n') {
                        out.pop();
                    }
                }
            }
            pos = r.end;
        }
        let close = root.span().end - 1;
        out.push_str(&text[pos..close]);
        while out.ends_with(char::is_whitespace) {
            out.pop();
        }
        out.push('\n');
        for item in &self.append {
            for line in item.lines() {
                out.push('\t');
                out.push_str(line);
                out.push('\n');
            }
        }
        out.push_str(")\n");
        out
    }
}

/// `(polygon (pts (xy ..) ...))` with `indent` tabs before the inner lines.
pub fn polygon_text(pts: &[Point], indent: usize) -> String {
    let tabs = "\t".repeat(indent + 1);
    let mut s = format!("(polygon\n{tabs}(pts\n");
    for chunk in pts.chunks(6) {
        s.push_str(&tabs);
        s.push('\t');
        let items: Vec<String> = chunk
            .iter()
            .map(|p| format!("(xy {} {})", fmt_mm(p.x), fmt_mm(p.y)))
            .collect();
        s.push_str(&items.join(" "));
        s.push('\n');
    }
    let _ = write!(s, "{tabs})\n{})", "\t".repeat(indent));
    s
}

/// Edge.Cuts rectangle.
pub fn edge_rect(p: &mut Patch, a: Point, b: Point) -> String {
    format!(
        "(gr_rect\n\t(start {} {})\n\t(end {} {})\n\t(stroke\n\t\t(width 0.1)\n\t\t(type default)\n\t)\n\t(fill no)\n\t(layer \"Edge.Cuts\")\n\t(uuid \"{}\")\n)",
        fmt_mm(a.x),
        fmt_mm(a.y),
        fmt_mm(b.x),
        fmt_mm(b.y),
        p.uuid()
    )
}

/// Edge.Cuts circle.
pub fn edge_circle(p: &mut Patch, c: Point, r: f64) -> String {
    format!(
        "(gr_circle\n\t(center {} {})\n\t(end {} {})\n\t(stroke\n\t\t(width 0.1)\n\t\t(type default)\n\t)\n\t(fill no)\n\t(layer \"Edge.Cuts\")\n\t(uuid \"{}\")\n)",
        fmt_mm(c.x),
        fmt_mm(c.y),
        fmt_mm(c.x + r),
        fmt_mm(c.y),
        p.uuid()
    )
}

/// A copper zone of `net` on `layer` with the given outline, settings like the Lichtschnitzel
/// planes (unfilled; KiCad fills it).
pub fn plane_zone(p: &mut Patch, net: &str, layer: &str, name: &str, outline: &[Point], clearance: f64) -> String {
    format!(
        "(zone\n\t(net {})\n\t(layer {})\n\t(uuid \"{}\")\n\t(name {})\n\t(hatch edge 0.5)\n\t(priority 1)\n\t(connect_pads\n\t\t(clearance {})\n\t)\n\t(min_thickness 0.25)\n\t(fill yes\n\t\t(thermal_gap 0.3)\n\t\t(thermal_bridge_width 0.5)\n\t\t(island_removal_mode 0)\n\t)\n\t{}\n)",
        quote(net),
        quote(layer),
        p.uuid(),
        quote(name),
        fmt_mm(clearance),
        polygon_text(outline, 1)
    )
}
