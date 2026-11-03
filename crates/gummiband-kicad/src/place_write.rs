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

use crate::sexpr::{Atom, Node, quote};

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

    /// Flip a footprint to the other side of the board and put it at `at` with rotation `rot`
    /// (degrees), as KiCad's flip does: the layers of the footprint and its items change side
    /// (`F.Cu` to `B.Cu`, `F.SilkS` to `B.SilkS`, ...), everything in the footprint frame is
    /// mirrored at its x axis (pads, graphics, texts, which also toggle `mirror`), relative pad
    /// and text angles change sign, and the footprint's own zones follow in board coordinates.
    /// Whether KiCad mirrors left-right or top-bottom only changes the rotation; with the
    /// rotation given here the result is the same.
    pub fn flip_footprint(&mut self, fp: &Node, at: Point, rot: f64) {
        let Some(at_node) = fp.child("at") else { return };
        let a = at_node.args();
        let old_at = pt(
            a.first().and_then(|v| v.as_f64()).unwrap_or(0.0),
            a.get(1).and_then(|v| v.as_f64()).unwrap_or(0.0),
        );
        let old_rot = a.get(2).and_then(|v| v.as_f64()).unwrap_or(0.0);
        let rot = norm_angle(rot);
        let mut s = format!("(at {} {}", fmt_mm(at.x), fmt_mm(at.y));
        if rot != 0.0 {
            let _ = write!(s, " {}", fmt_mm(rot));
        }
        s.push(')');
        self.replace(at_node, s);
        let board = move |p: Point| {
            let l = kicad_rot(p - old_at, -old_rot);
            at + kicad_rot(pt(l.x, -l.y), rot)
        };
        for c in fp.args() {
            match c.head() {
                Some("at") => {}
                Some("layer") | Some("layers") => self.flip_layer_node(c),
                Some("zone") => {
                    self.flip_layers_in(c);
                    self.transform_zone(c, &board);
                }
                Some("pad") | Some("property") | Some("fp_text") | Some("fp_text_box") => {
                    if let Some(an) = c.child("at") {
                        self.flip_at(an, old_rot, rot);
                    }
                    self.flip_local(c, true);
                }
                _ => self.flip_local(c, false),
            }
        }
    }

    /// `(at x y [angle])` of a pad or text in the footprint frame: y mirrored, the angle
    /// relative to the footprint (`old_rot` before, `rot` after) changes sign.
    fn flip_at(&mut self, at: &Node, old_rot: f64, rot: f64) {
        let a = at.args();
        let (Some(x), Some(y)) = (a.first().and_then(|v| v.as_f64()), a.get(1).and_then(|v| v.as_f64())) else {
            return;
        };
        let old = a.get(2).and_then(|v| v.as_f64()).unwrap_or(0.0);
        let extra: Vec<&str> = a.iter().skip(3).filter_map(|v| v.as_str()).collect();
        let ang = norm_angle(rot - (old - old_rot));
        let mut s = format!("(at {} {}", fmt_mm(x), fmt_mm(-y));
        if ang != 0.0 || a.len() > 2 {
            let _ = write!(s, " {}", fmt_mm(ang));
        }
        for e in extra {
            let _ = write!(s, " {e}");
        }
        s.push(')');
        self.replace(at, s);
    }

    /// Mirror an item in the footprint frame: points in y, layers to the other side, chamfered
    /// corners top to bottom, texts mirrored. `own_at`: its `(at ...)` is done already.
    fn flip_local(&mut self, n: &Node, own_at: bool) {
        for c in n.args() {
            match c.head() {
                Some("at") if own_at => {}
                Some("start") | Some("end") | Some("center") | Some("mid") | Some("xy") | Some("offset")
                | Some("at") => {
                    let a = c.args();
                    if let (Some(h), Some(x), Some(y)) = (
                        c.head(),
                        a.first().and_then(|v| v.as_f64()),
                        a.get(1).and_then(|v| v.as_f64()),
                    ) {
                        let mut s = format!("({h} {} {}", fmt_mm(x), fmt_mm(-y));
                        for e in a.iter().skip(2) {
                            s.push(' ');
                            s.push_str(&atom_text(e));
                        }
                        s.push(')');
                        self.replace(c, s);
                    }
                }
                Some("layer") | Some("layers") => self.flip_layer_node(c),
                Some("chamfer") => {
                    let mut s = String::from("(chamfer");
                    for e in c.args() {
                        let t = match e.as_str() {
                            Some("top_left") => "bottom_left".to_string(),
                            Some("bottom_left") => "top_left".to_string(),
                            Some("top_right") => "bottom_right".to_string(),
                            Some("bottom_right") => "top_right".to_string(),
                            _ => atom_text(e),
                        };
                        let _ = write!(s, " {t}");
                    }
                    s.push(')');
                    self.replace(c, s);
                }
                Some("effects") => self.mirror_text(c),
                Some("model") => {}
                Some(_) => self.flip_local(c, false),
                None => {}
            }
        }
    }

    /// Toggle `mirror` in the `justify` of a text's effects, left and right swap.
    fn mirror_text(&mut self, effects: &Node) {
        match effects.child("justify") {
            Some(j) => {
                // a mirrored text grows to the other side: left becomes right, as KiCad's
                // left-right flip does
                let mut items: Vec<String> = j
                    .args()
                    .iter()
                    .map(|e| match e.as_str() {
                        Some("left") => "right".to_string(),
                        Some("right") => "left".to_string(),
                        _ => atom_text(e),
                    })
                    .collect();
                match items.iter().position(|t| t == "mirror") {
                    Some(i) => {
                        items.remove(i);
                    }
                    None => items.push("mirror".into()),
                }
                if items.is_empty() {
                    self.remove(j);
                } else {
                    self.replace(j, format!("(justify {})", items.join(" ")));
                }
            }
            None => {
                let end = effects.span().end - 1;
                self.edits.push((end..end, " (justify mirror)".into()));
            }
        }
    }

    /// The layers of a zone to the other side, for a keepout that follows a flipped part.
    pub fn flip_zone_layers(&mut self, z: &Node) {
        self.flip_layers_in(z);
    }

    /// The layers of a zone to the other side.
    fn flip_layers_in(&mut self, n: &Node) {
        for c in n.args() {
            if matches!(c.head(), Some("layer") | Some("layers")) {
                self.flip_layer_node(c);
            }
        }
    }

    fn flip_layer_node(&mut self, c: &Node) {
        let Some(h) = c.head() else { return };
        let mut changed = false;
        let mut s = format!("({h}");
        for e in c.args() {
            let t = match e {
                Node::Atom(Atom::Str(v), _) => {
                    let w = other_side(v);
                    changed |= w != *v;
                    quote(&w)
                }
                _ => atom_text(e),
            };
            let _ = write!(s, " {t}");
        }
        s.push(')');
        if changed {
            self.replace(c, s);
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

/// A layer of the other side: `F.SilkS` to `B.SilkS` and back; inner layers, `*.Cu`, `F&B.Cu`
/// and user layers stay.
pub fn other_side(layer: &str) -> String {
    if let Some(r) = layer.strip_prefix("F.") {
        format!("B.{r}")
    } else if let Some(r) = layer.strip_prefix("B.") {
        format!("F.{r}")
    } else {
        layer.to_string()
    }
}

/// An atom as it stands in the file; a list as `()` (not expected where this is used).
fn atom_text(n: &Node) -> String {
    match n {
        Node::Atom(Atom::Str(v), _) => quote(v),
        Node::Atom(Atom::Symbol(v), _) => v.clone(),
        Node::List(..) => "()".into(),
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

/// A copper zone of `net` on `layer` with the given outline, settings like the LED controller
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

#[cfg(test)]
mod tests {
    use super::*;

    const FP: &str = "(kicad_pcb\n\
        \t(footprint \"Lib:SOT\" (layer \"F.Cu\") (at 10 20 90)\n\
        \t\t(property \"Reference\" \"U1\" (at 0 -2.5 90) (layer \"F.SilkS\")\n\
        \t\t\t(effects (font (size 1 1)) (justify left)))\n\
        \t\t(fp_rect (start -1 -2) (end 1 2) (layer \"F.CrtYd\"))\n\
        \t\t(pad \"1\" smd rect (at 1 0.5 90) (size 0.6 0.4) (layers \"F.Cu\" \"F.Paste\" \"F.Mask\"))\n\
        \t\t(pad \"2\" thru_hole circle (at -1 -0.5 90) (size 1.6 1.6) (drill 0.8) (layers \"*.Cu\" \"*.Mask\"))\n\
        \t)\n\
        )\n";

    #[test]
    fn a_flip_mirrors_the_footprint_frame_and_swaps_the_layers() {
        let root = crate::sexpr::parse(FP).unwrap();
        let fp = root.children("footprint").next().unwrap();
        let mut patch = Patch::new(0);
        patch.flip_footprint(fp, pt(30.0, 40.0), 0.0);
        let out = patch.apply(FP, &root);
        let flat = out.split_whitespace().collect::<Vec<_>>().join(" ");
        for want in [
            "(layer \"B.Cu\") (at 30 40)",
            // local y mirrored; the pads turned with the part (90 at 90) keep their place on it
            "(pad \"1\" smd rect (at 1 -0.5 0)",
            "(layers \"B.Cu\" \"B.Paste\" \"B.Mask\")",
            "(pad \"2\" thru_hole circle (at -1 0.5 0)",
            "(layers \"*.Cu\" \"*.Mask\")",
            "(fp_rect (start -1 2) (end 1 -2) (layer \"B.CrtYd\"))",
            "(layer \"B.SilkS\")",
            "(justify right mirror)",
        ] {
            assert!(flat.contains(want), "{want} missing in\n{out}");
        }
        assert!(!flat.contains("F."), "{out}");
    }
}
