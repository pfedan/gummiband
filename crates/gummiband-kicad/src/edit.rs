//! Edits of a board that the placement and routing writers do not make: remove zones, copper
//! and footprints, add a mounting hole, and place the reference texts on the silk screen clear
//! of pads, other silk and the board edge.

use gummiband_core::board::ObstacleKind;
use gummiband_core::geom::{Point, Rect, pt};
use gummiband_core::math;

use crate::place_write::{Patch, fmt_mm, kicad_rot, norm_angle};
use crate::power::glob;
use crate::read::KicadBoard;
use crate::sexpr::Node;
use crate::write::{footprint_reference, item_net_name};

// ------------------------------------------------------------------------------ remove

/// Zones to remove: every given field must match (name and net as globs with `*` and `?`).
#[derive(Debug, Clone, Default)]
pub struct ZoneFilter {
    pub name: Option<String>,
    pub net: Option<String>,
    pub layer: Option<String>,
}

#[derive(Debug, Clone, Default)]
pub struct RemoveSpec {
    pub zones: Vec<ZoneFilter>,
    /// Nets (globs) whose tracks, arcs and vias go.
    pub nets: Vec<String>,
    /// Copper of any net within this box goes (with `nets`: of those nets only).
    pub area: Option<Rect>,
    /// Only on these layers (tracks; a via goes if it touches one of them).
    pub layers: Vec<String>,
    /// "tracks", "vias" or "all" (default).
    pub copper: Option<String>,
    /// Some(true): only locked copper, Some(false): only unlocked, None: both.
    pub locked: Option<bool>,
    /// Footprints by reference.
    pub parts: Vec<String>,
}

#[derive(Debug, Clone, Default)]
pub struct RemoveCounts {
    /// The zones removed: "name (net on layer)".
    pub zones: Vec<String>,
    pub tracks: usize,
    pub vias: usize,
    pub parts: Vec<String>,
    /// Removed footprints that had pads on a net (the netlist still has them).
    pub parts_with_nets: Vec<String>,
    pub unknown_parts: Vec<String>,
}

fn xy(n: &Node, head: &str) -> Option<Point> {
    let a = n.child(head)?.args();
    Some(pt(a.first()?.as_f64()?, a.get(1)?.as_f64()?))
}

fn layers_of(n: &Node) -> Vec<String> {
    let mut out: Vec<String> = n
        .child("layer")
        .and_then(|l| l.args().first())
        .and_then(|x| x.as_str())
        .map(|s| vec![s.to_string()])
        .unwrap_or_default();
    if let Some(ls) = n.child("layers") {
        out.extend(ls.args().iter().filter_map(|x| x.as_str()).map(str::to_string));
    }
    out
}

/// A layer list of a via (`F.Cu B.Cu`: a through via) touches `layer` if it lies between them.
fn via_touches(kb: &KicadBoard, via_layers: &[String], layer: &str) -> bool {
    let idx = |l: &str| kb.layer_names.iter().position(|x| x == l);
    let (Some(a), Some(b)) = (
        via_layers.first().and_then(|l| idx(l)),
        via_layers.last().and_then(|l| idx(l)),
    ) else {
        return true;
    };
    idx(layer).is_some_and(|i| i >= a.min(b) && i <= a.max(b))
}

fn zone_net(kb: &KicadBoard, z: &Node) -> String {
    z.child_str("net_name")
        .map(str::to_string)
        .or_else(|| item_net_name(kb, z))
        .unwrap_or_default()
}

/// Remove what `spec` names; returns the new text and what went.
pub fn remove_items(kb: &KicadBoard, spec: &RemoveSpec) -> (String, RemoveCounts) {
    let mut patch = Patch::new(0);
    let mut c = RemoveCounts::default();
    let copper_selected = !spec.nets.is_empty() || spec.area.is_some();
    let want_tracks = !matches!(spec.copper.as_deref(), Some("vias"));
    let want_vias = !matches!(spec.copper.as_deref(), Some("tracks"));
    for n in kb.root.args() {
        match n.head() {
            Some("zone") => {
                let name = n.child_str("name").unwrap_or("").to_string();
                let net = zone_net(kb, n);
                let layers = layers_of(n);
                let hit = spec.zones.iter().any(|f| {
                    f.name.as_ref().is_none_or(|p| glob(p, &name))
                        && f.net.as_ref().is_none_or(|p| glob(p, &net))
                        && f.layer.as_ref().is_none_or(|l| layers.iter().any(|x| x == l))
                        && (f.name.is_some() || f.net.is_some() || f.layer.is_some())
                });
                // a zone of a footprint is part of it, these are root zones only
                if hit {
                    c.zones.push(format!(
                        "{} ({} on {})",
                        if name.is_empty() { "unnamed" } else { &name },
                        if net.is_empty() { "no net" } else { &net },
                        layers.join(" ")
                    ));
                    patch.remove(n);
                }
            }
            Some(h @ ("segment" | "arc" | "via")) if copper_selected => {
                let via = h == "via";
                if (via && !want_vias) || (!via && !want_tracks) {
                    continue;
                }
                if spec.locked.is_some_and(|l| l != n.flag("locked")) {
                    continue;
                }
                let net = item_net_name(kb, n).unwrap_or_default();
                if !spec.nets.is_empty() && !spec.nets.iter().any(|p| glob(p, &net)) {
                    continue;
                }
                let layers = layers_of(n);
                if !spec.layers.is_empty()
                    && !spec.layers.iter().any(|l| {
                        if via {
                            via_touches(kb, &layers, l)
                        } else {
                            layers.iter().any(|x| x == l)
                        }
                    })
                {
                    continue;
                }
                if let Some(area) = &spec.area {
                    let inside = if via {
                        xy(n, "at").is_some_and(|p| area.contains(p))
                    } else {
                        let pts: Vec<Point> = ["start", "mid", "end"].iter().filter_map(|k| xy(n, k)).collect();
                        pts.windows(2).any(|w| area.touches_seg(w[0], w[1]))
                    };
                    if !inside {
                        continue;
                    }
                }
                if via {
                    c.vias += 1;
                } else {
                    c.tracks += 1;
                }
                patch.remove(n);
            }
            Some("footprint") | Some("module") => {
                let Some(r) = footprint_reference(n) else { continue };
                if !spec.parts.iter().any(|p| p == r) {
                    continue;
                }
                c.parts.push(r.to_string());
                if n.children("pad").any(|p| p.child("net").is_some()) {
                    c.parts_with_nets.push(r.to_string());
                }
                patch.remove(n);
            }
            _ => {}
        }
    }
    c.unknown_parts = spec.parts.iter().filter(|p| !c.parts.contains(p)).cloned().collect();
    (patch.apply(&kb.text, &kb.root), c)
}

// ------------------------------------------------------------------------------ mounting holes

/// Append a locked, plain non-plated mounting hole of `diameter` mm at x, y. Without a
/// reference it gets the next free `H<n>`. Returns the text and the reference.
pub fn add_mounting_hole(
    kb: &KicadBoard,
    x: f64,
    y: f64,
    diameter: f64,
    reference: Option<&str>,
) -> anyhow::Result<(String, String)> {
    if diameter <= 0.0 {
        anyhow::bail!("diameter must be positive");
    }
    let used: Vec<&str> = kb
        .root
        .args()
        .iter()
        .filter(|n| matches!(n.head(), Some("footprint") | Some("module")))
        .filter_map(footprint_reference)
        .collect();
    let reference = match reference {
        Some(r) if used.contains(&r) => anyhow::bail!("{r} is already on the board"),
        Some(r) => r.to_string(),
        None => (1..)
            .map(|k| format!("H{k}"))
            .find(|r| !used.contains(&r.as_str()))
            .unwrap(),
    };
    let text = crate::create::hole_text(diameter, &reference, (x, y), kb.text.len() as u64 ^ 0x4d48);
    let mut patch = Patch::new(0);
    let body: Vec<&str> = text.lines().map(|l| l.strip_prefix('\t').unwrap_or(l)).collect();
    patch.append(body.join("\n"));
    Ok((patch.apply(&kb.text, &kb.root), reference))
}

// ------------------------------------------------------------------------------ silk

#[derive(Debug, Clone)]
pub struct SilkOptions {
    /// Only these references (empty: all).
    pub parts: Vec<String>,
    /// Smallest text height a reference may shrink to, mm.
    pub min_size: f64,
    /// Room kept to pads, other silk and texts, mm.
    pub margin: f64,
    /// Room kept to the board edge, mm.
    pub edge_margin: f64,
    /// Hide a reference that finds no spot (else it stays where it is).
    pub hide_unplaced: bool,
    /// Farthest distance from the part's courtyard, mm.
    pub reach: f64,
}

impl Default for SilkOptions {
    fn default() -> Self {
        SilkOptions {
            parts: Vec::new(),
            min_size: 0.8,
            margin: 0.1,
            edge_margin: 0.3,
            hide_unplaced: true,
            reach: 2.0,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct SilkReport {
    /// References that sit clear where they were.
    pub kept: Vec<String>,
    pub moved: Vec<String>,
    /// Moved with a smaller text: (reference, new height).
    pub shrunk: Vec<(String, f64)>,
    pub hidden: Vec<String>,
    /// No spot and not hidden: they stay where they were.
    pub left: Vec<String>,
}

/// One footprint's reference on a silk layer, with what the placement needs to know.
struct RefText<'a> {
    reference: String,
    fp_at: Point,
    fp_rot: f64,
    node: &'a Node,
    /// Text height and width of one character, stroke.
    size: (f64, f64),
    thickness: f64,
    angle: f64,
    court: Rect,
}

/// A text's box on the board: KiCad's stroke font is about 0.9 of the size wide per
/// character, centred on its position.
fn text_box(at: Point, chars: usize, size: (f64, f64), thickness: f64, angle: f64) -> Rect {
    let w = chars as f64 * size.1 * 0.9 + thickness;
    let h = size.0 + thickness;
    let a = norm_angle(angle).abs();
    let (w, h) = if (a - 90.0).abs() < 45.0 { (h, w) } else { (w, h) };
    Rect {
        min: pt(at.x - w / 2.0, at.y - h / 2.0),
        max: pt(at.x + w / 2.0, at.y + h / 2.0),
    }
}

fn rect_dist(a: &Rect, b: &Rect) -> f64 {
    let dx = (b.min.x - a.max.x).max(a.min.x - b.max.x).max(0.0);
    let dy = (b.min.y - a.max.y).max(a.min.y - b.max.y).max(0.0);
    math::hypot(dx, dy)
}

/// Silk graphics of a footprint on `layer` in board coordinates, as segments with their width.
fn silk_segments(fp: &Node, at: Point, rot: f64, layer: &str, out: &mut Vec<(Point, Point, f64)>) {
    for g in fp.args() {
        if g.child_str("layer") == Some(layer) {
            graphic_segments(g, at, rot, out);
        }
    }
}

/// One graphic (`fp_line`, `fp_arc`, `fp_rect`, `fp_circle`, `fp_poly`) of a footprint at `at`
/// with rotation `rot`, as segments in board coordinates with the stroke width.
fn graphic_segments(g: &Node, at: Point, rot: f64, out: &mut Vec<(Point, Point, f64)>) {
    let board = |p: Point| at + kicad_rot(p, rot);
    let Some(h) = g.head() else { return };
    let w = g
        .child("stroke")
        .and_then(|s| s.child("width"))
        .and_then(|x| x.args().first())
        .and_then(|x| x.as_f64())
        .or_else(|| g.child("width").and_then(|x| x.args().first()).and_then(|x| x.as_f64()))
        .unwrap_or(0.12);
    let mut push = |pts: &[Point]| {
        for s in pts.windows(2) {
            out.push((board(s[0]), board(s[1]), w));
        }
    };
    match h {
        "fp_line" | "fp_arc" => {
            let pts: Vec<Point> = ["start", "mid", "end"].iter().filter_map(|k| xy(g, k)).collect();
            push(&pts);
        }
        "fp_rect" => {
            if let (Some(a), Some(b)) = (xy(g, "start"), xy(g, "end")) {
                push(&[a, pt(b.x, a.y), b, pt(a.x, b.y), a]);
            }
        }
        "fp_circle" => {
            if let (Some(c), Some(e)) = (xy(g, "center"), xy(g, "end")) {
                let r = c.dist(e);
                let ring: Vec<Point> = (0..=16)
                    .map(|k| {
                        let t = k as f64 / 16.0 * std::f64::consts::TAU;
                        c + pt(r * math::cos(t), r * math::sin(t))
                    })
                    .collect();
                push(&ring);
            }
        }
        "fp_poly" => {
            let mut pts: Vec<Point> = g
                .child("pts")
                .map(|p| {
                    p.children("xy")
                        .filter_map(|q| {
                            let a = q.args();
                            Some(pt(a.first()?.as_f64()?, a.get(1)?.as_f64()?))
                        })
                        .collect()
                })
                .unwrap_or_default();
            if pts.len() > 2 {
                pts.push(pts[0]);
            }
            push(&pts);
        }
        _ => {}
    }
}

/// Place the reference texts of the footprints on the silk screen: each where it is if that
/// spot is clear, else on the nearest clear spot around its courtyard (above, below, left,
/// right, the corners, up to `reach` away), else smaller down to `min_size`, else hidden. Clear
/// means: off the pads of its side (KiCad's silk_over_copper), off the silk graphics of other
/// footprints and the texts placed before (silk_overlap), inside the board by `edge_margin`.
/// Larger parts choose first.
pub fn place_references(kb: &KicadBoard, o: &SilkOptions) -> (String, SilkReport) {
    let b = &kb.board;
    let last = b.layers.len().saturating_sub(1) as u8;
    let mut refs: Vec<RefText> = Vec::new();
    // silk graphics per side, by the footprint they belong to
    let mut graphics: [Vec<(String, Point, Point, f64)>; 2] = [Vec::new(), Vec::new()];
    // texts that stay as they are (other footprints' references outside `parts`, board texts)
    let mut fixed_texts: [Vec<Rect>; 2] = [Vec::new(), Vec::new()];
    for n in kb.root.args() {
        match n.head() {
            Some("footprint") | Some("module") => {
                let Some(r) = footprint_reference(n) else { continue };
                let Some(at_n) = n.child("at") else { continue };
                let a = at_n.args();
                let fp_at = pt(
                    a.first().and_then(|x| x.as_f64()).unwrap_or(0.0),
                    a.get(1).and_then(|x| x.as_f64()).unwrap_or(0.0),
                );
                let fp_rot = a.get(2).and_then(|x| x.as_f64()).unwrap_or(0.0);
                for (side, layer) in [(0, "F.SilkS"), (1, "B.SilkS")] {
                    let mut segs = Vec::new();
                    silk_segments(n, fp_at, fp_rot, layer, &mut segs);
                    graphics[side].extend(segs.into_iter().map(|(p, q, w)| (r.to_string(), p, q, w)));
                }
                let prop = n
                    .children("property")
                    .find(|p| p.args().first().and_then(|x| x.as_str()) == Some("Reference"))
                    .or_else(|| {
                        n.children("fp_text")
                            .find(|t| t.args().first().and_then(|x| x.as_str()) == Some("reference"))
                    });
                let Some(prop) = prop else { continue };
                let layer = prop.child_str("layer").unwrap_or("");
                if !layer.ends_with("SilkS")
                    || prop.flag("hide")
                    || prop.args().iter().any(|x| x.as_str() == Some("hide"))
                {
                    continue;
                }
                let font = prop.child("effects").and_then(|e| e.child("font"));
                let size = font
                    .and_then(|f| f.child("size"))
                    .map(|s| {
                        let a = s.args();
                        (
                            a.first().and_then(|x| x.as_f64()).unwrap_or(1.0),
                            a.get(1).and_then(|x| x.as_f64()).unwrap_or(1.0),
                        )
                    })
                    .unwrap_or((1.0, 1.0));
                let thickness = font
                    .and_then(|f| f.child("thickness"))
                    .and_then(|t| t.args().first())
                    .and_then(|x| x.as_f64())
                    .unwrap_or(0.15);
                let pa = prop.child("at").map(|x| x.args()).unwrap_or(&[]);
                let local = pt(
                    pa.first().and_then(|x| x.as_f64()).unwrap_or(0.0),
                    pa.get(1).and_then(|x| x.as_f64()).unwrap_or(0.0),
                );
                let angle = pa.get(2).and_then(|x| x.as_f64()).unwrap_or(0.0);
                let court = b.courtyards.get(r).copied().unwrap_or(Rect { min: fp_at, max: fp_at });
                let side = layer.starts_with("B.") as usize;
                let chosen = o.parts.is_empty() || o.parts.iter().any(|p| p == r);
                if !chosen {
                    fixed_texts[side].push(text_box(
                        fp_at + kicad_rot(local, fp_rot),
                        r.chars().count(),
                        size,
                        thickness,
                        angle,
                    ));
                    continue;
                }
                refs.push(RefText {
                    reference: r.to_string(),
                    fp_at,
                    fp_rot,
                    node: prop,
                    size,
                    thickness,
                    angle,
                    court,
                });
            }
            Some("gr_text") => {
                let layer = n.child_str("layer").unwrap_or("");
                if !layer.ends_with("SilkS") {
                    continue;
                }
                if let (Some(at), Some(t)) = (xy(n, "at"), n.args().first().and_then(|x| x.as_str())) {
                    fixed_texts[layer.starts_with("B.") as usize].push(text_box(
                        at,
                        t.chars().count(),
                        (1.0, 1.0),
                        0.15,
                        0.0,
                    ));
                }
            }
            _ => {}
        }
    }
    // pads (and holes) per side: what the solder mask opens
    let pads: [Vec<Rect>; 2] = [0u8, last].map(|l| {
        b.obstacles
            .iter()
            .filter(|x| x.kind == ObstacleKind::Pad && (x.layers.has(l) || x.hole.is_some()))
            .map(|x| x.shape.bbox())
            .collect()
    });
    let outline = b
        .outline
        .iter()
        .max_by(|p, q| p.signed_area().abs().total_cmp(&q.signed_area().abs()));
    let inside = |r: &Rect| -> bool {
        let Some(poly) = outline else { return true };
        let corners = [r.min, pt(r.max.x, r.min.y), r.max, pt(r.min.x, r.max.y)];
        corners
            .iter()
            .all(|&c| poly.contains(c) && poly.boundary_dist(c) >= o.edge_margin)
    };
    refs.sort_by(|a, b| {
        let area = |r: &Rect| (r.max.x - r.min.x) * (r.max.y - r.min.y);
        area(&b.court)
            .total_cmp(&area(&a.court))
            .then(a.reference.cmp(&b.reference))
    });
    let mut placed: [Vec<Rect>; 2] = fixed_texts;
    let mut rep = SilkReport::default();
    let mut patch = Patch::new(0);
    for t in &refs {
        let side = (t.node.child_str("layer").unwrap_or("").starts_with("B.")) as usize;
        let chars = t.reference.chars().count();
        let clear = |bx: &Rect, placed: &[Vec<Rect>; 2]| -> bool {
            inside(bx)
                && pads[side].iter().all(|p| rect_dist(p, bx) >= o.margin)
                && placed[side].iter().all(|p| rect_dist(p, bx) >= o.margin)
                && graphics[side]
                    .iter()
                    .filter(|(r, ..)| *r != t.reference)
                    .all(|(_, p, q, w)| bx.dist_seg(*p, *q) >= o.margin + w / 2.0)
        };
        let cur_local = t
            .node
            .child("at")
            .map(|x| {
                let a = x.args();
                pt(
                    a.first().and_then(|v| v.as_f64()).unwrap_or(0.0),
                    a.get(1).and_then(|v| v.as_f64()).unwrap_or(0.0),
                )
            })
            .unwrap_or(pt(0.0, 0.0));
        let cur = t.fp_at + kicad_rot(cur_local, t.fp_rot);
        let cur_box = text_box(cur, chars, t.size, t.thickness, t.angle);
        if clear(&cur_box, &placed) {
            placed[side].push(cur_box);
            rep.kept.push(t.reference.clone());
            continue;
        }
        // the sizes to try, the angle the text has (0 or 90 for a part standing upright)
        let mut sizes = vec![t.size];
        let mut s = t.size;
        while s.0 * 0.8 >= o.min_size - 1e-9 {
            s = (s.0 * 0.8, s.1 * 0.8);
            sizes.push(s);
        }
        if sizes.last().is_some_and(|l| l.0 > o.min_size + 1e-9) && t.size.0 > o.min_size {
            let k = o.min_size / t.size.0;
            sizes.push((t.size.0 * k, t.size.1 * k));
        }
        let angles = [t.angle, norm_angle(t.angle + 90.0)];
        let c = t.court;
        let mid = pt((c.min.x + c.max.x) / 2.0, (c.min.y + c.max.y) / 2.0);
        let mut found: Option<(Point, (f64, f64), f64)> = None;
        'search: for &size in &sizes {
            let thick = t.thickness * size.0 / t.size.0;
            for &ang in &angles {
                let probe = text_box(pt(0.0, 0.0), chars, size, thick, ang);
                let (hw, hh) = ((probe.max.x - probe.min.x) / 2.0, (probe.max.y - probe.min.y) / 2.0);
                let mut d = o.margin;
                while d <= o.reach + 1e-9 {
                    let cands = [
                        pt(mid.x, c.min.y - d - hh),
                        pt(mid.x, c.max.y + d + hh),
                        pt(c.min.x - d - hw, mid.y),
                        pt(c.max.x + d + hw, mid.y),
                        pt(c.min.x - d - hw, c.min.y - d - hh),
                        pt(c.max.x + d + hw, c.min.y - d - hh),
                        pt(c.min.x - d - hw, c.max.y + d + hh),
                        pt(c.max.x + d + hw, c.max.y + d + hh),
                    ];
                    for p in cands {
                        let bx = text_box(p, chars, size, thick, ang);
                        if clear(&bx, &placed) {
                            found = Some((p, size, ang));
                            placed[side].push(bx);
                            break 'search;
                        }
                    }
                    d += 0.25;
                }
            }
        }
        match found {
            Some((p, size, ang)) => {
                let local = kicad_rot(p - t.fp_at, -t.fp_rot);
                let mut at = format!("(at {} {}", fmt_mm(local.x), fmt_mm(local.y));
                if ang != 0.0 {
                    at += &format!(" {}", fmt_mm(ang));
                }
                at.push(')');
                if let Some(an) = t.node.child("at") {
                    patch.replace_node(an, at);
                }
                if (size.0 - t.size.0).abs() > 1e-9 {
                    let font = t.node.child("effects").and_then(|e| e.child("font"));
                    if let Some(sz) = font.and_then(|f| f.child("size")) {
                        patch.replace_node(sz, format!("(size {} {})", fmt_mm(size.0), fmt_mm(size.1)));
                    }
                    if let Some(th) = font.and_then(|f| f.child("thickness")) {
                        let thick = (t.thickness * size.0 / t.size.0).max(0.1);
                        patch.replace_node(th, format!("(thickness {})", fmt_mm(thick)));
                    }
                    rep.shrunk
                        .push((t.reference.clone(), (size.0 * 1000.0).round() / 1000.0));
                } else {
                    rep.moved.push(t.reference.clone());
                }
            }
            None if o.hide_unplaced => {
                match t.node.child("hide") {
                    Some(h) => patch.replace_node(h, "(hide yes)".into()),
                    None => {
                        let layer = t.node.child("layer");
                        let end = layer.map_or(t.node.span().end - 1, |l| l.span().end);
                        patch.insert_at(end, " (hide yes)".into());
                    }
                }
                rep.hidden.push(t.reference.clone());
            }
            None => {
                placed[side].push(cur_box);
                rep.left.push(t.reference.clone());
            }
        }
    }
    (patch.apply(&kb.text, &kb.root), rep)
}

/// Remove the silk graphics of footprints that lie outside the board outline or within
/// `margin` of it (a connector's body drawn over the edge): KiCad's silk_edge_clearance.
/// Returns the text and the references whose silk was trimmed.
pub fn trim_edge_silk(kb: &KicadBoard, margin: f64) -> (String, Vec<String>) {
    let b = &kb.board;
    let outline = b
        .outline
        .iter()
        .max_by(|p, q| p.signed_area().abs().total_cmp(&q.signed_area().abs()));
    let mut patch = Patch::new(0);
    let mut refs = Vec::new();
    let Some(poly) = outline else {
        return (kb.text.clone(), refs);
    };
    for n in kb.root.args() {
        if !matches!(n.head(), Some("footprint") | Some("module")) {
            continue;
        }
        let Some(r) = footprint_reference(n) else { continue };
        let Some(at_n) = n.child("at") else { continue };
        let a = at_n.args();
        let at = pt(
            a.first().and_then(|x| x.as_f64()).unwrap_or(0.0),
            a.get(1).and_then(|x| x.as_f64()).unwrap_or(0.0),
        );
        let rot = a.get(2).and_then(|x| x.as_f64()).unwrap_or(0.0);
        let mut any = false;
        for g in n.args() {
            let layer = g.child_str("layer").unwrap_or("");
            if !layer.ends_with("SilkS")
                || !g.head().is_some_and(|h| h.starts_with("fp_"))
                || g.head() == Some("fp_text")
            {
                continue;
            }
            let mut segs = Vec::new();
            graphic_segments(g, at, rot, &mut segs);
            let bad = segs.iter().any(|(p, q, w)| {
                let near = |c: Point| !poly.contains(c) || poly.boundary_dist(c) < margin + w / 2.0;
                near(*p)
                    || near(*q)
                    || poly
                        .edges()
                        .any(|(e0, e1)| gummiband_core::geom::seg_seg_dist(*p, *q, e0, e1) < margin + w / 2.0)
            });
            if bad {
                patch.remove(g);
                any = true;
            }
        }
        if any {
            refs.push(r.to_string());
        }
    }
    (patch.apply(&kb.text, &kb.root), refs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::read::{ReadOptions, parse_board};

    const BOARD: &str = "(kicad_pcb (version 20240108) (generator \"test\")
\t(layers (0 \"F.Cu\" signal) (31 \"B.Cu\" signal) (37 \"F.SilkS\" user) (44 \"Edge.Cuts\" user))
\t(net 0 \"\") (net 1 \"A\") (net 2 \"B\")
\t(gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) (type solid)) (fill none) (layer \"Edge.Cuts\"))
\t(footprint \"R\"
\t\t(layer \"F.Cu\")
\t\t(at 10 10)
\t\t(property \"Reference\" \"R1\" (at 0 0 0) (layer \"F.SilkS\") (effects (font (size 1 1) (thickness 0.15))))
\t\t(fp_line (start -1.5 -0.8) (end 1.5 -0.8) (stroke (width 0.12) (type solid)) (layer \"F.SilkS\"))
\t\t(fp_rect (start -1.6 -0.9) (end 1.6 0.9) (stroke (width 0.05) (type solid)) (fill none) (layer \"F.CrtYd\"))
\t\t(pad \"1\" smd rect (at -0.8 0) (size 0.8 1) (layers \"F.Cu\") (net 1 \"A\"))
\t\t(pad \"2\" smd rect (at 0.8 0) (size 0.8 1) (layers \"F.Cu\") (net 2 \"B\"))
\t)
\t(segment (start 9.2 10) (end 5 10) (width 0.25) (layer \"F.Cu\") (net 1))
\t(segment (locked yes) (start 10.8 10) (end 15 10) (width 0.25) (layer \"F.Cu\") (net 2))
\t(via (locked yes) (at 5 10) (size 0.6) (drill 0.3) (layers \"F.Cu\" \"B.Cu\") (net 1))
\t(via (at 15 10) (size 0.6) (drill 0.3) (layers \"F.Cu\" \"B.Cu\") (net 2))
\t(zone (net 1) (net_name \"A\") (layer \"F.Cu\") (name \"gb: pour A\") (polygon (pts (xy 1 1) (xy 4 1) (xy 4 4))))
\t(zone (net 2) (net_name \"B\") (layer \"B.Cu\") (polygon (pts (xy 1 1) (xy 4 1) (xy 4 4))))
)
";

    fn kb(t: &str) -> KicadBoard {
        parse_board(t.to_string(), None, &ReadOptions::default()).unwrap()
    }

    #[test]
    fn removes_zones_copper_and_parts() {
        let k = kb(BOARD);
        let (out, c) = remove_items(
            &k,
            &RemoveSpec {
                zones: vec![ZoneFilter {
                    name: Some("gb:*".into()),
                    ..Default::default()
                }],
                ..Default::default()
            },
        );
        assert_eq!(c.zones, vec!["gb: pour A (A on F.Cu)".to_string()]);
        assert_eq!(out.matches("(zone").count(), 1, "{out}");
        // locked vias of A within a box, nothing else
        let (out, c) = remove_items(
            &k,
            &RemoveSpec {
                area: Some(Rect {
                    min: pt(4.0, 9.0),
                    max: pt(6.0, 11.0),
                }),
                copper: Some("vias".into()),
                locked: Some(true),
                ..Default::default()
            },
        );
        assert_eq!((c.tracks, c.vias), (0, 1));
        assert!(!out.contains("(at 5 10)") && out.contains("(at 15 10)"), "{out}");
        // all copper of B, and the part
        let (out, c) = remove_items(
            &k,
            &RemoveSpec {
                nets: vec!["B".into()],
                parts: vec!["R1".into(), "R7".into()],
                ..Default::default()
            },
        );
        assert_eq!((c.tracks, c.vias), (1, 1));
        assert_eq!(c.parts, vec!["R1".to_string()]);
        assert_eq!(c.parts_with_nets, vec!["R1".to_string()]);
        assert_eq!(c.unknown_parts, vec!["R7".to_string()]);
        assert!(!out.contains("(footprint") && out.contains("(start 9.2 10)"), "{out}");
        assert!(crate::sexpr::parse(&out).is_ok());
    }

    #[test]
    fn adds_a_mounting_hole() {
        let k = kb(BOARD);
        let (out, r) = add_mounting_hole(&k, 25.0, 15.0, 2.7, None).unwrap();
        assert_eq!(r, "H1");
        let k2 = kb(&out);
        assert!(k2.board.courtyards.contains_key("H1"), "{out}");
        assert!(out.contains("(at 25 15)") && out.contains("(drill 2.7)"), "{out}");
        assert!(add_mounting_hole(&k2, 5.0, 5.0, 3.2, Some("H1")).is_err());
        let (_, r) = add_mounting_hole(&k2, 5.0, 5.0, 3.2, None).unwrap();
        assert_eq!(r, "H2");
    }

    #[test]
    fn moves_a_reference_off_its_pads() {
        let k = kb(BOARD);
        let (out, rep) = place_references(&k, &SilkOptions::default());
        assert_eq!(rep.moved, vec!["R1".to_string()], "{rep:?}");
        // above or below the courtyard, clear of the silk line at y -0.8
        let k2 = kb(&out);
        let fp = k2.root.args().iter().find(|n| n.head() == Some("footprint")).unwrap();
        let prop = fp.child("property").unwrap();
        let at = xy(prop, "at").unwrap();
        assert!(at.y.abs() >= 0.9 + 0.5, "{at:?}");
        // a second run keeps it
        let (_, rep) = place_references(&k2, &SilkOptions::default());
        assert_eq!(rep.kept, vec!["R1".to_string()], "{rep:?}");
    }

    #[test]
    fn hides_what_does_not_fit() {
        // a board barely larger than the part: no room for the text anywhere
        let t = BOARD
            .replace("(end 30 20)", "(end 11.9 11.2)")
            .replace("(start 0 0)", "(start 8.1 8.8)");
        let k = kb(&t);
        let (out, rep) = place_references(&k, &SilkOptions::default());
        assert_eq!(rep.hidden, vec!["R1".to_string()], "{rep:?}");
        assert!(out.contains("(layer \"F.SilkS\") (hide yes)"), "{out}");
    }
}
