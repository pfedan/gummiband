//! Footprints of a KiCad board as placement objects: reference, pose, courtyard as rectangles in
//! the footprint frame, pads with nets. Parsed from the s-expression tree of the board.

use std::collections::HashMap;

use gummiband_core::geom::{Point, pt};
use gummiband_kicad::place_write::kicad_rot;
use gummiband_kicad::sexpr::Node;

/// Axis aligned rectangle in some frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct R {
    pub x0: f64,
    pub y0: f64,
    pub x1: f64,
    pub y1: f64,
}

impl R {
    pub fn w(&self) -> f64 {
        self.x1 - self.x0
    }
    pub fn h(&self) -> f64 {
        self.y1 - self.y0
    }
    pub fn area(&self) -> f64 {
        self.w().max(0.0) * self.h().max(0.0)
    }
    pub fn grow(&self, d: f64) -> R {
        R {
            x0: self.x0 - d,
            y0: self.y0 - d,
            x1: self.x1 + d,
            y1: self.y1 + d,
        }
    }
    pub fn shift(&self, p: Point) -> R {
        R {
            x0: self.x0 + p.x,
            y0: self.y0 + p.y,
            x1: self.x1 + p.x,
            y1: self.y1 + p.y,
        }
    }
    pub fn overlap(&self, o: &R) -> f64 {
        let w = self.x1.min(o.x1) - self.x0.max(o.x0);
        let h = self.y1.min(o.y1) - self.y0.max(o.y0);
        if w > 0.0 && h > 0.0 { w * h } else { 0.0 }
    }
    pub fn corners(&self) -> [Point; 4] {
        [
            pt(self.x0, self.y0),
            pt(self.x1, self.y0),
            pt(self.x1, self.y1),
            pt(self.x0, self.y1),
        ]
    }
    pub fn center(&self) -> Point {
        pt((self.x0 + self.x1) * 0.5, (self.y0 + self.y1) * 0.5)
    }
    pub fn union(&self, o: &R) -> R {
        R {
            x0: self.x0.min(o.x0),
            y0: self.y0.min(o.y0),
            x1: self.x1.max(o.x1),
            y1: self.y1.max(o.y1),
        }
    }
    pub fn empty() -> R {
        R {
            x0: f64::INFINITY,
            y0: f64::INFINITY,
            x1: f64::NEG_INFINITY,
            y1: f64::NEG_INFINITY,
        }
    }
    /// Bounding box of this rectangle (in a local frame) placed at `pos` with rotation `rot`.
    pub fn transformed_bbox(&self, pos: Point, rot: f64) -> R {
        let mut r = R::empty();
        for c in self.corners() {
            let q = pos + kicad_rot(c, rot);
            r.x0 = r.x0.min(q.x);
            r.y0 = r.y0.min(q.y);
            r.x1 = r.x1.max(q.x);
            r.y1 = r.y1.max(q.y);
        }
        r
    }
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct Pad {
    pub number: String,
    pub local: Point,
    pub net: Option<usize>,
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct Part {
    pub reference: String,
    pub value: String,
    pub footprint: String,
    /// Index of the footprint node in the root's arguments.
    pub node: usize,
    pub pos: Point,
    pub rot: f64,
    /// Courtyard in the footprint frame (unrotated).
    pub court: Vec<R>,
    pub pads: Vec<Pad>,
    /// Keepout polygons of the footprint (board coordinates in the file) that forbid copper pours
    /// on inner layers, with their layer names.
    pub keepouts: Vec<(Vec<Point>, Vec<String>)>,
}

impl Part {
    pub fn pad_world(&self, pad: &Pad) -> Point {
        self.pos + kicad_rot(pad.local, self.rot)
    }
    pub fn court_area(&self) -> f64 {
        self.court.iter().map(|r| r.area()).sum()
    }
}

pub struct Design {
    pub parts: Vec<Part>,
    pub nets: Vec<String>,
    pub by_ref: HashMap<String, usize>,
}

fn xy(n: &Node) -> Option<Point> {
    let a = n.args();
    Some(pt(a.first()?.as_f64()?, a.get(1)?.as_f64()?))
}

fn child_xy(n: &Node, h: &str) -> Option<Point> {
    xy(n.child(h)?)
}

impl Design {
    pub fn net_id(&self, name: &str) -> Option<usize> {
        self.nets.iter().position(|n| n == name)
    }

    pub fn from_root(root: &Node) -> Design {
        let mut nets: Vec<String> = Vec::new();
        let mut net_ix: HashMap<String, usize> = HashMap::new();
        let mut parts = Vec::new();
        for (i, fp) in root.args().iter().enumerate() {
            if !matches!(fp.head(), Some("footprint") | Some("module")) {
                continue;
            }
            let at = fp.child_f64s("at").unwrap_or_default();
            let pos = pt(at.first().copied().unwrap_or(0.0), at.get(1).copied().unwrap_or(0.0));
            let rot = at.get(2).copied().unwrap_or(0.0);
            let prop = |name: &str| -> String {
                fp.children("property")
                    .find(|p| p.args().first().and_then(|x| x.as_str()) == Some(name))
                    .and_then(|p| p.args().get(1)?.as_str())
                    .unwrap_or("")
                    .to_string()
            };
            let reference = prop("Reference");
            let value = prop("Value");
            let footprint = fp.args().first().and_then(|x| x.as_str()).unwrap_or("").to_string();
            let mut pads = Vec::new();
            for pad in fp.children("pad") {
                let number = pad.args().first().and_then(|x| x.as_str()).unwrap_or("").to_string();
                let local = child_xy(pad, "at").unwrap_or(pt(0.0, 0.0));
                let net = pad.child("net").and_then(|n| n.args().last()?.as_str()).map(|name| {
                    *net_ix.entry(name.to_string()).or_insert_with(|| {
                        nets.push(name.to_string());
                        nets.len() - 1
                    })
                });
                pads.push(Pad { number, local, net });
            }
            let court = courtyard(fp);
            let mut keepouts = Vec::new();
            for z in fp.children("zone") {
                let Some(k) = z.child("keepout") else { continue };
                if k.child_str("copperpour") != Some("not_allowed") {
                    continue;
                }
                let mut layers: Vec<String> = Vec::new();
                for h in ["layer", "layers"] {
                    if let Some(l) = z.child(h) {
                        layers.extend(l.args().iter().filter_map(|x| x.as_str()).map(String::from));
                    }
                }
                let pts: Vec<Point> = z
                    .child("polygon")
                    .and_then(|p| p.child("pts"))
                    .map(|p| p.children("xy").filter_map(xy).collect())
                    .unwrap_or_default();
                if pts.len() >= 3 {
                    keepouts.push((pts, layers));
                }
            }
            parts.push(Part {
                reference,
                value,
                footprint,
                node: i,
                pos,
                rot,
                court,
                pads,
                keepouts,
            });
        }
        let by_ref = parts
            .iter()
            .enumerate()
            .map(|(i, p)| (p.reference.clone(), i))
            .collect();
        Design { parts, nets, by_ref }
    }
}

/// Courtyard of a footprint as rectangles in its frame: rectilinear outlines are split into
/// horizontal slabs, anything else becomes its bounding box. Without a courtyard: pad extents.
fn courtyard(fp: &Node) -> Vec<R> {
    let mut segs: Vec<(Point, Point)> = Vec::new();
    let mut bbox = R::empty();
    let mut curved = false;
    let add = |p: Point, bbox: &mut R| {
        bbox.x0 = bbox.x0.min(p.x);
        bbox.y0 = bbox.y0.min(p.y);
        bbox.x1 = bbox.x1.max(p.x);
        bbox.y1 = bbox.y1.max(p.y);
    };
    for g in fp.args() {
        let Some(h) = g.head() else { continue };
        if !matches!(g.child_str("layer"), Some("F.CrtYd") | Some("B.CrtYd")) {
            continue;
        }
        match h {
            "fp_line" => {
                if let (Some(a), Some(b)) = (child_xy(g, "start"), child_xy(g, "end")) {
                    segs.push((a, b));
                    add(a, &mut bbox);
                    add(b, &mut bbox);
                }
            }
            "fp_rect" => {
                if let (Some(a), Some(b)) = (child_xy(g, "start"), child_xy(g, "end")) {
                    let c = [a, pt(b.x, a.y), b, pt(a.x, b.y)];
                    for i in 0..4 {
                        segs.push((c[i], c[(i + 1) % 4]));
                        add(c[i], &mut bbox);
                    }
                }
            }
            "fp_poly" => {
                let ps: Vec<Point> = g
                    .child("pts")
                    .map(|p| p.children("xy").filter_map(xy).collect())
                    .unwrap_or_default();
                for i in 0..ps.len() {
                    segs.push((ps[i], ps[(i + 1) % ps.len()]));
                    add(ps[i], &mut bbox);
                }
            }
            "fp_circle" => {
                if let (Some(c), Some(e)) = (child_xy(g, "center"), child_xy(g, "end")) {
                    let r = c.dist(e);
                    add(pt(c.x - r, c.y - r), &mut bbox);
                    add(pt(c.x + r, c.y + r), &mut bbox);
                    curved = true;
                }
            }
            "fp_arc" => {
                for k in ["start", "mid", "end"] {
                    if let Some(p) = child_xy(g, k) {
                        add(p, &mut bbox);
                    }
                }
                curved = true;
            }
            _ => {}
        }
    }
    if bbox.x0 > bbox.x1 {
        // no courtyard: pads
        for pad in fp.children("pad") {
            let (Some(at), Some(sz)) = (child_xy(pad, "at"), pad.child_f64s("size")) else {
                continue;
            };
            let r = sz.iter().cloned().fold(0.0, f64::max) * 0.5 + 0.25;
            add(pt(at.x - r, at.y - r), &mut bbox);
            add(pt(at.x + r, at.y + r), &mut bbox);
        }
        if bbox.x0 > bbox.x1 {
            return vec![];
        }
        return vec![bbox];
    }
    let tol = 1e-6;
    let rectilinear = !curved
        && segs
            .iter()
            .all(|(a, b)| (a.x - b.x).abs() < tol || (a.y - b.y).abs() < tol);
    if !rectilinear {
        return vec![bbox];
    }
    let mut vert: Vec<(f64, f64, f64)> = Vec::new();
    for (a, b) in &segs {
        if (a.x - b.x).abs() < tol && (a.y - b.y).abs() > tol {
            let v = (a.x, a.y.min(b.y), a.y.max(b.y));
            if !vert
                .iter()
                .any(|w| (w.0 - v.0).abs() < tol && (w.1 - v.1).abs() < tol && (w.2 - v.2).abs() < tol)
            {
                vert.push(v);
            }
        }
    }
    let mut ys: Vec<f64> = vert.iter().flat_map(|v| [v.1, v.2]).collect();
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ys.dedup_by(|a, b| (*a - *b).abs() < tol);
    let mut out: Vec<R> = Vec::new();
    let mut prev: Vec<(f64, f64)> = Vec::new();
    let mut open: Vec<usize> = Vec::new();
    for w in ys.windows(2) {
        let ym = (w[0] + w[1]) * 0.5;
        let mut xs: Vec<f64> = vert.iter().filter(|v| v.1 < ym && v.2 > ym).map(|v| v.0).collect();
        xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        if xs.len() % 2 == 1 {
            return vec![bbox];
        }
        let iv: Vec<(f64, f64)> = xs.chunks(2).map(|c| (c[0], c[1])).collect();
        if iv == prev && !open.is_empty() {
            for &k in &open {
                out[k].y1 = w[1];
            }
        } else {
            open.clear();
            for &(a, b) in &iv {
                open.push(out.len());
                out.push(R {
                    x0: a,
                    y0: w[0],
                    x1: b,
                    y1: w[1],
                });
            }
            prev = iv;
        }
    }
    if out.is_empty() { vec![bbox] } else { out }
}
