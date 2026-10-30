//! KiCad 9/10 `.kicad_pcb` (+ `.kicad_pro`) to [`Board`].

use std::collections::HashMap;
use std::path::Path;

use anyhow::{Context, Result, anyhow};
use gummiband_core::board::*;
use gummiband_core::geom::*;

use crate::sexpr::{self, Node};

/// What to do with existing tracks and vias of the input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeepCopper {
    /// Remove everything (route the board from scratch).
    None,
    /// Keep locked tracks and vias as fixed copper.
    Locked,
    /// Keep all tracks and vias.
    All,
}

#[derive(Debug, Clone)]
pub struct ReadOptions {
    pub keep: KeepCopper,
    /// Treat copper zones on routing layers as obstacles (otherwise they are refilled later).
    pub zones_as_obstacles: bool,
    /// Force these copper layers to be planes (not routed). Layers of KiCad type "power" are
    /// planes by default.
    pub plane_layers: Vec<String>,
    /// Let the router shift footprints a little: all except locked ones, connectors, modules,
    /// mounting holes and the references in `fixed`.
    pub move_parts: bool,
    /// References that never move.
    pub fixed: Vec<String>,
}

impl Default for ReadOptions {
    fn default() -> Self {
        ReadOptions {
            keep: KeepCopper::Locked,
            zones_as_obstacles: false,
            plane_layers: Vec::new(),
            move_parts: false,
            fixed: Vec::new(),
        }
    }
}

/// Parsed file plus the board model. The tree is kept for the writer.
pub struct KicadBoard {
    pub text: String,
    pub root: Node,
    pub board: Board,
    /// KiCad 9 style numbered nets (`(net 3 "name")` table at the root): name -> number.
    pub net_numbers: Option<HashMap<String, i64>>,
    /// Copper layer names in board order, as written in the file.
    pub layer_names: Vec<String>,
}

pub fn read_board(path: &Path, opts: &ReadOptions) -> Result<KicadBoard> {
    let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    let pro = path.with_extension("kicad_pro");
    let pro_json = std::fs::read_to_string(&pro)
        .ok()
        .and_then(|s| serde_json::from_str::<serde_json::Value>(&s).ok());
    let mut kb = parse_board(text, pro_json.as_ref(), opts)?;
    if let Ok(dru) = std::fs::read_to_string(path.with_extension("kicad_dru")) {
        let rules = crate::dru::parse_rules(&dru);
        kb.board.pair_clearance = crate::dru::pair_clearances(&kb.board, &rules).into_iter().collect();
        crate::dru::apply_width_rules(&mut kb.board, &dru);
    }
    Ok(kb)
}

pub fn parse_board(text: String, pro: Option<&serde_json::Value>, opts: &ReadOptions) -> Result<KicadBoard> {
    let root = sexpr::parse(&text).map_err(|e| anyhow!("parse error: {e}"))?;
    if root.head() != Some("kicad_pcb") {
        return Err(anyhow!("not a kicad_pcb file"));
    }
    let mut r = Reader {
        b: Board::default(),
        net_map: HashMap::new(),
        net_numbers: None,
        numbered: HashMap::new(),
        patterns: Vec::new(),
        copper: Vec::new(),
        edges: Vec::new(),
        movable: Vec::new(),
        fp_uuids: HashMap::new(),
        copper_uuids: HashMap::new(),
    };

    r.read_layers(&root, opts);
    r.read_rules(pro);
    r.read_classes(pro);
    r.read_net_table(&root);

    for (i, n) in root.args().iter().enumerate() {
        match n.head() {
            Some("footprint") | Some("module") => r.read_footprint(n),
            Some("segment") | Some("arc") | Some("via") => {
                let first = r.b.obstacles.len();
                r.read_copper(n, opts.keep, i as u32);
                if let Some(id) = item_id(n) {
                    r.copper_uuids
                        .insert(id.to_string(), (first..r.b.obstacles.len()).collect());
                }
            }
            Some("zone") => r.read_zone(n, opts),
            Some("gr_rect") | Some("gr_line") | Some("gr_arc") | Some("gr_circle") | Some("gr_poly") => {
                r.read_graphic(n, None)
            }
            _ => {}
        }
    }
    r.finish_outline();
    r.read_groups(&root);
    if !matches!(opts.keep, KeepCopper::None) {
        r.b.add_island_pins();
    }
    if opts.move_parts {
        r.b.movable = std::mem::take(&mut r.movable)
            .into_iter()
            .filter(|f| !opts.fixed.contains(f) && r.b.courtyards.contains_key(f))
            .collect();
        // a block with one fixed member stays where it is
        for block in &r.b.blocks {
            if block.refs.iter().any(|f| !r.b.movable.contains(f)) {
                r.b.movable.retain(|f| !block.refs.contains(f));
            }
        }
    }

    Ok(KicadBoard {
        text,
        root,
        board: r.b,
        net_numbers: r.net_numbers,
        layer_names: r.copper,
    })
}

struct Reader {
    b: Board,
    net_map: HashMap<String, NetId>,
    net_numbers: Option<HashMap<String, i64>>,
    numbered: HashMap<i64, String>,
    /// Net class patterns (glob, class name), first match wins.
    patterns: Vec<(String, String)>,
    copper: Vec<String>,
    /// Loose Edge.Cuts segments, chained into polygons at the end.
    edges: Vec<(Point, Point)>,
    /// Footprints that may move under `move_parts` (unlocked, no connector, module or hole).
    movable: Vec<String>,
    /// File ids of footprints and of copper with its obstacles, to resolve groups.
    fp_uuids: HashMap<String, String>,
    copper_uuids: HashMap<String, Vec<usize>>,
}

/// The file id of an item: `uuid` since KiCad 8, `tstamp` before.
fn item_id(n: &Node) -> Option<&str> {
    n.child_str("uuid").or_else(|| n.child_str("tstamp"))
}

fn xy(n: &Node) -> Option<Point> {
    let a = n.args();
    Some(pt(a.first()?.as_f64()?, a.get(1)?.as_f64()?))
}

fn child_xy(n: &Node, head: &str) -> Option<Point> {
    xy(n.child(head)?)
}

fn pts(n: &Node) -> Vec<Point> {
    n.child("pts")
        .map(|p| p.children("xy").filter_map(xy).collect())
        .unwrap_or_default()
}

/// KiCad rotates counter-clockwise on screen with y pointing down.
fn kicad_rot(p: Point, deg: f64) -> Point {
    p.rotate(-deg.to_radians())
}

fn glob_match(pat: &str, s: &str) -> bool {
    fn rec(p: &[u8], s: &[u8]) -> bool {
        match (p.first(), s.first()) {
            (None, None) => true,
            (Some(b'*'), _) => rec(&p[1..], s) || (!s.is_empty() && rec(p, &s[1..])),
            (Some(b'?'), Some(_)) => rec(&p[1..], &s[1..]),
            (Some(a), Some(b)) if a == b => rec(&p[1..], &s[1..]),
            _ => false,
        }
    }
    rec(pat.as_bytes(), s.as_bytes())
}

impl Reader {
    fn read_layers(&mut self, root: &Node, opts: &ReadOptions) {
        let mut cu: Vec<(i64, String, bool)> = Vec::new();
        if let Some(layers) = root.child("layers") {
            for l in layers.args() {
                let a = l.items();
                let (Some(id), Some(name)) = (a.first().and_then(|x| x.as_f64()), a.get(1).and_then(|x| x.as_str()))
                else {
                    continue;
                };
                if !name.ends_with(".Cu") {
                    continue;
                }
                let kind = a.get(2).and_then(|x| x.as_str()).unwrap_or("signal");
                let plane = kind == "power" || opts.plane_layers.iter().any(|p| p == name);
                cu.push((id as i64, name.to_string(), plane));
            }
        }
        // Board order: F.Cu, In1..InN, B.Cu.
        let rank = |name: &str| -> i64 {
            match name {
                "F.Cu" => 0,
                "B.Cu" => 1000,
                _ => name
                    .trim_start_matches("In")
                    .trim_end_matches(".Cu")
                    .parse()
                    .unwrap_or(500),
            }
        };
        cu.sort_by_key(|(_, n, _)| rank(n));
        for (_, name, plane) in cu {
            self.copper.push(name.clone());
            self.b.layers.push(Layer { name, is_plane: plane });
        }
    }

    fn read_rules(&mut self, pro: Option<&serde_json::Value>) {
        let Some(r) = pro.and_then(|p| p.pointer("/board/design_settings/rules")) else {
            return;
        };
        let g = |k: &str| r.get(k).and_then(|v| v.as_f64());
        let rules = &mut self.b.rules;
        if let Some(v) = g("min_clearance") {
            rules.min_clearance = v.max(rules.min_clearance.min(v));
        }
        if let Some(v) = g("min_track_width") {
            rules.min_track_width = v;
        }
        if let Some(v) = g("min_copper_edge_clearance") {
            rules.edge_clearance = v;
        }
        if let Some(v) = g("min_hole_to_hole") {
            rules.hole_to_hole = v;
        }
        if let Some(v) = g("min_hole_clearance") {
            rules.hole_clearance = v;
        }
        if let Some(v) = g("min_via_diameter") {
            rules.min_via_diameter = v;
        }
        if let Some(v) = g("max_error") {
            rules.max_error = v;
        }
    }

    fn read_classes(&mut self, pro: Option<&serde_json::Value>) {
        let classes = pro
            .and_then(|p| p.pointer("/net_settings/classes"))
            .and_then(|c| c.as_array());
        for c in classes.into_iter().flatten() {
            let f = |k: &str, d: f64| c.get(k).and_then(|v| v.as_f64()).unwrap_or(d);
            let name = c.get("name").and_then(|v| v.as_str()).unwrap_or("Default").to_string();
            let prio = c.get("priority").and_then(|v| v.as_i64()).unwrap_or(i32::MAX as i64);
            self.b.classes.push(NetClass {
                name,
                track_width: f("track_width", 0.2),
                clearance: f("clearance", 0.2),
                via_diameter: f("via_diameter", 0.6),
                via_drill: f("via_drill", 0.3),
                priority: prio.clamp(i32::MIN as i64, i32::MAX as i64) as i32,
                min_width: None,
                neck_refs: Vec::new(),
            });
        }
        if !self.b.classes.iter().any(|c| c.name == "Default") {
            self.b.classes.insert(
                0,
                NetClass {
                    name: "Default".into(),
                    track_width: 0.2,
                    clearance: 0.2,
                    via_diameter: 0.6,
                    via_drill: 0.3,
                    priority: i32::MAX,
                    min_width: None,
                    neck_refs: Vec::new(),
                },
            );
        }
        self.patterns = pro
            .and_then(|p| p.pointer("/net_settings/netclass_patterns"))
            .and_then(|v| v.as_array())
            .into_iter()
            .flatten()
            .filter_map(|v| {
                Some((
                    v.get("pattern")?.as_str()?.to_string(),
                    v.get("netclass")?.as_str()?.to_string(),
                ))
            })
            .collect();
        if let Some(assign) = pro
            .and_then(|p| p.pointer("/net_settings/netclass_assignments"))
            .and_then(|v| v.as_object())
        {
            for (net, cls) in assign {
                let cls = cls
                    .as_str()
                    .map(str::to_string)
                    .or_else(|| cls.as_array()?.first()?.as_str().map(str::to_string));
                if let Some(cls) = cls {
                    self.patterns.insert(0, (net.clone(), cls));
                }
            }
        }
    }

    fn read_net_table(&mut self, root: &Node) {
        let mut table = HashMap::new();
        for n in root.children("net") {
            let a = n.args();
            if let (Some(num), Some(name)) = (a.first().and_then(|x| x.as_f64()), a.get(1).and_then(|x| x.as_str())) {
                self.numbered.insert(num as i64, name.to_string());
                table.insert(name.to_string(), num as i64);
            }
        }
        if !table.is_empty() {
            self.net_numbers = Some(table);
        }
    }

    /// Net of an item: `(net "name")` (KiCad 10) or `(net 3 ["name"])` (KiCad 9).
    fn net_of(&mut self, n: &Node) -> Option<NetId> {
        let net = n.child("net")?;
        let a = net.args();
        let name = match (a.first(), a.get(1)) {
            (_, Some(s)) => s.as_str()?.to_string(),
            (Some(first), None) => match (&self.net_numbers, first.as_f64()) {
                (Some(_), Some(num)) => self.numbered.get(&(num as i64))?.clone(),
                _ => first.as_str()?.to_string(),
            },
            _ => return None,
        };
        if name.is_empty() {
            return None;
        }
        Some(self.net(&name))
    }

    fn net(&mut self, name: &str) -> NetId {
        if let Some(&id) = self.net_map.get(name) {
            return id;
        }
        let class = self
            .patterns
            .iter()
            .find(|(p, _)| glob_match(p, name))
            .and_then(|(_, c)| self.b.classes.iter().position(|k| &k.name == c))
            .or_else(|| self.b.classes.iter().position(|k| k.name == "Default"))
            .unwrap_or(0) as ClassId;
        let id = self.b.nets.len() as NetId;
        self.b.nets.push(Net {
            name: name.to_string(),
            class,
            pins: Vec::new(),
            planes: LayerSet::default(),
            priority: self.b.classes[class as usize].priority,
        });
        self.net_map.insert(name.to_string(), id);
        id
    }

    fn layer_set(&self, names: &[&str]) -> LayerSet {
        let mut s = LayerSet::default();
        for name in names {
            match *name {
                "*.Cu" => s = self.b.all_layers(),
                "F&B.Cu" => {
                    s.insert(0);
                    s.insert((self.b.layers.len() - 1) as LayerId);
                }
                _ => {
                    if let Some(id) = self.b.layer_id(name) {
                        s.insert(id);
                    }
                }
            }
        }
        s
    }

    fn layers_of(&self, n: &Node) -> LayerSet {
        let mut names: Vec<&str> = Vec::new();
        if let Some(l) = n.child("layers") {
            names.extend(l.args().iter().filter_map(|x| x.as_str()));
        }
        if let Some(l) = n.child("layer") {
            names.extend(l.args().iter().filter_map(|x| x.as_str()));
        }
        self.layer_set(&names)
    }

    fn clearance_of(&self, net: Option<NetId>) -> f64 {
        net.map_or(self.b.rules.min_clearance, |n| self.b.clearance(Some(n), None))
    }

    fn push_obstacle(&mut self, o: Obstacle) -> usize {
        self.b.obstacles.push(o);
        self.b.obstacles.len() - 1
    }

    fn read_footprint(&mut self, fp: &Node) {
        let at = fp.child_f64s("at").unwrap_or_default();
        let pos = pt(at.first().copied().unwrap_or(0.0), at.get(1).copied().unwrap_or(0.0));
        let rot = at.get(2).copied().unwrap_or(0.0);
        let reference = fp
            .children("property")
            .find(|p| p.args().first().and_then(|x| x.as_str()) == Some("Reference"))
            .and_then(|p| p.args().get(1)?.as_str())
            .or_else(|| {
                fp.children("fp_text")
                    .find(|t| t.args().first().and_then(|x| x.as_str()) == Some("reference"))
                    .and_then(|t| t.args().get(1)?.as_str())
            })
            .unwrap_or("?")
            .to_string();
        if let Some(id) = item_id(fp) {
            self.fp_uuids.insert(id.to_string(), reference.clone());
        }
        let first_obstacle = self.b.obstacles.len();
        // parts with an outside interface or a mechanical role stay where they are
        let lib = fp.args().first().and_then(|x| x.as_str()).unwrap_or("");
        let fixed_kind = [
            "Connector",
            "Module",
            "MountingHole",
            "Button_Switch",
            "Jumper",
            "TestPoint",
        ]
        .iter()
        .any(|k| lib.contains(k))
            || ["J", "P", "CN", "USB", "H", "MH", "SW", "TP"].iter().any(|p| {
                reference
                    .strip_prefix(p)
                    .is_some_and(|rest| rest.starts_with(|c: char| c.is_ascii_digit()))
            });
        if !fp.flag("locked") && !fixed_kind {
            self.movable.push(reference.clone());
        }
        for pad in fp.children("pad") {
            self.read_pad(pad, pos, rot, &reference);
        }
        let mut court = Rect::empty();
        for g in fp.args() {
            if let Some(h) = g.head()
                && matches!(h, "fp_line" | "fp_arc" | "fp_rect" | "fp_circle" | "fp_poly")
                && matches!(g.child_str("layer"), Some("F.CrtYd") | Some("B.CrtYd"))
            {
                let mut ps: Vec<Point> = ["start", "end", "mid", "center"]
                    .iter()
                    .filter_map(|k| child_xy(g, k))
                    .collect();
                ps.extend(pts(g));
                for q in ps {
                    court.add(pos + kicad_rot(q, rot));
                }
            }
        }
        if court.width() > 0.0 {
            self.b.courtyards.insert(reference.clone(), court);
        }
        for g in fp.args() {
            if let Some(h) = g.head()
                && matches!(h, "fp_line" | "fp_arc" | "fp_rect" | "fp_circle" | "fp_poly")
            {
                self.read_graphic(g, Some((pos, rot)));
            }
        }
        for z in fp.children("zone") {
            self.read_zone(
                z,
                &ReadOptions {
                    keep: KeepCopper::None,
                    zones_as_obstacles: false,
                    plane_layers: vec![],
                    move_parts: false,
                    fixed: vec![],
                },
            );
        }
        if self.b.obstacles.len() > first_obstacle {
            self.b
                .footprint_obstacles
                .insert(reference, (first_obstacle..self.b.obstacles.len()).collect());
        }
    }

    fn read_pad(&mut self, pad: &Node, fp_pos: Point, fp_rot: f64, reference: &str) {
        let a = pad.args();
        let number = a.first().and_then(|x| x.as_str()).unwrap_or("").to_string();
        let kind = a.get(1).and_then(|x| x.as_str()).unwrap_or("smd");
        let shape = a.get(2).and_then(|x| x.as_str()).unwrap_or("rect");
        let at = pad.child_f64s("at").unwrap_or_default();
        let local = pt(at.first().copied().unwrap_or(0.0), at.get(1).copied().unwrap_or(0.0));
        let angle = at.get(2).copied().unwrap_or(fp_rot);
        let center = fp_pos + kicad_rot(local, fp_rot);
        let size = pad.child_f64s("size").unwrap_or_else(|| vec![0.0, 0.0]);
        let (w, h) = (
            size.first().copied().unwrap_or(0.0),
            size.get(1).copied().unwrap_or(0.0),
        );
        let layers = self.layers_of(pad);
        let net = self.net_of(pad);
        let me = self.b.rules.max_error;
        let hole = pad.child("drill").and_then(|d| {
            let nums: Vec<f64> = d.args().iter().filter_map(|x| x.as_f64()).collect();
            nums.iter()
                .copied()
                .fold(None, |m: Option<f64>, v| Some(m.map_or(v, |m| m.min(v))))
                .map(|v| v / 2.0)
        });

        if kind == "np_thru_hole" {
            let r = hole.unwrap_or(w.min(h) / 2.0).max(w.min(h) / 2.0);
            let clearance = self.b.rules.hole_clearance;
            self.push_obstacle(Obstacle {
                shape: Shape::Circle { c: center, r },
                layers: self.b.all_layers(),
                net: None,
                kind: ObstacleKind::Hole,
                hole: Some(r),
                clearance,
                pin: None,
                locked: true,
            });
            return;
        }
        if layers.is_empty() {
            return;
        }

        let rad = -angle.to_radians();
        let mut shapes: Vec<Shape> = Vec::new();
        match shape {
            "circle" => shapes.push(Shape::Circle { c: center, r: w / 2.0 }),
            "oval" => {
                let (len, r, dir) = if w >= h {
                    (w - h, h / 2.0, pt(1.0, 0.0))
                } else {
                    (h - w, w / 2.0, pt(0.0, 1.0))
                };
                let d = dir.rotate(rad) * (len / 2.0);
                if len < 1e-9 {
                    shapes.push(Shape::Circle { c: center, r });
                } else {
                    shapes.push(Shape::Capsule {
                        a: center - d,
                        b: center + d,
                        r,
                    });
                }
            }
            "roundrect" => {
                let ratio = pad.child_f64("roundrect_rratio").unwrap_or(0.25);
                let r = w.min(h) * ratio;
                shapes.push(Shape::Polygon(roundrect_polygon(center, w, h, r, rad, me)));
            }
            "custom" => {
                let anchor_circle = pad
                    .child("options")
                    .and_then(|o| o.child_str("anchor"))
                    .map(|s| s == "circle")
                    .unwrap_or(false);
                if anchor_circle {
                    shapes.push(Shape::Circle { c: center, r: w / 2.0 });
                } else {
                    shapes.push(Shape::Polygon(roundrect_polygon(center, w, h, 0.0, rad, me)));
                }
                if let Some(prims) = pad.child("primitives") {
                    for p in prims.args() {
                        let tf = |q: Point| center + q.rotate(rad);
                        let width = p.child_f64("width").unwrap_or(0.0);
                        match p.head() {
                            Some("gr_poly") => {
                                let ps: Vec<Point> = pts(p).into_iter().map(tf).collect();
                                if ps.len() >= 3 {
                                    shapes.push(Shape::Polygon(Polygon::new(ps)));
                                }
                            }
                            Some("gr_circle") => {
                                if let (Some(c), Some(e)) = (child_xy(p, "center"), child_xy(p, "end")) {
                                    shapes.push(Shape::Circle {
                                        c: tf(c),
                                        r: c.dist(e) + width / 2.0,
                                    });
                                }
                            }
                            Some("gr_rect") => {
                                if let (Some(s), Some(e)) = (child_xy(p, "start"), child_xy(p, "end")) {
                                    let ps = vec![s, pt(e.x, s.y), e, pt(s.x, e.y)].into_iter().map(tf).collect();
                                    shapes.push(Shape::Polygon(Polygon::new(ps)));
                                }
                            }
                            Some("gr_line") => {
                                if let (Some(s), Some(e)) = (child_xy(p, "start"), child_xy(p, "end")) {
                                    shapes.push(Shape::Capsule {
                                        a: tf(s),
                                        b: tf(e),
                                        r: width / 2.0,
                                    });
                                }
                            }
                            _ => {}
                        }
                    }
                }
            }
            // rect, trapezoid, chamfered_rect: the bounding rectangle
            _ => shapes.push(Shape::Polygon(roundrect_polygon(center, w, h, 0.0, rad, me))),
        }

        let clearance = pad.child_f64("clearance").unwrap_or(0.0).max(self.clearance_of(net));
        let pin_idx = net.map(|_| self.b.pins.len());
        let mut obs = Vec::new();
        for s in shapes {
            obs.push(self.push_obstacle(Obstacle {
                shape: s,
                layers,
                net,
                kind: ObstacleKind::Pad,
                hole,
                clearance,
                pin: pin_idx,
                locked: true,
            }));
        }
        if let (Some(net), Some(pin)) = (net, pin_idx) {
            let mut bb = Rect::empty();
            for &o in &obs {
                let ob = self.b.obstacles[o].shape.bbox();
                bb.add(ob.min);
                bb.add(ob.max);
            }
            self.b.pins.push(Pin {
                name: format!("{reference}.{number}"),
                footprint: reference.to_string(),
                net,
                pos: center,
                layers,
                obstacles: obs,
                size: (bb.width(), bb.height()),
            });
            self.b.nets[net as usize].pins.push(pin);
        }
    }

    fn read_copper(&mut self, n: &Node, keep: KeepCopper, source: u32) {
        let locked = n.flag("locked");
        match keep {
            KeepCopper::None => return,
            KeepCopper::Locked if !locked => return,
            _ => {}
        }
        let net = self.net_of(n);
        let clearance = self.clearance_of(net);
        let me = self.b.rules.max_error;
        let first = self.b.obstacles.len() as u32;
        let kept = |r: &mut Self, kind: CopperKind| {
            let last = r.b.obstacles.len() as u32;
            r.b.copper.push(CopperItem {
                kind,
                net,
                locked,
                source,
                first,
                last,
            });
        };
        match n.head() {
            Some("via") => {
                let (Some(at), Some(size)) = (child_xy(n, "at"), n.child_f64("size")) else {
                    return;
                };
                let drill = n.child_f64("drill").unwrap_or(size / 2.0);
                let layers = self.layers_of(n);
                // Through vias span all layers between the two named ones.
                let layers = if layers.0.count_ones() >= 2 {
                    let lo = layers.0.trailing_zeros();
                    let hi = 63 - layers.0.leading_zeros();
                    LayerSet(((1u64 << (hi + 1)) - 1) & !((1u64 << lo) - 1))
                } else {
                    self.b.all_layers()
                };
                self.push_obstacle(Obstacle {
                    shape: Shape::Circle { c: at, r: size / 2.0 },
                    layers,
                    net,
                    kind: ObstacleKind::Via,
                    hole: Some(drill / 2.0),
                    clearance,
                    pin: None,
                    locked,
                });
                kept(
                    self,
                    CopperKind::Via {
                        at,
                        diameter: size,
                        drill,
                        layers,
                    },
                );
            }
            Some(kind) => {
                let (Some(s), Some(e)) = (child_xy(n, "start"), child_xy(n, "end")) else {
                    return;
                };
                let w = n.child_f64("width").unwrap_or(0.2);
                let layers = self.layers_of(n);
                let path = if kind == "arc" {
                    match child_xy(n, "mid") {
                        Some(m) => arc_polyline(s, m, e, me),
                        None => vec![s, e],
                    }
                } else {
                    vec![s, e]
                };
                for seg in path.windows(2) {
                    self.push_obstacle(Obstacle {
                        shape: Shape::Capsule {
                            a: seg[0],
                            b: seg[1],
                            r: w / 2.0,
                        },
                        layers,
                        net,
                        kind: ObstacleKind::Track,
                        hole: None,
                        clearance,
                        pin: None,
                        locked,
                    });
                }
                let layer = layers.iter().next().unwrap_or(0);
                kept(
                    self,
                    match (kind, child_xy(n, "mid")) {
                        ("arc", Some(mid)) => CopperKind::Arc {
                            a: s,
                            mid,
                            b: e,
                            width: w,
                            layer,
                        },
                        _ => CopperKind::Line {
                            a: s,
                            b: e,
                            width: w,
                            layer,
                        },
                    },
                );
            }
            None => {}
        }
    }

    fn read_zone(&mut self, z: &Node, opts: &ReadOptions) {
        let layers = self.layers_of(z);
        let net = self.net_of(z);
        let outline: Vec<Point> = z.child("polygon").map(pts).unwrap_or_default();
        if outline.len() < 3 {
            return;
        }
        if let Some(k) = z.child("keepout") {
            let na = |h: &str| k.child_str(h) == Some("not_allowed");
            let (tracks, vias) = (na("tracks"), na("vias"));
            if tracks || vias {
                self.push_obstacle(Obstacle {
                    shape: Shape::Polygon(Polygon::new(outline)),
                    layers,
                    net: None,
                    kind: ObstacleKind::Keepout { vias, tracks },
                    hole: None,
                    clearance: 0.0,
                    pin: None,
                    locked: true,
                });
            }
            return;
        }
        let Some(net) = net else { return };
        for l in layers.iter() {
            if self.b.layers[l as usize].is_plane {
                self.b.nets[net as usize].planes.insert(l);
                self.b.planes.push(Plane {
                    net,
                    layer: l,
                    outline: Polygon::new(outline.clone()),
                });
            } else if opts.zones_as_obstacles {
                let clearance = self.clearance_of(Some(net));
                self.push_obstacle(Obstacle {
                    shape: Shape::Polygon(Polygon::new(outline.clone())),
                    layers: LayerSet::single(l),
                    net: Some(net),
                    kind: ObstacleKind::Zone,
                    hole: None,
                    clearance,
                    pin: None,
                    locked: true,
                });
            }
        }
    }

    /// KiCad groups with footprints become blocks.
    fn read_groups(&mut self, root: &Node) {
        for g in root.children("group") {
            let mut block = Block {
                name: g.args().first().and_then(|x| x.as_str()).unwrap_or("").to_string(),
                ..Default::default()
            };
            for m in g.child("members").map(|m| m.args()).unwrap_or(&[]) {
                let Some(id) = m.as_str() else { continue };
                if let Some(f) = self.fp_uuids.get(id) {
                    block.refs.push(f.clone());
                } else if let Some(obs) = self.copper_uuids.get(id) {
                    block.uuids.push(id.to_string());
                    block.copper.extend(obs);
                }
            }
            if !block.refs.is_empty() {
                self.b.blocks.push(block);
            }
        }
    }

    fn read_graphic(&mut self, g: &Node, tf: Option<(Point, f64)>) {
        let layer = g.child_str("layer").unwrap_or("");
        let is_edge = layer == "Edge.Cuts";
        let cu = self.b.layer_id(layer);
        if !is_edge && cu.is_none() {
            return;
        }
        let me = self.b.rules.max_error;
        let map = |p: Point| match tf {
            Some((pos, rot)) => pos + kicad_rot(p, rot),
            None => p,
        };
        let width = g
            .child("stroke")
            .and_then(|s| s.child_f64("width"))
            .or_else(|| g.child_f64("width"))
            .unwrap_or(0.0);
        let filled = matches!(g.child_str("fill"), Some("yes") | Some("solid"));
        let head = g.head().unwrap_or("");
        let kind = head.trim_start_matches("gr_").trim_start_matches("fp_");
        let mut polyline: Vec<Point> = Vec::new();
        let mut closed = false;
        match kind {
            "line" => {
                if let (Some(s), Some(e)) = (child_xy(g, "start"), child_xy(g, "end")) {
                    polyline = vec![map(s), map(e)];
                }
            }
            "arc" => {
                if let (Some(s), Some(m), Some(e)) = (child_xy(g, "start"), child_xy(g, "mid"), child_xy(g, "end")) {
                    polyline = arc_polyline(map(s), map(m), map(e), me);
                }
            }
            "rect" => {
                if let (Some(s), Some(e)) = (child_xy(g, "start"), child_xy(g, "end")) {
                    polyline = vec![map(s), map(pt(e.x, s.y)), map(e), map(pt(s.x, e.y))];
                    closed = true;
                }
            }
            "circle" => {
                if let (Some(c), Some(e)) = (child_xy(g, "center"), child_xy(g, "end")) {
                    polyline = circle_polygon(map(c), c.dist(e), me).pts;
                    closed = true;
                }
            }
            "poly" => {
                polyline = pts(g).into_iter().map(map).collect();
                closed = true;
            }
            _ => {}
        }
        if polyline.len() < 2 {
            return;
        }
        if is_edge {
            if closed {
                self.b.outline.push(Polygon::new(polyline));
            } else {
                self.edges.extend(polyline.windows(2).map(|w| (w[0], w[1])));
            }
            return;
        }
        let layer = cu.unwrap();
        let net = self.net_of(g);
        let clearance = self.clearance_of(net);
        let mut shapes = Vec::new();
        if closed && filled && polyline.len() >= 3 {
            shapes.push(Shape::Polygon(Polygon::new(polyline.clone())));
        }
        if width > 0.0 {
            let n = polyline.len();
            let segs = if closed { n } else { n - 1 };
            for i in 0..segs {
                shapes.push(Shape::Capsule {
                    a: polyline[i],
                    b: polyline[(i + 1) % n],
                    r: width / 2.0,
                });
            }
        }
        for s in shapes {
            self.push_obstacle(Obstacle {
                shape: s,
                layers: LayerSet::single(layer),
                net,
                kind: ObstacleKind::Graphic,
                hole: None,
                clearance,
                pin: None,
                locked: true,
            });
        }
    }

    /// Chain loose Edge.Cuts segments into closed polygons. Exact joints first; chains left
    /// open are then joined at their nearest ends (older files leave gaps of a few 0.1 mm that
    /// KiCad bridges as well). Chains that stay open are dropped.
    fn finish_outline(&mut self) {
        let mut segs = std::mem::take(&mut self.edges);
        let all_segs = segs.clone();
        let tol = 1e-3;
        let gap = 0.3;
        let mut dropped = false;
        let mut open: Vec<Vec<Point>> = Vec::new();
        while let Some((a, b)) = segs.pop() {
            let mut chain = vec![a, b];
            let mut closed = false;
            loop {
                let end = *chain.last().unwrap();
                if end.dist(chain[0]) < tol && chain.len() > 2 {
                    chain.pop();
                    closed = true;
                    break;
                }
                let Some(i) = segs.iter().position(|(p, q)| p.dist(end) < tol || q.dist(end) < tol) else {
                    break;
                };
                let (p, q) = segs.swap_remove(i);
                chain.push(if p.dist(end) < tol { q } else { p });
            }
            if closed {
                self.b.outline.push(Polygon::new(chain));
            } else {
                open.push(chain);
            }
        }
        // join open chains: the end of the current chain goes to the nearest free end (its own
        // start closes it)
        while let Some(mut chain) = open.pop() {
            loop {
                let end = *chain.last().unwrap();
                let own = end.dist(chain[0]);
                let best = open
                    .iter()
                    .enumerate()
                    .flat_map(|(i, c)| [(i, false, c[0].dist(end)), (i, true, c.last().unwrap().dist(end))])
                    .min_by(|x, y| x.2.total_cmp(&y.2));
                match best {
                    Some((i, rev, d)) if d < gap && d < own => {
                        let mut c = open.swap_remove(i);
                        if rev {
                            c.reverse();
                        }
                        chain.extend(c);
                    }
                    _ => {
                        if own < gap && chain.len() > 2 {
                            self.b.outline.push(Polygon::new(chain));
                        } else {
                            dropped = true;
                        }
                        break;
                    }
                }
            }
        }
        if dropped {
            self.outline_from_lines(all_segs);
        }
    }

    /// Fallback for outlines that do not chain into closed polygons (panels with mouse bites
    /// and T joints): the board is the bounding box of Edge.Cuts and every edge line is an
    /// obstacle on all copper layers, so routes stay on their side of the cuts.
    fn outline_from_lines(&mut self, segs: Vec<(Point, Point)>) {
        let mut lines = segs;
        for o in std::mem::take(&mut self.b.outline) {
            lines.extend(o.edges());
        }
        let mut bb = Rect::empty();
        for (a, b) in &lines {
            bb.add(*a);
            bb.add(*b);
        }
        self.b.outline.push(Polygon::new(vec![
            bb.min,
            pt(bb.max.x, bb.min.y),
            bb.max,
            pt(bb.min.x, bb.max.y),
        ]));
        let layers = LayerSet::all(self.b.layers.len());
        let clearance = self.b.rules.edge_clearance;
        eprintln!(
            "board outline does not close: bounding box, {} edge lines as obstacles",
            lines.len()
        );
        for (a, b) in lines {
            self.push_obstacle(Obstacle {
                shape: Shape::Capsule { a, b, r: 0.005 },
                layers,
                net: None,
                kind: ObstacleKind::Graphic,
                hole: None,
                clearance,
                pin: None,
                locked: true,
            });
        }
    }
}
