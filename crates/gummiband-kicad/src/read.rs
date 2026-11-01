//! KiCad 9/10 `.kicad_pcb` (+ `.kicad_pro`) to [`Board`].

use std::collections::HashMap;
use std::path::Path;

use anyhow::{Context, Result, anyhow};
use gummiband_core::board::*;
use gummiband_core::geom::*;

use crate::sexpr::{self, Node};

/// Room between copper of another net and a pad's solder mask opening. KiCad accepts the exact
/// margin, but the file's grid and the arcs of the tracks round. Less does not pay: 0.003 and
/// 0.005 mm leave as many bridges on Corne, with more violations (docs/MILESTONES.md).
const MASK_GAP: f64 = 0.01;

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
    /// Do not keep the unlocked tracks and vias of these nets.
    pub ignore_nets: Vec<String>,
}

impl Default for ReadOptions {
    fn default() -> Self {
        ReadOptions {
            keep: KeepCopper::Locked,
            zones_as_obstacles: false,
            plane_layers: Vec::new(),
            move_parts: false,
            fixed: Vec::new(),
            ignore_nets: Vec::new(),
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
        kb.board.item_clearance = crate::dru::item_clearances(&kb.board, &rules).into_iter().collect();
        kb.board.net_hole_clearance = crate::dru::hole_clearances(&kb.board, &dru).into_iter().collect();
        crate::dru::apply_width_rules(&mut kb.board, &dru);
        crate::dru::apply_pair_rules(&mut kb.board, &dru);
        kb.board.match_rules = crate::dru::match_rules(&kb.board, &dru);
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
        mask_margin: 0.0,
        mask_copper: 0.0,
        openings: Vec::new(),
    };

    r.read_layers(&root, opts);
    r.read_stackup(&root);
    r.b.thickness = root.child("general").and_then(|g| g.child_f64("thickness")).unwrap_or(1.6);
    r.mask_margin = root
        .child("setup")
        .and_then(|s| s.child_f64("pad_to_mask_clearance"))
        .unwrap_or(0.0);
    r.read_rules(pro);
    r.read_classes(pro);
    r.read_net_table(&root);

    for (i, n) in root.args().iter().enumerate() {
        match n.head() {
            Some("footprint") | Some("module") => r.read_footprint(n),
            Some("segment") | Some("arc") | Some("via") => {
                let first = r.b.obstacles.len();
                r.read_copper(n, opts.keep, &opts.ignore_nets, i as u32);
                if let Some(id) = item_id(n) {
                    r.copper_uuids
                        .insert(id.to_string(), (first..r.b.obstacles.len()).collect());
                }
            }
            Some("zone") => r.read_zone(n, opts),
            Some("gr_rect") | Some("gr_line") | Some("gr_arc") | Some("gr_circle") | Some("gr_poly")
            | Some("gr_curve") => r.read_graphic(n, None),
            Some("gr_text") => r.read_text(n, None),
            _ => {}
        }
    }
    r.finish_outline();
    r.finish_openings();
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
    /// `pad_to_mask_clearance` of the board.
    mask_margin: f64,
    /// `solder_mask_to_copper_clearance` of the rules, on top of the margin.
    mask_copper: f64,
    /// Obstacles that are solder mask openings of their own, see [`Reader::finish_openings`].
    openings: Vec<usize>,
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
    /// Copper and dielectric layers of the stackup, for impedance estimates.
    fn read_stackup(&mut self, root: &Node) {
        let Some(st) = root.child("setup").and_then(|x| x.child("stackup")) else {
            return;
        };
        let number = |l: &Node, k: &str| l.child(k).and_then(|c| c.args().first()).and_then(|v| v.as_f64());
        for l in st.children("layer") {
            let name = l.args().first().and_then(|x| x.as_str()).unwrap_or("");
            let kind = l.child_str("type").unwrap_or("");
            let Some(thickness) = number(l, "thickness") else {
                continue;
            };
            if kind == "copper" {
                self.b.stackup.push(StackItem {
                    copper: self.b.layer_id(name),
                    thickness,
                    epsilon_r: 1.0,
                });
            } else if kind == "core" || kind == "prepreg" {
                self.b.stackup.push(StackItem {
                    copper: None,
                    thickness,
                    epsilon_r: number(l, "epsilon_r").unwrap_or(4.5),
                });
            }
        }
    }

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
        self.mask_copper = g("solder_mask_to_copper_clearance").unwrap_or(0.0);
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
                target_width: None,
                neck_refs: Vec::new(),
                diff_width: f("diff_pair_width", 0.0),
                diff_gap: f("diff_pair_gap", 0.0),
                diff_via_gap: f("diff_pair_via_gap", 0.0),
                ..Default::default()
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
                    target_width: None,
                    neck_refs: Vec::new(),
                    ..Default::default()
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

    /// The outer copper layer below a solder mask layer.
    fn below_mask(&self, layer: &str) -> Option<LayerId> {
        match layer {
            "F.Mask" => self.b.layer_id("F.Cu"),
            "B.Mask" => self.b.layer_id("B.Cu"),
            _ => None,
        }
    }

    /// The outer copper layers below the solder mask layers of a pad.
    fn pad_masks(&self, pad: &Node) -> Vec<LayerId> {
        let on = |name: &str| {
            pad.child("layers")
                .is_some_and(|l| l.args().iter().any(|x| x.as_str() == Some(name)))
        };
        let both = on("*.Mask") || on("F&B.Mask");
        ["F.Mask", "B.Mask"]
            .into_iter()
            .filter(|m| both || on(m))
            .filter_map(|m| self.below_mask(m))
            .collect()
    }

    /// A solder mask opening of its own over copper layer `layer`, with no copper of its own.
    fn push_opening(&mut self, shape: Shape, layer: LayerId) {
        let i = self.push_obstacle(Obstacle {
            shape,
            layers: LayerSet::single(layer),
            net: None,
            kind: ObstacleKind::Keepout {
                vias: true,
                tracks: true,
                mask: true,
            },
            hole: None,
            clearance: 0.0,
            pin: None,
            locked: true,
        });
        self.openings.push(i);
    }

    /// A shape drawn on a mask layer: for the bridge check of KiCad it reaches as far as the
    /// board's margin (not the one of its footprint, measured with KiCad 10) beyond its edge.
    fn push_drawn_opening(&mut self, shape: Shape, layer: LayerId) {
        let reach = (self.mask_margin + self.mask_copper).max(0.0);
        self.push_opening(shape.grown(reach), layer);
    }

    /// KiCad calls an opening a bridge when it shows copper of two nets. So the net of the pads and
    /// graphics in an opening may enter it, and an opening over bare board is closed to all. An
    /// opening with several nets in it is a bridge by itself: no track changes that, and an
    /// obstacle would only cut off the pads, so it is dropped.
    fn finish_openings(&mut self) {
        for i in std::mem::take(&mut self.openings) {
            let opening = &self.b.obstacles[i];
            let bb = opening.shape.bbox();
            let mut nets: Vec<NetId> = Vec::new();
            for o in &self.b.obstacles {
                if matches!(o.kind, ObstacleKind::Pad | ObstacleKind::Graphic)
                    && o.layers.intersects(opening.layers)
                    && o.shape.bbox().overlaps(&bb)
                    && o.shape.dist(&opening.shape) <= 0.0
                    && let Some(n) = o.net
                    && !nets.contains(&n)
                {
                    nets.push(n);
                }
            }
            let opening = &mut self.b.obstacles[i];
            match nets[..] {
                [] => {}
                [n] => opening.net = Some(n),
                _ => {
                    eprintln!("solder mask opening with {} nets in it: no obstacle", nets.len());
                    opening.layers = LayerSet::default();
                    opening.kind = ObstacleKind::Keepout {
                        vias: false,
                        tracks: false,
                        mask: true,
                    };
                }
            }
        }
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
        // openings of a footprint that allows bridges do not count, not even for the tracks
        let bridges = fp
            .child("attr")
            .is_some_and(|a| a.args().iter().any(|x| x.as_str() == Some("allow_soldermask_bridges")));
        let margin = fp.child_f64("solder_mask_margin").filter(|m| *m != 0.0);
        let margin = (!bridges).then_some(margin.unwrap_or(self.mask_margin));
        for pad in fp.children("pad") {
            self.read_pad(pad, pos, rot, &reference, margin);
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
                && !(bridges && self.below_mask(g.child_str("layer").unwrap_or("")).is_some())
            {
                self.read_graphic(g, Some((pos, rot)));
            }
        }
        for t in fp.children("fp_text") {
            if !(bridges && self.below_mask(t.child_str("layer").unwrap_or("")).is_some()) {
                self.read_text(t, Some((pos, rot)));
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
                    ignore_nets: vec![],
                },
            );
        }
        if self.b.obstacles.len() > first_obstacle {
            self.b
                .footprint_obstacles
                .insert(reference, (first_obstacle..self.b.obstacles.len()).collect());
        }
    }

    /// `margin`: the solder mask margin of the footprint, None if its openings do not count.
    fn read_pad(&mut self, pad: &Node, fp_pos: Point, fp_rot: f64, reference: &str, margin: Option<f64>) {
        let a = pad.args();
        let number = a.first().and_then(|x| x.as_str()).unwrap_or("").to_string();
        let kind = a.get(1).and_then(|x| x.as_str()).unwrap_or("smd");
        let shape = a.get(2).and_then(|x| x.as_str()).unwrap_or("rect");
        let at = pad.child_f64s("at").unwrap_or_default();
        let local = pt(at.first().copied().unwrap_or(0.0), at.get(1).copied().unwrap_or(0.0));
        let angle = at.get(2).copied().unwrap_or(0.0);
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
        // (KiCad takes a margin of 0 as not set)
        let margin = margin.map(|m| pad.child_f64("solder_mask_margin").filter(|v| *v != 0.0).unwrap_or(m));
        let masks = self.pad_masks(pad);

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
            // a bare drill opens no mask, a pad wider than the drill does
            if let Some(m) = margin
                && w.min(h) / 2.0 > hole.unwrap_or(r)
                && m + self.mask_copper + MASK_GAP > clearance
            {
                for &l in &masks {
                    self.push_opening(
                        Shape::Circle {
                            c: center,
                            r: r + m + self.mask_copper,
                        },
                        l,
                    );
                }
            }
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

        // a pad without copper opens the mask in its own shape: KiCad gives it no margin
        if layers.is_empty() {
            if margin.is_some() {
                for &l in &masks {
                    for s in &shapes {
                        self.push_opening(s.grown(self.mask_copper), l);
                    }
                }
            }
            return;
        }

        // The mask opening of a pad is the pad grown by its margin, which is what the clearance
        // of the pad describes: copper of another net stays out, its own net gets to the pad as
        // ever, and pads with openings that overlap do not lock each other out. A through hole
        // pad has the one clearance on all layers, a little more than its inner ones need.
        let mut clearance = pad.child_f64("clearance").unwrap_or(0.0).max(self.clearance_of(net));
        if let Some(m) = margin
            && m + self.mask_copper > 0.0
            && masks.iter().any(|&l| layers.has(l))
        {
            clearance = clearance.max(m + self.mask_copper + MASK_GAP);
        }
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
            if let Some(d) = pad.child_f64("die_length").filter(|d| *d > 0.0) {
                self.b.die_length.insert(pin, d);
            }
        }
    }

    fn read_copper(&mut self, n: &Node, keep: KeepCopper, ignore_nets: &[String], source: u32) {
        let locked = n.flag("locked");
        match keep {
            KeepCopper::None => return,
            KeepCopper::Locked if !locked => return,
            _ => {}
        }
        let net = self.net_of(n);
        if !locked && net.is_some_and(|i| ignore_nets.contains(&self.b.nets[i as usize].name)) {
            return;
        }
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
                    kind: ObstacleKind::Keepout {
                        vias,
                        tracks,
                        mask: false,
                    },
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
        let mask = self.below_mask(layer);
        if !is_edge && cu.is_none() && mask.is_none() {
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
            "curve" => {
                if let [a, b, c, d] = pts(g)[..] {
                    polyline = (0..=32)
                        .map(|i| {
                            let t = i as f64 / 32.0;
                            let u = 1.0 - t;
                            map(a * (u * u * u) + b * (3.0 * u * u * t) + c * (3.0 * u * t * t) + d * (t * t * t))
                        })
                        .collect();
                }
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
        if let Some(l) = mask {
            for s in shapes {
                self.push_drawn_opening(s, l);
            }
            return;
        }
        let layer = cu.unwrap();
        let net = self.net_of(g);
        let clearance = self.clearance_of(net);
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

    /// Text on a copper layer is copper, without a net. The router does not need the letters: a
    /// box around the text is an obstacle that is a little too large, from the widths of KiCad's
    /// stroke font, which average about 0.8 of the character size. Text on a solder mask layer
    /// opens the mask, the same box.
    fn read_text(&mut self, t: &Node, tf: Option<(Point, f64)>) {
        let name = t.child_str("layer").unwrap_or("");
        let mask = self.below_mask(name);
        let Some(layer) = self.b.layer_id(name).or(mask) else {
            return;
        };
        // (gr_text "text" ...), (fp_text user "text" ...)
        let which = usize::from(t.head() == Some("fp_text"));
        let Some(text) = t.args().get(which).and_then(|a| a.as_str()) else {
            return;
        };
        let effects = t.child("effects");
        if t.flag("hide") || effects.is_some_and(|e| e.flag("hide")) {
            return;
        }
        let font = effects.and_then(|e| e.child("font"));
        let size = font.and_then(|f| f.child_f64s("size")).unwrap_or_default();
        let (h, w) = (size.first().copied().unwrap_or(1.0), size.get(1).copied().unwrap_or(1.0));
        let stroke = font.and_then(|f| f.child_f64("thickness")).unwrap_or(0.15);
        let lines = text.split(['\n', '\r']).filter(|l| !l.is_empty()).count().max(1);
        let longest = text.split(['\n', '\r']).map(|l| l.chars().count()).max().unwrap_or(0);
        let (width, height) = (
            longest as f64 * w * 0.85 + stroke,
            h + (lines - 1) as f64 * h * 1.6 + stroke,
        );
        let justify = effects.and_then(|e| e.child("justify"));
        let has = |word: &str| justify.is_some_and(|j| j.args().iter().any(|a| a.as_str() == Some(word)));
        let (left, right) = (has("left") != has("mirror"), has("right") != has("mirror"));
        let x0 = if left {
            -stroke / 2.0
        } else if right {
            stroke / 2.0 - width
        } else {
            -width / 2.0
        };
        let y0 = if has("top") {
            -stroke / 2.0
        } else if has("bottom") {
            stroke / 2.0 - height
        } else {
            -height / 2.0
        };
        let at = t.child_f64s("at").unwrap_or_default();
        let origin = pt(at.first().copied().unwrap_or(0.0), at.get(1).copied().unwrap_or(0.0));
        let origin = match tf {
            Some((pos, rot)) => pos + kicad_rot(origin, rot),
            None => origin,
        };
        let angle = at.get(2).copied().unwrap_or(0.0);
        let corner = |x: f64, y: f64| origin + kicad_rot(pt(x, y), angle);
        let shape = Shape::Polygon(Polygon::new(vec![
            corner(x0, y0),
            corner(x0 + width, y0),
            corner(x0 + width, y0 + height),
            corner(x0, y0 + height),
        ]));
        if mask.is_some() {
            self.push_drawn_opening(shape, layer);
            return;
        }
        let clearance = self.clearance_of(None);
        self.push_obstacle(Obstacle {
            shape,
            layers: LayerSet::single(layer),
            net: None,
            kind: ObstacleKind::Graphic,
            hole: None,
            clearance,
            pin: None,
            locked: true,
        });
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Text on copper blocks the router, text elsewhere does not.
    #[test]
    fn text_on_copper_is_an_obstacle() {
        let text = r#"(kicad_pcb (version 20240108) (generator "test")
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (37 "F.SilkS" user))
  (net 0 "")
  (gr_text "ABCD" (at 10 20 0) (layer "F.Cu")
    (effects (font (size 1 1) (thickness 0.2)) (justify left)))
  (gr_text "HIDDEN" (at 10 30 0) (layer "F.Cu") (effects (font (size 1 1)) (hide yes)))
  (gr_text "SILK" (at 10 40 0) (layer "F.SilkS") (effects (font (size 1 1))))
)"#;
        let kb = parse_board(text.to_string(), None, &ReadOptions::default()).unwrap();
        let shapes: Vec<_> = kb.board.obstacles.iter().filter(|o| o.kind == ObstacleKind::Graphic).collect();
        assert_eq!(shapes.len(), 1);
        let bb = shapes[0].shape.bbox();
        assert!((bb.min.x - 9.9).abs() < 1e-9 && (bb.max.x - (9.9 + 3.6)).abs() < 1e-9, "{bb:?}");
        assert!((bb.min.y - 19.4).abs() < 1e-9 && (bb.max.y - 20.6).abs() < 1e-9, "{bb:?}");
    }

    fn mask_board(body: &str) -> Board {
        let text = format!(
            r#"(kicad_pcb (version 20221018) (generator "test")
            (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (38 "B.Mask" user) (39 "F.Mask" user))
            (setup (pad_to_mask_clearance 0.3))
            (net 0 "") (net 1 "A") (net 2 "B")
            {body})"#
        );
        parse_board(text, None, &ReadOptions::default()).unwrap().board
    }

    fn pad_clearance(b: &Board, reference: &str) -> f64 {
        let pin = b.pins.iter().find(|p| p.footprint == reference).unwrap();
        b.obstacles[pin.obstacles[0]].clearance
    }

    fn openings(b: &Board) -> Vec<&Obstacle> {
        let mask = |o: &&Obstacle| matches!(o.kind, ObstacleKind::Keepout { mask: true, .. }) && !o.layers.is_empty();
        b.obstacles.iter().filter(mask).collect()
    }

    const PAD: &str = r#"(pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Mask") (net 1 "A"))"#;

    fn footprint(reference: &str, at: &str, inner: &str) -> String {
        format!(
            r#"(footprint "t" (layer "F.Cu") (at {at}) (fp_text reference "{reference}" (at 0 0) (layer "F.Fab")) {inner})"#
        )
    }

    /// The opening of a pad is the pad and its margin: pad, then footprint, then board. The
    /// default clearance of 0.2 stays when the margin is smaller, and so do pads without opening.
    #[test]
    fn the_mask_margin_of_a_pad_is_a_clearance() {
        let pads = [
            footprint("BOARD", "10 10", PAD),
            footprint("FP", "20 10", &format!("(solder_mask_margin 0.4) {PAD}")),
            footprint(
                "ZERO",
                "30 10",
                r#"(pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Mask") (solder_mask_margin 0) (net 1 "A"))"#,
            ),
            footprint(
                "PAD",
                "40 10",
                r#"(solder_mask_margin 0.4) (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Mask") (solder_mask_margin 0.5) (net 1 "A"))"#,
            ),
            footprint("SMALL", "50 10", &format!("(solder_mask_margin 0.05) {PAD}")),
            footprint(
                "BRIDGES",
                "60 10",
                &format!("(attr smd allow_soldermask_bridges) {PAD}"),
            ),
            footprint(
                "BARE",
                "70 10",
                r#"(pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "A"))"#,
            ),
            footprint(
                "BACK",
                "80 10",
                r#"(pad "1" thru_hole circle (at 0 0) (size 1 1) (drill 0.5) (layers "*.Cu" "B.Mask") (net 1 "A"))"#,
            ),
        ];
        let b = mask_board(&pads.concat());
        let want = [
            ("BOARD", 0.31),
            ("FP", 0.41),
            ("ZERO", 0.31),
            ("PAD", 0.51),
            ("SMALL", 0.2),
            ("BRIDGES", 0.2),
            ("BARE", 0.2),
            ("BACK", 0.31),
        ];
        for (reference, clearance) in want {
            assert!((pad_clearance(&b, reference) - clearance).abs() < 1e-9, "{reference}");
        }
        assert!(openings(&b).is_empty());
    }

    /// Mask shapes: closed to everyone over bare board, open to the net of the copper in them,
    /// no obstacle where they show two nets already or the footprint allows bridges.
    #[test]
    fn shapes_on_the_mask_are_openings() {
        let square = |layer: &str, head: &str| {
            format!(
                r#"({head} (pts (xy -2 -2) (xy 2 -2) (xy 2 2) (xy -2 2)) (layer "{layer}") (width 0) (fill solid))"#
            )
        };
        let second = r#"(pad "2" smd rect (at 1 0) (size 0.5 0.5) (layers "F.Cu") (net 2 "B"))"#;
        let parts = [
            format!(
                "(gr_poly (pts (xy 8 8) (xy 12 8) (xy 12 12) (xy 8 12)) (layer \"B.Mask\") (width 0) (fill solid))"
            ),
            r#"(gr_line (start 20 10) (end 24 10) (layer "F.Mask") (width 0.2))"#.to_string(),
            r#"(gr_line (start 20 20) (end 24 20) (layer "F.SilkS") (width 0.2))"#.to_string(),
            r#"(gr_text "AB" (at 30 10 0) (layer "F.Mask") (effects (font (size 1 1) (thickness 0.2))))"#.to_string(),
            footprint("EMPTY", "40 10", &square("F.Mask", "fp_poly")),
            footprint("ONE", "50 10", &format!("{PAD} {}", square("F.Mask", "fp_poly"))),
            footprint(
                "TWO",
                "60 10",
                &format!("{PAD} {second} {}", square("F.Mask", "fp_poly")),
            ),
            footprint(
                "BRIDGES",
                "70 10",
                &format!("(attr smd allow_soldermask_bridges) {}", square("F.Mask", "fp_poly")),
            ),
            footprint(
                "HOLE",
                "80 10",
                r#"(pad "" np_thru_hole circle (at 0 0) (size 2 2) (drill 1) (layers "*.Cu" "*.Mask"))"#,
            ),
            footprint(
                "DRILL",
                "90 10",
                r#"(pad "" np_thru_hole circle (at 0 0) (size 1 1) (drill 1) (layers "*.Cu" "*.Mask"))"#,
            ),
            footprint(
                "FREE",
                "100 10",
                r#"(pad "" smd rect (at 0 0) (size 1 1) (layers "F.Mask"))"#,
            ),
        ];
        let b = mask_board(&parts.concat());
        let seen: Vec<_> = openings(&b)
            .iter()
            .map(|o| (o.shape.bbox().center().x.round() as i64, o.layers, o.net))
            .collect();
        let (top, bottom) = (LayerSet::single(0), LayerSet::single(1));
        let a = Some(b.net_id("A").unwrap());
        let want = [
            (10, bottom, None),
            (22, top, None),
            (30, top, None),
            (40, top, None),
            (50, top, a),
            (80, top, None),
            (80, bottom, None),
            (100, top, None),
        ];
        assert_eq!(seen.len(), want.len(), "{seen:?}");
        for w in want {
            assert!(seen.contains(&w), "{w:?} in {seen:?}");
        }
        // a drawn shape reaches as far as the board's margin of 0.3, a pad without copper does not
        let bb = |x: f64| {
            openings(&b)
                .iter()
                .find(|o| (o.shape.bbox().center().x - x).abs() < 1.0)
                .unwrap()
                .shape
                .bbox()
        };
        assert!((bb(10.0).width() - 4.6).abs() < 1e-9 && (bb(40.0).width() - 4.6).abs() < 1e-9);
        assert!((bb(100.0).width() - 1.0).abs() < 1e-9);
        // they belong to the footprint, so that they move with it
        let of = |r: &str| b.footprint_obstacles.get(r).map_or(0, |o| o.len());
        assert_eq!((of("EMPTY"), of("HOLE"), of("BRIDGES")), (1, 3, 0));
    }

    /// A pad's angle in the file is absolute, not relative to the footprint, and KiCad leaves it
    /// out when it is 0: the pad of a footprint at -90 degrees whose local angle is 90.
    #[test]
    fn missing_pad_angle_means_zero_in_a_rotated_footprint() {
        let text = r#"(kicad_pcb (version 20240108) (generator "test")
            (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
            (net 0 "") (net 1 "A") (net 2 "B")
            (footprint "test" (layer "F.Cu") (at 10 10 -90)
                (pad "1" smd roundrect (at -1 0) (size 0.3 0.7) (layers "F.Cu") (net 1 "A"))
                (pad "2" smd roundrect (at 1 0 270) (size 0.3 0.7) (layers "F.Cu") (net 2 "B"))))"#;
        let kb = parse_board(text.to_string(), None, &ReadOptions::default()).unwrap();
        let size = |name: &str| kb.board.pins.iter().find(|p| p.name.ends_with(name)).unwrap().size;
        let (w, h) = size(".1");
        assert!((w - 0.3).abs() < 1e-3 && (h - 0.7).abs() < 1e-3, "{w} x {h}");
        let (w, h) = size(".2");
        assert!((w - 0.7).abs() < 1e-3 && (h - 0.3).abs() < 1e-3, "{w} x {h}");
    }
}
