//! Board model: everything the router needs to know, independent of the file format.

use crate::geom::{Point, Polygon, Rect, Shape};

/// Distance kept from rule areas: KiCad flags a track that merely touches one.
pub const KEEPOUT_GAP: f64 = 0.02;

pub type LayerId = u8;
pub type NetId = u32;
pub type ClassId = u16;

/// Set of copper layers as a bit mask (bit i = copper layer index i).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, serde::Serialize, serde::Deserialize)]
pub struct LayerSet(pub u64);

impl LayerSet {
    pub const fn single(l: LayerId) -> LayerSet {
        LayerSet(1 << l)
    }
    pub fn all(n: usize) -> LayerSet {
        LayerSet(if n >= 64 { u64::MAX } else { (1u64 << n) - 1 })
    }
    pub fn has(self, l: LayerId) -> bool {
        self.0 & (1 << l) != 0
    }
    pub fn insert(&mut self, l: LayerId) {
        self.0 |= 1 << l;
    }
    pub fn intersects(self, o: LayerSet) -> bool {
        self.0 & o.0 != 0
    }
    pub fn iter(self) -> impl Iterator<Item = LayerId> {
        (0..64u8).filter(move |&l| self.has(l))
    }
    pub fn is_empty(self) -> bool {
        self.0 == 0
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Layer {
    pub name: String,
    /// Inner layers used as planes are not routed on.
    pub is_plane: bool,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct NetClass {
    pub name: String,
    pub track_width: f64,
    pub clearance: f64,
    pub via_diameter: f64,
    pub via_drill: f64,
    /// Lower value = routed first (KiCad net class priority convention).
    pub priority: i32,
    /// Minimum track width from custom rules (necking below the class width is limited by it).
    #[serde(default)]
    pub min_width: Option<f64>,
    /// Footprints (references) inside whose courtyard the minimum width does not apply.
    #[serde(default)]
    pub neck_refs: Vec<String>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Net {
    pub name: String,
    pub class: ClassId,
    pub pins: Vec<usize>,
    /// Inner layers that carry a plane of this net (connect pins by fan-out vias).
    pub planes: LayerSet,
    /// Routing order; lower first. Defaults to the class priority.
    pub priority: i32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ObstacleKind {
    Pad,
    Track,
    Via,
    /// Rule area: no tracks (and maybe no vias) on its layers.
    Keepout {
        vias: bool,
        tracks: bool,
    },
    /// Copper zone fill or outline.
    Zone,
    /// Copper graphics without net.
    Graphic,
    /// Drilled hole without copper (NPTH).
    Hole,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Obstacle {
    pub shape: Shape,
    pub layers: LayerSet,
    pub net: Option<NetId>,
    pub kind: ObstacleKind,
    /// Drill radius if the obstacle has a hole (THT pad, via, NPTH).
    pub hole: Option<f64>,
    /// Clearance the obstacle itself demands (from its net class), at least the board minimum.
    pub clearance: f64,
    /// Index of the pin if this obstacle is (part of) a pad.
    pub pin: Option<usize>,
    pub locked: bool,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Pin {
    /// "R8.1"
    pub name: String,
    /// Footprint reference, "R8".
    pub footprint: String,
    pub net: NetId,
    pub pos: Point,
    pub layers: LayerSet,
    /// Obstacles that make up the pad copper.
    pub obstacles: Vec<usize>,
    /// Size of the pad bounding box, for fan-out decisions.
    pub size: (f64, f64),
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Rules {
    pub min_clearance: f64,
    pub min_track_width: f64,
    pub edge_clearance: f64,
    pub hole_to_hole: f64,
    pub hole_clearance: f64,
    pub min_via_diameter: f64,
    /// Polygon approximation error for round shapes.
    pub max_error: f64,
}

impl Default for Rules {
    fn default() -> Self {
        Rules {
            min_clearance: 0.15,
            min_track_width: 0.15,
            edge_clearance: 0.3,
            hole_to_hole: 0.25,
            hole_clearance: 0.2,
            min_via_diameter: 0.45,
            max_error: 0.005,
        }
    }
}

/// Plane area of a net on an inner layer. Vias of the net inside it connect to the plane.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Plane {
    pub net: NetId,
    pub layer: LayerId,
    pub outline: Polygon,
}

#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct Board {
    pub layers: Vec<Layer>,
    pub outline: Vec<Polygon>,
    pub obstacles: Vec<Obstacle>,
    pub pins: Vec<Pin>,
    pub planes: Vec<Plane>,
    pub nets: Vec<Net>,
    pub classes: Vec<NetClass>,
    pub rules: Rules,
    /// Courtyard bounding boxes by footprint reference.
    pub courtyards: std::collections::HashMap<String, Rect>,
    /// Track to track clearances of custom rules, above the net class clearance (a < b).
    pub pair_clearance: crate::index::FastMap<(NetId, NetId), f64>,
    /// Obstacles (pads, holes, copper graphics) by footprint reference.
    #[serde(default)]
    pub footprint_obstacles: std::collections::HashMap<String, Vec<usize>>,
    /// Footprints the router may shift a little (empty: placement is fixed).
    #[serde(default)]
    pub movable: Vec<String>,
    /// Groups of parts with their pre-routed copper that move only as a whole.
    #[serde(default)]
    pub blocks: Vec<Block>,
}

/// A fixed block: a KiCad group of footprints and copper.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct Block {
    pub name: String,
    /// Footprint references.
    pub refs: Vec<String>,
    /// Obstacles of the group's kept tracks and vias.
    pub copper: Vec<usize>,
    /// File ids of the group's tracks, arcs and vias, for the writer.
    pub uuids: Vec<String>,
}

impl Board {
    /// The board with footprints shifted by the given offsets (pads, holes, graphics, pins and
    /// courtyards move together).
    pub fn with_offsets(&self, offsets: &std::collections::HashMap<String, Point>) -> Board {
        let mut b = self.clone();
        for block in &self.blocks {
            if let Some(&d) = block.refs.iter().find_map(|r| offsets.get(r)) {
                for &i in &block.copper {
                    b.obstacles[i].shape = b.obstacles[i].shape.translated(d);
                }
            }
        }
        for (fp, &d) in offsets {
            if let Some(obs) = self.footprint_obstacles.get(fp) {
                for &i in obs {
                    b.obstacles[i].shape = b.obstacles[i].shape.translated(d);
                }
            }
            for p in b.pins.iter_mut().filter(|p| &p.footprint == fp) {
                p.pos = p.pos + d;
            }
            if let Some(r) = b.courtyards.get_mut(fp) {
                *r = Rect {
                    min: r.min + d,
                    max: r.max + d,
                };
            }
        }
        b
    }

    /// Kept tracks and vias that touch no pad of their net are pieces of a route in progress:
    /// each such island becomes a pin of the net, so that the router connects it.
    pub fn add_island_pins(&mut self) {
        let kept: Vec<usize> = (0..self.obstacles.len())
            .filter(|&i| matches!(self.obstacles[i].kind, ObstacleKind::Track | ObstacleKind::Via))
            .collect();
        let touches = |a: &Obstacle, b: &Obstacle| {
            a.net == b.net && a.layers.intersects(b.layers) && a.shape.dist(&b.shape) <= 0.0
        };
        let mut seen = vec![false; self.obstacles.len()];
        let mut islands: Vec<Vec<usize>> = Vec::new();
        for &start in &kept {
            if seen[start] {
                continue;
            }
            seen[start] = true;
            let (mut stack, mut members, mut on_pin) = (vec![start], vec![start], false);
            while let Some(o) = stack.pop() {
                let a = &self.obstacles[o];
                let Some(net) = a.net else { continue };
                let bb = a.shape.bbox();
                on_pin |= self.nets[net as usize].pins.iter().any(|&p| {
                    self.pins[p].obstacles.iter().any(|&q| {
                        let b = &self.obstacles[q];
                        b.shape.bbox().overlaps(&bb) && touches(a, b)
                    })
                });
                for &q in &kept {
                    if !seen[q] && self.obstacles[q].net == a.net {
                        let b = &self.obstacles[q];
                        if b.shape.bbox().overlaps(&bb) && touches(a, b) {
                            seen[q] = true;
                            stack.push(q);
                            members.push(q);
                        }
                    }
                }
            }
            if !on_pin && self.obstacles[start].net.is_some() {
                islands.push(members);
            }
        }
        for members in islands {
            let first = &self.obstacles[members[0]];
            let net = first.net.unwrap();
            let bb = first.shape.bbox();
            let id = self.pins.len();
            let mut layers = LayerSet::default();
            for &m in &members {
                layers.0 |= self.obstacles[m].layers.0;
                self.obstacles[m].pin = Some(id);
            }
            self.pins.push(Pin {
                name: format!("copper{id}"),
                footprint: String::new(),
                net,
                pos: (bb.min + bb.max) * 0.5,
                layers,
                obstacles: members,
                size: (bb.width().min(bb.height()), bb.width().max(bb.height())),
            });
            self.nets[net as usize].pins.push(id);
        }
    }

    /// Differential pairs by net name, as KiCad pairs them: the same name with `_P`/`_N`,
    /// `P`/`N`, `+`/`-` at the end (so `DP`/`DN` as well). Both nets need 2 or more pins.
    pub fn diff_pairs(&self) -> Vec<(NetId, NetId)> {
        let mut out = Vec::new();
        for (i, n) in self.nets.iter().enumerate() {
            if n.pins.len() < 2 {
                continue;
            }
            for (a, b) in [('P', 'N'), ('+', '-')] {
                if let Some(stem) = n.name.strip_suffix(a)
                    && let Some(j) = self.net_id(&format!("{stem}{b}"))
                    && self.nets[j as usize].pins.len() >= 2
                {
                    out.push((i as NetId, j));
                }
            }
        }
        out
    }

    /// The board for routing only `nets` and `given` (all nets if `nets` is empty). The unlocked
    /// kept copper of `given` is gone; the other nets get no connections and stay as obstacles.
    pub fn only_nets(&self, nets: &[String], given: &std::collections::BTreeSet<String>) -> Board {
        let mut b = self.clone();
        if nets.is_empty() {
            return b;
        }
        let ids: Vec<bool> = b.nets.iter().map(|n| given.contains(&n.name)).collect();
        for o in b.obstacles.iter_mut() {
            if !o.locked
                && matches!(o.kind, ObstacleKind::Track | ObstacleKind::Via)
                && o.net.is_some_and(|n| ids[n as usize])
            {
                o.layers = LayerSet::default();
            }
        }
        let pins = &b.pins;
        for n in b.nets.iter_mut() {
            if nets.contains(&n.name) {
                continue;
            }
            if given.contains(&n.name) {
                n.pins.retain(|&p| !pins[p].footprint.is_empty());
            } else {
                n.pins.clear();
            }
        }
        b
    }

    /// Nets with unlocked tracks within `radius` of the pins of unrouted connections, given as
    /// "NET from PIN".
    pub fn nets_near_pins(&self, unrouted: &[String], radius: f64) -> Vec<String> {
        let mut out = Vec::new();
        for u in unrouted {
            let Some(pin) = u
                .rsplit(" from ")
                .next()
                .and_then(|n| self.pins.iter().find(|p| p.name == n))
            else {
                continue;
            };
            let targets = self.nets[pin.net as usize].pins.iter().map(|&p| self.pins[p].pos);
            for at in std::iter::once(pin.pos).chain(targets) {
                for o in &self.obstacles {
                    if !o.locked
                        && matches!(o.kind, ObstacleKind::Track | ObstacleKind::Via)
                        && o.shape.dist_to_point(at) < radius
                        && let Some(n) = o.net
                    {
                        let name = self.nets[n as usize].name.clone();
                        // plane nets stay: they have hundreds of small stubs and vias
                        let plane = self.planes.iter().any(|p| p.net == n);
                        if n != pin.net && !plane && !out.contains(&name) {
                            out.push(name);
                        }
                    }
                }
            }
        }
        out
    }

    /// Give one net its own track width (a copy of its class with that width; the class's
    /// other nets keep theirs).
    pub fn set_net_width(&mut self, net: NetId, width: f64) {
        let ci = self.nets[net as usize].class as usize;
        if (self.classes[ci].track_width - width).abs() < 1e-9 {
            return;
        }
        let mut c = self.classes[ci].clone();
        c.name = format!("{}@{width}", c.name);
        c.track_width = width;
        if let Some(i) = self.classes.iter().position(|k| k.name == c.name) {
            self.nets[net as usize].class = i as ClassId;
        } else {
            self.classes.push(c);
            self.nets[net as usize].class = (self.classes.len() - 1) as ClassId;
        }
    }

    pub fn layer_id(&self, name: &str) -> Option<LayerId> {
        self.layers.iter().position(|l| l.name == name).map(|i| i as LayerId)
    }

    pub fn all_layers(&self) -> LayerSet {
        LayerSet::all(self.layers.len())
    }

    /// Copper layers tracks may be drawn on.
    pub fn routing_layers(&self) -> LayerSet {
        let mut s = LayerSet::default();
        for (i, l) in self.layers.iter().enumerate() {
            if !l.is_plane {
                s.insert(i as LayerId);
            }
        }
        s
    }

    pub fn net_id(&self, name: &str) -> Option<NetId> {
        self.nets.iter().position(|n| n.name == name).map(|i| i as NetId)
    }

    pub fn class_of(&self, net: NetId) -> &NetClass {
        &self.classes[self.nets[net as usize].class as usize]
    }

    /// Clearance between copper of two nets: the larger of both class clearances.
    pub fn clearance(&self, a: Option<NetId>, b: Option<NetId>) -> f64 {
        let ca = a.map_or(0.0, |n| self.class_of(n).clearance);
        let cb = b.map_or(0.0, |n| self.class_of(n).clearance);
        ca.max(cb).max(self.rules.min_clearance)
    }

    /// Clearance between tracks of two nets, custom rules included.
    pub fn track_clearance(&self, a: NetId, b: NetId) -> f64 {
        let base = self.clearance(Some(a), Some(b));
        let key = (a.min(b), a.max(b));
        self.pair_clearance.get(&key).map_or(base, |&c| c.max(base))
    }

    pub fn bbox(&self) -> Rect {
        let mut r = Rect::empty();
        for p in &self.outline {
            for &q in &p.pts {
                r.add(q);
            }
        }
        r
    }

    /// Main outline: the polygon with the largest area.
    pub fn main_outline(&self) -> Option<&Polygon> {
        self.outline
            .iter()
            .max_by(|a, b| a.signed_area().abs().total_cmp(&b.signed_area().abs()))
    }

    /// True if p lies inside the board: inside an odd number of outline polygons (a board may
    /// consist of several pieces, cut-outs lie inside a piece, islands inside cut-outs).
    pub fn inside(&self, p: Point) -> bool {
        if self.outline.is_empty() {
            return true;
        }
        self.outline.iter().filter(|o| o.contains(p)).count() % 2 == 1
    }

    /// Distance from p to the board edge (0 outside).
    pub fn edge_dist(&self, p: Point) -> f64 {
        if !self.inside(p) {
            return 0.0;
        }
        self.outline
            .iter()
            .map(|o| o.boundary_dist(p))
            .fold(f64::INFINITY, f64::min)
    }

    /// Nets that need routing: two or more pins.
    pub fn routable_nets(&self) -> impl Iterator<Item = NetId> + '_ {
        self.nets
            .iter()
            .enumerate()
            .filter(|(_, n)| n.pins.len() >= 2)
            .map(|(i, _)| i as NetId)
    }
}
