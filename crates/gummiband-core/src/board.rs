//! Board model: everything the router needs to know, independent of the file format.

use serde::{Deserialize, Serialize};

use crate::geom::{Point, Polygon, Rect, Shape};

/// Distance kept from rule areas: KiCad flags a track that merely touches one.
pub const KEEPOUT_GAP: f64 = 0.02;
/// Copper thickness of a layer without a stackup (1 oz).
pub const COPPER_MM: f64 = 0.035;
/// Plating of a via barrel (IPC class 2 average).
pub const PLATING_MM: f64 = 0.025;

/// Cross-section of a via barrel of `drill` mm in mm².
fn barrel_mm2(drill: f64) -> f64 {
    std::f64::consts::PI * (drill + PLATING_MM) * PLATING_MM
}

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

#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
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
    /// Width the tracks of the class grow to where there is room, from the `opt` of a custom
    /// track width rule (the class width stays the width they are routed with).
    #[serde(default)]
    pub target_width: Option<f64>,
    /// Footprints (references) inside whose courtyard the minimum width does not apply.
    #[serde(default)]
    pub neck_refs: Vec<String>,
    /// Track width, gap and via gap of differential pairs (0: not set).
    #[serde(default)]
    pub diff_width: f64,
    #[serde(default)]
    pub diff_gap: f64,
    #[serde(default)]
    pub diff_via_gap: f64,
    /// Width and gap of differential pairs on outer and on inner layers, from custom rules; they
    /// win over the values above.
    #[serde(default)]
    pub diff_outer: Option<(f64, f64)>,
    #[serde(default)]
    pub diff_inner: Option<(f64, f64)>,
    /// Largest length difference within a pair, from custom rules.
    #[serde(default)]
    pub diff_skew: Option<f64>,
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
        /// A solder mask opening, a rule area on the outer copper layer below it. It may have a
        /// net: the net of the copper that lies in it, which is free to enter (see
        /// [`crate::topo::Mesh::opening`]).
        #[serde(default)]
        mask: bool,
    },
    /// Copper zone fill or outline.
    Zone,
    /// A pour laid out before routing (a "gb:" zone of `power_copper`): like a zone, and the
    /// pads under it count as joined.
    Pour,
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
    /// Zone priority (KiCad fills a higher priority zone first).
    #[serde(default)]
    pub priority: i64,
    /// Outlines of other nets' zones on the layer with a higher priority: KiCad fills them
    /// first, so the plane has no copper there.
    #[serde(default)]
    pub cut: Vec<Polygon>,
}

impl Plane {
    pub fn new(net: NetId, layer: LayerId, outline: Polygon) -> Plane {
        Plane {
            net,
            layer,
            outline,
            priority: 0,
            cut: Vec::new(),
        }
    }

    /// Whether p lies in the copper of the plane: inside its outline and outside the zones of
    /// other nets that KiCad fills first.
    pub fn covers(&self, p: Point) -> bool {
        self.outline.contains(p) && !self.cut.iter().any(|c| c.contains(p))
    }

    /// Distance from p to the nearest border of the plane copper (its outline or a cut).
    pub fn edge_dist(&self, p: Point) -> f64 {
        self.cut
            .iter()
            .map(|c| c.boundary_dist(p))
            .fold(self.outline.boundary_dist(p), f64::min)
    }

    /// p in the plane copper with at least `margin` to its borders.
    pub fn holds(&self, p: Point, margin: f64) -> bool {
        self.covers(p) && self.edge_dist(p) >= margin
    }
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
    /// Courtyard outlines by footprint reference, where the courtyard lines close into
    /// polygons (an L or T shaped module courtyard is much smaller than its box).
    #[serde(default)]
    pub courtyard_shapes: std::collections::HashMap<String, Vec<Polygon>>,
    /// Track to track clearances of custom rules, above the net class clearance (a < b).
    pub pair_clearance: crate::index::FastMap<(NetId, NetId), f64>,
    /// Clearances of custom rules for copper of any kind, above the net class clearance (a < b).
    #[serde(default)]
    pub item_clearance: crate::index::FastMap<(NetId, NetId), f64>,
    /// Hole clearances of custom rules per net, above the one of the board.
    #[serde(default)]
    pub net_hole_clearance: crate::index::FastMap<NetId, f64>,
    /// Obstacles (pads, holes, copper graphics) by footprint reference.
    #[serde(default)]
    pub footprint_obstacles: std::collections::HashMap<String, Vec<usize>>,
    /// Footprints the router may shift a little (empty: placement is fixed).
    #[serde(default)]
    pub movable: Vec<String>,
    /// Groups of parts with their pre-routed copper that move only as a whole.
    #[serde(default)]
    pub blocks: Vec<Block>,
    /// Kept tracks and vias as the file has them.
    #[serde(default)]
    pub copper: Vec<CopperItem>,
    /// Unlocked copper that is part of the topology instead of an obstacle, see [`Board::soften`].
    #[serde(skip)]
    pub soft: crate::import::SoftCopper,
    /// Stackup from top to bottom, if the board has one.
    #[serde(default)]
    pub stackup: Vec<StackItem>,
    /// Board thickness from the file (`general`), for via lengths without a stackup.
    #[serde(default)]
    pub thickness: f64,
    /// Pad-to-die lengths of pins that have one, by pin index.
    #[serde(default)]
    pub die_length: std::collections::HashMap<usize, f64>,
    /// Length, skew and uncoupled length rules of the `.kicad_dru`, with the nets they hold for.
    #[serde(default)]
    pub match_rules: Vec<MatchRule>,
}

/// A min, opt and max value of a custom rule constraint.
#[derive(Debug, Clone, Copy, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct MinOptMax {
    pub min: Option<f64>,
    pub opt: Option<f64>,
    pub max: Option<f64>,
}

/// What a [`MatchRule`] constrains.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum MatchKind {
    /// Length of every net (`length`).
    Length,
    /// Length difference to the longest net of the rule (`skew`), or within each differential
    /// pair of the rule (`skew ... (within_diff_pairs)`).
    Skew,
    /// Length of a differential pair that does not run coupled (`diff_pair_uncoupled`).
    Uncoupled,
    /// Gap within a differential pair (`diff_pair_gap`): only a gap inside its min and max counts
    /// as coupled.
    Gap,
}

/// One constraint of a custom rule on lengths, as KiCad groups it: every net whose items the rule
/// governs for this kind of constraint (the last rule in the file whose condition holds wins).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct MatchRule {
    pub name: String,
    pub kind: MatchKind,
    pub value: MinOptMax,
    /// Skew within each differential pair instead of across the whole group.
    #[serde(default)]
    pub within_pairs: bool,
    pub nets: Vec<NetId>,
}

/// A layer of the stackup: copper (with its layer) or dielectric.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct StackItem {
    pub copper: Option<LayerId>,
    pub thickness: f64,
    pub epsilon_r: f64,
}

/// One kept track, arc or via of the file.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CopperItem {
    pub kind: CopperKind,
    pub net: Option<NetId>,
    pub locked: bool,
    /// Its place among the items of the file, for the writer.
    pub source: u32,
    /// Its obstacles.
    pub first: u32,
    pub last: u32,
    /// Member of a length tuning pattern (KiCad leaves those out of the diff pair coupling).
    #[serde(default)]
    pub pattern: bool,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum CopperKind {
    Line {
        a: Point,
        b: Point,
        width: f64,
        layer: LayerId,
    },
    Arc {
        a: Point,
        mid: Point,
        b: Point,
        width: f64,
        layer: LayerId,
    },
    Via {
        at: Point,
        diameter: f64,
        drill: f64,
        layers: LayerSet,
    },
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
            if let Some(polys) = b.courtyard_shapes.get_mut(fp) {
                for q in polys.iter_mut() {
                    for p in q.pts.iter_mut() {
                        *p = *p + d;
                    }
                }
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

    /// Track width, gap and via gap of a differential pair on a layer: custom rules, then the net
    /// class, then track width and clearance. The gap is never below the clearance of the nets.
    pub fn pair_shape(&self, p: NetId, n: NetId, layer: LayerId) -> (f64, f64, f64) {
        let (cp, cn) = (self.class_of(p), self.class_of(n));
        let outer = layer == 0 || layer as usize + 1 == self.layers.len();
        let (w, g) = if outer { cp.diff_outer } else { cp.diff_inner }.unwrap_or((0.0, 0.0));
        let set = |x: f64, d: f64| if x > 0.0 { x } else { d };
        let (w, g) = (set(w, set(cp.diff_width, cp.track_width)), set(g, cp.diff_gap));
        let clr = self.track_clearance(p, n).max(cn.clearance);
        // a little above: exactly the clearance is a coin flip after rounding to the file's grid
        let g = g.max(clr + 0.005);
        (w, g, set(cp.diff_via_gap, g).max(clr + 0.005))
    }

    /// Differential impedance of a pair on a layer in ohm, from the stackup: IPC-2141 formulas,
    /// microstrip on outer layers, symmetric stripline on inner ones. Good to a few percent;
    /// a field solver or the manufacturer has the last word.
    pub fn zdiff(&self, layer: LayerId, w: f64, g: f64) -> Option<f64> {
        let at = self.stackup.iter().position(|s| s.copper == Some(layer))?;
        let t = self.stackup[at].thickness;
        // dielectric up to the next copper on each side
        let side = |it: &mut dyn Iterator<Item = &StackItem>| -> Option<(f64, f64)> {
            let (mut h, mut e) = (0.0, 0.0);
            for s in it {
                if s.copper.is_some() {
                    return (h > 0.0).then_some((h, e / h));
                }
                h += s.thickness;
                e += s.thickness * s.epsilon_r;
            }
            None
        };
        let up = side(&mut self.stackup[..at].iter().rev());
        let down = side(&mut self.stackup[at + 1..].iter());
        match (up, down) {
            (Some((h1, e1)), Some((h2, e2))) => {
                let b = h1 + h2 + t;
                let er = (h1 * e1 + h2 * e2) / (h1 + h2);
                let z0 = 60.0 / er.sqrt() * (1.9 * b / (0.8 * w + t)).ln();
                Some(2.0 * z0 * (1.0 - 0.347 * (-2.9 * g / b).exp()))
            }
            (Some((h, er)), None) | (None, Some((h, er))) => {
                let z0 = 87.0 / (er + 1.41).sqrt() * (5.98 * h / (0.8 * w + t)).ln();
                Some(2.0 * z0 * (1.0 - 0.48 * (-0.96 * g / h).exp()))
            }
            (None, None) => None,
        }
    }

    /// Track width that gives `target` ohm on a layer with gap g.
    pub fn width_for_zdiff(&self, layer: LayerId, g: f64, target: f64, min_w: f64) -> Option<f64> {
        let (mut lo, mut hi) = (min_w, 2.0);
        if self.zdiff(layer, lo, g)? < target {
            return Some(lo);
        }
        for _ in 0..40 {
            let mid = (lo + hi) / 2.0;
            if self.zdiff(layer, mid, g)? > target {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        // the writer's grid is fine enough, a board house's is not finer than a micrometre
        Some((lo * 1000.0).round() / 1000.0)
    }

    /// Distance between two copper layers through the board, as KiCad counts the length of a via
    /// (`BOARD_STACKUP::GetLayerDistance`): the copper of both layers (half of an inner one) and the
    /// dielectric between them. Without a stackup the board thickness is shared out evenly.
    pub fn layer_distance(&self, a: LayerId, b: LayerId) -> f64 {
        if a == b {
            return 0.0;
        }
        let (a, b) = (a.min(b), a.max(b));
        let last = self.layers.len().saturating_sub(1) as LayerId;
        let outer = |l: LayerId| l == 0 || l == last;
        let ia = self.stackup.iter().position(|s| s.copper == Some(a));
        let ib = self.stackup.iter().position(|s| s.copper == Some(b));
        if let (Some(ia), Some(ib)) = (ia, ib) {
            let mut total = 0.0;
            for (i, s) in self.stackup.iter().enumerate().take(ib + 1).skip(ia) {
                let half = (i == ia && !outer(a)) || (i == ib && !outer(b));
                total += if half { s.thickness / 2.0 } else { s.thickness };
            }
            return total;
        }
        let t = if self.thickness > 0.0 { self.thickness } else { 1.6 };
        t * (b - a) as f64 / last.max(1) as f64
    }

    /// The pairs that `--pairs` routes coupled: pairs by name that a `diff_pair_gap` rule or a net
    /// class of their own marks as a differential pair. A pair of the default class without such a
    /// rule (`MOTOR_P` and `MOTOR_N`) is recognised but not coupled, unless `all` is set.
    pub fn coupled_pairs(&self, all: bool) -> Vec<(NetId, NetId)> {
        self.diff_pairs()
            .into_iter()
            .filter(|&(p, n)| {
                let (cp, cn) = (self.class_of(p), self.class_of(n));
                all || cp.diff_outer.is_some()
                    || cp.diff_inner.is_some()
                    || (cp.name != "Default" && cp.name == cn.name)
            })
            .collect()
    }

    /// The largest length difference allowed within the pair p, n: a `skew` rule with
    /// `within_diff_pairs` over the pair, or a plain `skew` rule whose nets are just these two.
    pub fn pair_skew_limit(&self, p: NetId, n: NetId) -> Option<f64> {
        self.match_rules
            .iter()
            .filter(|r| r.kind == MatchKind::Skew && r.nets.contains(&p) && r.nets.contains(&n))
            .filter(|r| r.within_pairs || r.nets.len() == 2)
            .filter_map(|r| r.value.max)
            .fold(None, |m: Option<f64>, v| Some(m.map_or(v, |m| m.min(v))))
    }

    /// The largest differential impedance of a pair over the routing layers, in ohm.
    pub fn pair_impedance(&self, p: NetId, n: NetId) -> Option<f64> {
        self.routing_layers()
            .iter()
            .filter_map(|l| {
                let (w, g, _) = self.pair_shape(p, n, l);
                self.zdiff(l, w, g)
            })
            .fold(None, |a: Option<f64>, z| Some(a.map_or(z, |x| x.max(z))))
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

    /// Width a net grows to where there is room, 0 if it does not grow: the target of its
    /// class, else `default` for the default class. Nets in `pairs` keep their width.
    pub fn target_width(&self, net: NetId, default: f64, pairs: &[NetId]) -> f64 {
        if pairs.contains(&net) {
            return 0.0;
        }
        let class = self.nets[net as usize].class;
        match self.classes.get(class as usize).and_then(|c| c.target_width) {
            Some(w) => w,
            None if class == 0 => default,
            None => 0.0,
        }
    }

    pub fn class_of(&self, net: NetId) -> &NetClass {
        &self.classes[self.nets[net as usize].class as usize]
    }

    /// How wide a track can be where it arrives at pin p: the narrow side of the pad, and at
    /// fine pitch the pitch to the next foreign pin of the part minus the clearance `clr`.
    pub fn pin_room(&self, p: usize, clr: f64) -> f64 {
        let pin = &self.pins[p];
        let pitch = self
            .pins
            .iter()
            .filter(|q| q.footprint == pin.footprint && q.net != pin.net)
            .map(|q| q.pos.dist(pin.pos))
            .fold(f64::INFINITY, f64::min);
        pin.size.0.min(pin.size.1).min(pitch - clr - 0.02)
    }

    /// True if the class of pin p has a minimum width rule with exempt parts and the copper at
    /// the pin is narrower than that minimum: such a pin ends a branch of its net, and the
    /// current between the other pins never passes it.
    pub fn narrow_pin(&self, p: usize) -> bool {
        let cls = self.class_of(self.pins[p].net);
        match cls.min_width {
            Some(min) if !cls.neck_refs.is_empty() => {
                self.pin_room(p, cls.clearance.max(self.rules.min_clearance)) < min - 1e-9
            }
            _ => false,
        }
    }

    /// Copper thickness of a layer in mm, from the stackup (35 um without one).
    pub fn copper_thickness(&self, layer: LayerId) -> f64 {
        self.stackup
            .iter()
            .find(|s| s.copper == Some(layer) && s.thickness > 0.0)
            .map_or(COPPER_MM, |s| s.thickness)
    }

    /// The copper a minimum width rule is made for: the thicker outer layer.
    fn reference_copper(&self) -> f64 {
        let last = self.layers.len().saturating_sub(1) as LayerId;
        self.copper_thickness(0).max(self.copper_thickness(last))
    }

    /// What a piece `w` mm wide on `layer` carries, as the width of a track on the outer
    /// copper. IPC-2221: I = k dT^0.44 A^0.725 with k = 0.048 outside and 0.024 inside, so
    /// the rise drops out of every comparison.
    pub fn current_width(&self, layer: LayerId, w: f64) -> f64 {
        let outer = layer == 0 || layer as usize + 1 == self.layers.len();
        let k: f64 = if outer { 1.0 } else { 0.5 };
        k.powf(1.0 / 0.725) * w * self.copper_thickness(layer) / self.reference_copper()
    }

    /// What vias of these drills carry together, as the width of a track on the outer copper:
    /// a barrel is an inner conductor with the plating as its thickness, and parallel barrels
    /// add their currents.
    pub fn via_current_width(&self, drills: impl IntoIterator<Item = f64>) -> f64 {
        let sum: f64 = drills.into_iter().map(|d| 0.5 * barrel_mm2(d).powf(0.725)).sum();
        sum.powf(1.0 / 0.725) / self.reference_copper()
    }

    /// How many vias of `drill` carry what `w` mm of outer copper carry (at least one).
    pub fn vias_for(&self, w: f64, drill: f64) -> usize {
        let n = 2.0 * (w * self.reference_copper() / barrel_mm2(drill)).powf(0.725);
        ((n - 1e-9).ceil() as usize).max(1)
    }

    /// What the copper of a current path of the net carries at least, as a width: the minimum
    /// of the width rule of its class. Not for a net of a differential pair, whose width is a
    /// matter of its impedance ([`Board::in_diff_pair`]).
    pub fn current_min(&self, net: NetId) -> Option<f64> {
        self.class_of(net).min_width.filter(|_| !self.in_diff_pair(net))
    }

    /// Whether the net is one of a differential pair ([`Board::diff_pairs`]).
    pub fn in_diff_pair(&self, net: NetId) -> bool {
        let n = &self.nets[net as usize];
        n.pins.len() >= 2
            && [('P', 'N'), ('N', 'P'), ('+', '-'), ('-', '+')].iter().any(|&(a, b)| {
                n.name
                    .strip_suffix(a)
                    .and_then(|stem| self.net_id(&format!("{stem}{b}")))
                    .is_some_and(|j| self.nets[j as usize].pins.len() >= 2)
            })
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

    /// Clearance of custom rules between any copper of two nets (0: none, the class decides).
    pub fn item_gap(&self, a: NetId, b: Option<NetId>) -> f64 {
        let Some(b) = b.filter(|_| !self.item_clearance.is_empty()) else {
            return 0.0;
        };
        self.item_clearance.get(&(a.min(b), a.max(b))).copied().unwrap_or(0.0)
    }

    /// Clearance between copper of a net and a hole.
    pub fn hole_gap(&self, net: NetId) -> f64 {
        self.net_hole_clearance
            .get(&net)
            .map_or(self.rules.hole_clearance, |&c| c.max(self.rules.hole_clearance))
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
    /// Cuts every plane by the higher priority planes of other nets on its layer, which KiCad
    /// fills first (see [`Plane::cut`]).
    pub fn cut_planes(&mut self) {
        let all = self.planes.clone();
        for p in self.planes.iter_mut() {
            p.cut = all
                .iter()
                .filter(|o| o.layer == p.layer && o.net != p.net && o.priority > p.priority)
                .map(|o| o.outline.clone())
                .collect();
        }
    }

    /// Whether p lies in the courtyard of `part` grown by `grow`: by its outline where the
    /// courtyard lines close into polygons, by its box otherwise.
    pub fn in_courtyard(&self, part: &str, p: Point, grow: f64) -> bool {
        match self.courtyard_shapes.get(part) {
            Some(polys) if !polys.is_empty() => polys.iter().any(|q| q.dist_to_point(p) < grow.max(1e-9)),
            _ => self.courtyards.get(part).is_some_and(|c| c.grow(grow).contains(p)),
        }
    }

    /// The parts whose courtyard holds p (grown by `grow`), see [`Board::in_courtyard`].
    pub fn courtyards_at(&self, p: Point, grow: f64) -> Vec<&str> {
        self.courtyards
            .iter()
            .filter(|(r, c)| c.grow(grow).contains(p) && self.in_courtyard(r, p, grow))
            .map(|(r, _)| r.as_str())
            .collect()
    }

    /// Courtyard outlines of `part`: the closed polygons, or its box as one polygon.
    pub fn courtyard_polys(&self, part: &str) -> Vec<Polygon> {
        match self.courtyard_shapes.get(part) {
            Some(polys) if !polys.is_empty() => polys.clone(),
            _ => self
                .courtyards
                .get(part)
                .map(|c| {
                    vec![Polygon::new(vec![
                        c.min,
                        Point { x: c.max.x, y: c.min.y },
                        c.max,
                        Point { x: c.min.x, y: c.max.y },
                    ])]
                })
                .unwrap_or_default(),
        }
    }

    /// Whether the courtyards of a and b overlap by more than `tol` (outlines, not boxes).
    pub fn courtyards_overlap(&self, a: &str, b: &str, tol: f64) -> bool {
        let (Some(ca), Some(cb)) = (self.courtyards.get(a), self.courtyards.get(b)) else {
            return false;
        };
        if !ca.grow(-tol).overlaps(&cb.grow(-tol)) {
            return false;
        }
        let pa: Vec<Polygon> = self.courtyard_polys(a).iter().map(|q| q.grown(-tol)).collect();
        let pb = self.courtyard_polys(b);
        pa.iter().any(|x| pb.iter().any(|y| x.dist_to_poly(y) <= 0.0))
    }

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
