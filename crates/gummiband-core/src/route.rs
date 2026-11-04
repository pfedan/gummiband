//! Topological router: A* over triangle edges with slot positions. Every route segment on a
//! layer is a chain of edge crossings; inside a free triangle it is a chord between two boundary
//! positions, and no two chords of different nets may interleave. So each layer stays planar
//! and the realization can pull every segment taut without crossings.
//!
//! Each routing layer has its own mesh. Vias sit on via sites inside free triangles; placing
//! one splits the triangle that holds the site in every layer mesh, and the routes that already
//! cross those triangles are threaded through the new ones.

use std::cmp::Ordering;
use std::collections::hash_map::Entry;
use std::collections::{HashMap, HashSet};

use crate::board::{Board, NetId, ObstacleKind};
use crate::geom::{Point, Rect, Shape, orient, point_seg_dist, pt, segs_intersect};
use crate::index::FastMap;
use crate::topo::{Mesh, NONE, OUTSIDE, PAD_SITE, STEINER, Substrate};

mod shut;

#[derive(Debug, Clone)]
pub struct RouterConfig {
    /// Cost of a via in mm of track.
    pub via_cost: f64,
    /// Weight of the pull between the nets of a differential pair: a step costs up to this
    /// factor of its length extra, the farther it is from the partner (0 = off).
    pub pair_pull: f64,
    /// Factor on the via cost in the pass that routes connections with vias again (0 = off).
    pub via_pass: f64,
    /// Weight of the preferred directions: routing layers alternate between horizontal and
    /// vertical, a step costs this much extra per mm across its layer's direction (0 = off).
    pub layer_dirs: f64,
    /// Route plane fan-outs before signal nets.
    pub fanout_first: bool,
    /// Expansion limit per connection search.
    pub max_expansions: usize,
    /// Seed for order perturbation (0 = plain priority order).
    pub seed: u64,
    /// Rip-up and reroute passes for failed connections.
    pub ripup_passes: usize,
    /// Rounds of the negotiation phase before the rip-up passes (0 = off), see
    /// [`Router::negotiate`].
    pub negotiate: usize,
    /// Via sites tried per triangle and expansion.
    pub sites_per_tri: usize,
    /// Extra clearance in mm kept in the capacity model (room for the geometry step).
    pub margin: f64,
    /// Allow vias inside SMD pads for plane fan-outs (otherwise only as a last resort).
    pub via_in_pad: bool,
    /// Extra cost (mm) for passing right next to a pad of another net that still waits for its
    /// route: keeps the escape of dense pin rows free.
    pub escape_cost: f64,
    /// A connection may end on a track of its net, next to where that crosses an edge. Whatever
    /// rips up a segment has to rip up what ends on it, see `improve.rs`.
    pub taps: bool,
    /// Hot spots (centre, extra clearance in mm) where an earlier geometry had violations: the
    /// capacity model asks for more room there, so the topology spreads out.
    pub hotspots: std::sync::Arc<Vec<(Point, f64)>>,
    /// Nets that may leave or enter a pad through a via in it at any time, not only as the last
    /// resort (empty by default; the experiment `GB_VIP_RETRY` fills it).
    pub vip_nets: std::sync::Arc<Vec<NetId>>,
    /// Local patterns of the board (see [`crate::patterns`]), for the hints below.
    pub patterns: Option<std::sync::Arc<crate::patterns::Patterns>>,
    /// Pins of a plane net close to each other reach the plane through one via: the leader fans
    /// out, the others connect to it ([`crate::patterns::PlaneShare`]).
    pub share_fanouts: bool,
    /// The connection between a decoupling capacitor and its IC pin goes before everything else.
    pub decaps_first: bool,
    /// Share of the moves of the open end that re-lay the connections of one pin row in pin
    /// order (0 = off).
    pub row_moves: f64,
    /// Differential pairs go before the other signal connections of their priority, the longest
    /// first: a pair has the fewest ways, the more so as one wide track (`--pairs`).
    pub pairs_first: bool,
    /// Connections between the same two parts (a bus, SPI, I2C, a connector) go as one block, in pin order
    /// across the block, see [`GroupOrder`].
    pub groups: GroupOrder,
}

/// Where the block of a part to part group goes in the routing order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GroupOrder {
    /// No groups: every connection by its own length.
    #[default]
    Off,
    /// Groups before the other signal connections of their priority, the shortest group first.
    Short,
    /// Groups before the other signal connections of their priority, the longest group first.
    Long,
    /// Each group at the place of its shortest connection among the others, as one block.
    Inline,
}

impl Default for RouterConfig {
    fn default() -> Self {
        let env = |k: &str, d: f64| std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d);
        RouterConfig {
            via_cost: 3.0,
            layer_dirs: 0.0,
            pair_pull: 0.0,
            via_pass: 4.0,
            fanout_first: true,
            max_expansions: env("GB_MAXEXP", 400_000.0) as usize,
            seed: 0,
            ripup_passes: env("GB_RIPUP", 3.0) as usize,
            negotiate: 0,
            sites_per_tri: 3,
            margin: 0.01,
            via_in_pad: false,
            escape_cost: env("GB_ESCAPE", 0.0),
            taps: false,
            hotspots: Default::default(),
            vip_nets: Default::default(),
            patterns: None,
            share_fanouts: false,
            decaps_first: false,
            row_moves: 0.0,
            pairs_first: false,
            groups: GroupOrder::Off,
        }
    }
}

impl RouterConfig {
    /// Switch on the hints of [`crate::patterns::hints`].
    pub fn with_hints(mut self, hints: &[&str]) -> RouterConfig {
        self.share_fanouts = hints.contains(&"shares");
        self.decaps_first = hints.contains(&"decaps");
        self.row_moves = if hints.contains(&"rows") { 0.15 } else { 0.0 };
        self.pairs_first = hints.contains(&"pairs");
        // `groups-short` and `groups-inline` pick the place of the blocks; plain `groups` is long,
        // unless `GB_GROUP_ORDER` (short, inline) says otherwise, as in earlier measurements
        self.groups = if hints.contains(&"groups-short") {
            GroupOrder::Short
        } else if hints.contains(&"groups-inline") {
            GroupOrder::Inline
        } else if hints.contains(&"groups-long") {
            GroupOrder::Long
        } else if hints.contains(&"groups") {
            match std::env::var("GB_GROUP_ORDER").as_deref() {
                Ok("short") => GroupOrder::Short,
                Ok("inline") => GroupOrder::Inline,
                _ => GroupOrder::Long,
            }
        } else {
            GroupOrder::Off
        };
        self
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Goal {
    /// Any of these pins.
    Pins(Vec<usize>),
    /// A via into a plane of the net (or a through hole pin of the net).
    Plane,
}

/// Place of a planned connection in the routing order below its priority, see
/// [`Router::order_keys`].
#[derive(Debug, Clone, Copy)]
struct OrderKey {
    /// 0 differential pairs (`pairs_first`), 1 part to part groups before the rest, 2 the rest.
    tier: u8,
    /// Position of the block: the jittered length of a single connection, or of the group.
    block: f64,
    /// Group id (usize::MAX for none): keeps the connections of a block together.
    group: usize,
    /// Place across the block.
    sweep: f64,
}

#[derive(Debug, Clone)]
pub struct Conn {
    pub net: NetId,
    pub from: usize,
    pub goal: Goal,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum End {
    Pin(usize),
    /// A via, by site id.
    Via(u32),
    /// A point on kept copper of the net.
    Point(Point),
    /// On a segment of the net, where it crosses this edge.
    Tap(u32, u32),
}

/// One crossing of a route segment: edge e, left from triangle `from`.
#[derive(Debug, Clone, Copy)]
pub struct Crossing {
    pub e: u32,
    pub from: u32,
}

#[derive(Debug, Clone)]
pub struct Seg {
    pub net: NetId,
    pub conn: u32,
    pub li: u8,
    pub width: f64,
    pub clearance: f64,
    pub start: End,
    pub end: End,
    pub crossings: Vec<Crossing>,
    /// Track width at each crossing (narrower inside the neck zones).
    pub widths: Vec<f64>,
    /// Free triangles with a chord of this segment.
    pub chord_tris: Vec<u32>,
    /// Zones around the connection's pins where the track may neck down, and the neck width.
    pub necks: Vec<Rect>,
    pub neck_w: f64,
    /// Set if this is copper of the board imported into the topology.
    pub imported: Option<Box<crate::import::Imported>>,
}

/// Position on a triangle boundary: local edge or local vertex.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum CPos {
    Edge(u8),
    Vertex(u8),
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct Chord {
    pub(crate) seg: u32,
    pub(crate) a: CPos,
    pub(crate) b: CPos,
}

#[derive(Debug, Clone, Copy)]
pub struct ViaInfo {
    pub net: NetId,
    pub r: f64,
    pub drill: f64,
    pub clearance: f64,
    pub users: u32,
}

/// Prices of the negotiation phase in mm of track, see [`Router::negotiate`].
#[derive(Clone)]
struct Prices {
    /// Capacity and planarity are soft: a track that does not fit, or crosses a track of
    /// another net, pays instead of being refused. Off at the end, only the history stays.
    soft: bool,
    /// Per mm of overflow of an edge.
    overflow: f64,
    /// Per track of another net crossed, and for a via where it does not fit.
    crossing: f64,
    /// [layer][edge] -> price of a crossing, grows wherever the conflicts stay.
    history: Vec<Vec<f64>>,
    /// Price of a crossing that leaves an edge no room at all, and the room per gap on an
    /// edge from which on a crossing is free: tracks take a tight gap only if they must, which
    /// leaves the geometry step its room.
    tight: f64,
    reserve: f64,
    /// An edge with no room for the track even without any other track stays closed: nothing
    /// that could move is in the way there.
    solid: bool,
}

/// Mutable routing state. Owns its copy of the meshes, because vias split triangles.
#[derive(Clone)]
pub struct Router<'a> {
    pub board: &'a Board,
    pub sub: Substrate,
    pub cfg: RouterConfig,
    /// [layer][edge] -> segment ids ordered from edges[e][0] to edges[e][1]
    pub cross: Vec<Vec<Vec<u32>>>,
    /// [layer][triangle] -> chords
    pub(crate) chords: Vec<Vec<Vec<Chord>>>,
    pub segs: Vec<Option<Seg>>,
    /// Placed vias by site id.
    pub vias: FastMap<u32, ViaInfo>,
    via_cells: FastMap<(i32, i32), Vec<u32>>,
    pub conns: Vec<Conn>,
    /// Segment ids per connection (empty = unrouted).
    pub conn_segs: Vec<Vec<u32>>,
    pub conn_vias: Vec<Vec<u32>>,
    /// Pins whose pads overlap (same net) share a group: they are connected already.
    pub pin_group: Vec<u32>,
    /// Pin groups whose kept copper has a via into a plane of the net.
    plane_groups: HashSet<u32>,
    /// Partner net of each differential pair net, and where the nets ran so far.
    partner: HashMap<NetId, NetId>,
    guides: HashMap<NetId, Vec<Guide>>,
    /// The board has kept tracks or vias.
    pub has_kept: bool,
    /// Soft copper of the board is in the topology, see [`crate::import`].
    pub soft_on: bool,
    /// Chains of the soft copper that could not be traced: they stay as they are.
    pub soft_lost: Vec<u32>,
    /// Per pin: number of unrouted connections that start or end there.
    pending: Vec<u32>,
    /// Prices while a negotiation runs and in the passes after it.
    prices: Option<Prices>,
    /// Connections that went with a track they ended on, see [`Router::rip_up`].
    pub(crate) torn: Vec<usize>,
    /// Buffers of the search, kept to avoid growing them again for every connection.
    search_buf: SearchBuf,
    /// Debugging: positions expanded by the last search of a traced connection.
    pub trace: std::sync::Arc<std::sync::Mutex<Option<Vec<(Point, u8)>>>>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum NK {
    /// In triangle t on layer li, entered through local edge `entry` (3 = none) at `slot`.
    Tri {
        t: u32,
        li: u8,
        entry: u8,
        slot: u16,
    },
    /// At an existing via vertex (by site) on layer li.
    Vert {
        site: u32,
        li: u8,
    },
    /// At a new via on `site`, continuing on layer li.
    Site {
        site: u32,
        li: u8,
    },
    GoalPin {
        pin: usize,
    },
    GoalVia {
        site: u32,
    },
    GoalSite {
        site: u32,
    },
    /// On segment `seg`, next to its crossing of edge `e`.
    GoalTap {
        seg: u32,
        e: u32,
    },
}

#[derive(Debug, Clone, Copy)]
struct Node {
    kind: NK,
    pos: Point,
    g: f64,
    parent: u32,
    /// Bloom filter of the triangles of this node's block: the triangles walked back from a
    /// node are cut into blocks, so a loop test looks at a few filters, not at a whole path.
    seen: u64,
    /// Last node of the block before (none at a via or the start) and the size of this one.
    block_up: u32,
    block_len: u8,
    /// Nearest ancestor that is a via, for the hole spacing within one connection.
    via_up: u32,
    /// A later push reached the same state for less: the node is skipped when it comes up.
    stale: bool,
}

/// Two bits of the filter for a triangle.
fn tri_bit(t: u32, li: u8) -> u64 {
    let h = t.wrapping_add((li as u32) << 28).wrapping_mul(0x9e37_79b1);
    1 << (h >> 26) | 1 << ((h >> 20) & 63)
}

/// Short blocks keep the filters sparse: with two bits per triangle, a full block of 24 set
/// almost half of the 64 and sent every fourth test through a whole block for nothing.
const BLOCK: u8 = 12;

/// A search that has expanded this many nodes asks whether its goal is shut in, looking at no
/// more than this many places from the goal side (see `shut`). A goal shut in on the boards
/// measured took 78 places in the middle and rarely more than 400; a successful search takes
/// about 1000 expansions in the middle, a failing one 24000 to 35000.
const GOAL_CHECK_AT: usize = 1000;
const GOAL_CHECK_BUDGET: usize = 1000;

thread_local! {
    /// After how many expansions a search of this thread asks (0: never), and how many goals
    /// it found shut in, for tests.
    static GOAL_CHECK_TEST: std::cell::Cell<Option<usize>> = const { std::cell::Cell::new(None) };
    static GOAL_SHUT_TEST: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// After how many expansions a search asks whether its goal is shut in; 0 never.
/// `GB_GOAL_CHECK=0` switches the check off, for comparisons: the results are the same.
fn goal_check_at() -> usize {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    GOAL_CHECK_TEST.with(|c| c.get()).unwrap_or_else(|| {
        if *ON.get_or_init(|| std::env::var("GB_GOAL_CHECK").map_or(true, |v| v != "0")) {
            GOAL_CHECK_AT
        } else {
            0
        }
    })
}

/// An edge counts as over capacity from this overflow in mm on.
const OVER_TOL: f64 = 1e-6;

/// A negotiation ends after this many rounds without fewer conflicts.
const MAX_STALE: usize = 5;

/// Per search state the least cost so far and the node that holds it.
type Best = FastMap<u64, (f64, u32)>;

#[derive(Default)]
struct SearchBuf {
    nodes: Vec<Node>,
    best: Best,
    open: OpenHeap,
}

/// A copy of a router starts with empty buffers: every search clears them first, and copying
/// the large ones of the last search would cost a mutant more than growing its own.
impl Clone for SearchBuf {
    fn clone(&self) -> SearchBuf {
        SearchBuf::default()
    }
}

/// The open list of a search: nodes by `f`, ties by node index, both ascending. One `u128` per
/// entry, `f` in its total order (as `f64::total_cmp`) above the index, in a four-ary heap: the
/// order is total, so the nodes come out in the same order as from any other exact queue, and a
/// pop compares integers and touches fewer cache lines than a binary heap of `(f64, u32)`.
#[derive(Default)]
struct OpenHeap(Vec<u128>);

impl OpenHeap {
    #[inline]
    fn entry(f: f64, idx: u32) -> u128 {
        let b = f.to_bits();
        let k = if b >> 63 == 1 { !b } else { b | 1 << 63 };
        (k as u128) << 32 | idx as u128
    }

    fn clear(&mut self) {
        self.0.clear();
    }

    #[inline]
    fn push(&mut self, f: f64, idx: u32) {
        let x = Self::entry(f, idx);
        let h = &mut self.0;
        let mut i = h.len();
        h.push(x);
        while i > 0 {
            let up = (i - 1) / 4;
            if h[up] <= x {
                break;
            }
            h[i] = h[up];
            i = up;
        }
        h[i] = x;
    }

    /// The node index with the least `f`.
    #[inline]
    fn pop(&mut self) -> Option<u32> {
        let h = &mut self.0;
        let top = *h.first()?;
        let last = h.pop().unwrap();
        let n = h.len();
        if n > 0 {
            let mut i = 0;
            loop {
                let c = 4 * i + 1;
                if c >= n {
                    break;
                }
                let mut m = c;
                for j in c + 1..(c + 4).min(n) {
                    if h[j] < h[m] {
                        m = j;
                    }
                }
                if h[m] >= last {
                    break;
                }
                h[i] = h[m];
                i = m;
            }
            h[i] = last;
        }
        Some(top as u32)
    }
}

/// Space a crossing needs on an edge.
#[derive(Debug, Clone, Copy)]
pub struct Item {
    pub net: NetId,
    pub w: f64,
    pub c: f64,
    /// Side of the apex of each adjacent triangle (edge_tris order) the track passes:
    /// 0 = towards edges[e][0], 1 = towards edges[e][1], 2 = unknown or not relevant.
    pub sides: [u8; 2],
}

/// A crossing already on an edge with all that packing needs from it.
#[derive(Debug, Clone, Copy)]
struct Placed {
    need_u: f64,
    need_v: f64,
    bounds: (f64, f64),
    /// Gap to the next crossing on the edge.
    gap_next: f64,
    /// Gap to the searched track on its right and on its left.
    gap_to_new: f64,
    gap_from_new: f64,
}

/// An edge as the search of one connection sees it: a search does not change the routes, and
/// its track has one net and one width per edge, so all this is computed once per edge and
/// search; only the sides of the apexes differ from crossing to crossing.
struct EdgeView {
    placed: Vec<Placed>,
    len: f64,
    /// Extra gap of hot spots.
    hot: f64,
    /// End needs of the searched track.
    need_u: f64,
    need_v: f64,
    /// Per adjacent triangle the bounds its apex sets on the searched track when it passes on
    /// side 0 (upper bound) or side 1 (lower bound).
    apex: [(f64, f64); 2],
}

/// An extra obstacle at a triangle apex for capacity checks (a via about to be placed).
#[derive(Debug, Clone, Copy)]
pub struct ExtraApex {
    /// Index into edge_tris[e] of the triangle whose apex it is.
    pub side: usize,
    pub p: Point,
    pub via: ViaInfo,
}

/// How much farther the way of the pair partner counts on another layer, in mm: right above it
/// the pull is still there, but less than on its own layer.
const OFF_LAYER: f64 = 1.0;

/// Where one connection of a differential pair net runs: the polyline of its committed chain,
/// with the routing layer of each piece. The partner's search is pulled towards it.
#[derive(Debug, Clone)]
struct Guide {
    conn: usize,
    /// Pieces from the point before to this point on routing layer `li` (the first has none).
    pts: Vec<(Point, u8)>,
}

impl Guide {
    fn of(conn: usize, chain: &[Node]) -> Guide {
        let first = chain.iter().find_map(|n| node_layer(&n.kind)).unwrap_or(0);
        let mut li = first;
        let pts = chain
            .iter()
            .map(|n| {
                if let Some(l) = node_layer(&n.kind) {
                    li = l;
                }
                (n.pos, li)
            })
            .collect();
        Guide { conn, pts }
    }
}

fn node_layer(k: &NK) -> Option<u8> {
    match *k {
        NK::Tri { li, .. } | NK::Vert { li, .. } | NK::Site { li, .. } => Some(li),
        _ => None,
    }
}

/// Distance from q on routing layer li to the nearest piece of the guides, a piece on another
/// layer [`OFF_LAYER`] farther; at most `cap`.
fn guide_dist(guides: &[Guide], li: usize, q: Point, cap: f64) -> f64 {
    let mut best = cap;
    for g in guides {
        if let [(p, _)] = g.pts[..] {
            best = best.min(p.dist(q) + OFF_LAYER);
        }
        for w in g.pts.windows(2) {
            let (a, b, l) = (w[0].0, w[1].0, w[1].1);
            let extra = if l as usize == li { 0.0 } else { OFF_LAYER };
            if extra >= best {
                continue;
            }
            best = best.min(point_seg_dist(q, a, b) + extra);
        }
    }
    best
}

/// The search state of a node in one word: the kind in the top three bits, the triangle, site,
/// pin or segment below, the rest in the low half. One round of the hasher instead of three.
#[inline]
fn key(k: &NK) -> u64 {
    let (kind, a, b) = match *k {
        NK::Tri { t, li, entry, slot } => (0u64, t, ((li as u32) << 24) | ((entry as u32) << 16) | slot as u32),
        NK::Vert { site, li } => (1, site, li as u32),
        NK::GoalPin { pin } => (2, pin as u32, 0),
        NK::GoalVia { site } => (3, site, 0),
        NK::Site { site, li } => (4, site, li as u32),
        NK::GoalSite { site } => (5, site, 0),
        NK::GoalTap { seg, e } => (6, seg, e),
    };
    assert!(a < 1 << 29, "search state out of range");
    kind << 61 | (a as u64) << 32 | b as u64
}

impl<'a> Router<'a> {
    pub fn new(board: &'a Board, sub: &Substrate, cfg: RouterConfig) -> Router<'a> {
        let partner = if cfg.pair_pull > 0.0 {
            board
                .diff_pairs()
                .into_iter()
                .flat_map(|(a, b)| [(a, b), (b, a)])
                .collect()
        } else {
            HashMap::new()
        };
        let (pin_group, plane_groups) = Self::pin_groups(board, sub);
        let mut r = Router {
            board,
            sub: sub.clone(),
            cfg,
            cross: sub.m.iter().map(|m| vec![Vec::new(); m.edges.len()]).collect(),
            chords: sub.m.iter().map(|m| vec![Vec::new(); m.tris.len()]).collect(),
            segs: Vec::new(),
            vias: FastMap::default(),
            via_cells: FastMap::default(),
            conns: Vec::new(),
            conn_segs: Vec::new(),
            conn_vias: Vec::new(),
            search_buf: Default::default(),
            trace: Default::default(),
            pin_group,
            plane_groups,
            partner,
            guides: HashMap::new(),
            has_kept: board
                .obstacles
                .iter()
                .any(|o| matches!(o.kind, ObstacleKind::Track | ObstacleKind::Via)),
            pending: vec![0; board.pins.len()],
            prices: None,
            torn: Vec::new(),
            soft_on: false,
            soft_lost: Vec::new(),
        };
        if !board.soft.chains.is_empty() {
            r.import_soft();
        }
        r
    }

    fn pin_groups(board: &Board, sub: &Substrate) -> (Vec<u32>, HashSet<u32>) {
        let n = board.pins.len();
        let mut parent: Vec<u32> = (0..n as u32).collect();
        fn find(p: &mut [u32], x: u32) -> u32 {
            let mut r = x;
            while p[r as usize] != r {
                r = p[r as usize];
            }
            let mut y = x;
            while p[y as usize] != r {
                let next = p[y as usize];
                p[y as usize] = r;
                y = next;
            }
            r
        }
        for (pi, pin) in board.pins.iter().enumerate() {
            for &o in &pin.obstacles {
                let poly = &sub.shared.obs_polys[o];
                for q in sub.shared.obs_index.query(&poly.bbox()) {
                    let other = &board.obstacles[q as usize];
                    let Some(pj) = other.pin else { continue };
                    if pj == pi || board.pins[pj].net != pin.net || !other.layers.intersects(board.obstacles[o].layers)
                    {
                        continue;
                    }
                    if poly.dist_to_poly(&sub.shared.obs_polys[q as usize]) <= 0.0 {
                        let (a, b) = (find(&mut parent, pi as u32), find(&mut parent, pj as u32));
                        if a != b {
                            parent[a as usize] = b;
                        }
                    }
                }
            }
        }
        // kept copper connects pins: everything on one piece of copper is one pin already (a
        // pour laid out before routing joins its pads too; other zones may not reach a pad once
        // KiCad fills them, so they join nothing)
        let kept =
            |o: &crate::board::Obstacle| matches!(o.kind, ObstacleKind::Track | ObstacleKind::Via | ObstacleKind::Pour);
        let into_plane = |o: &crate::board::Obstacle| {
            let (ObstacleKind::Via, Shape::Circle { c, .. }) = (o.kind, &o.shape) else {
                return false;
            };
            let on = |pl: &crate::board::Plane| Some(pl.net) == o.net && o.layers.has(pl.layer) && pl.covers(*c);
            board.planes.iter().any(on)
        };
        let mut on_plane: Vec<u32> = Vec::new();
        let mut seen = vec![false; board.obstacles.len()];
        for start in 0..board.obstacles.len() {
            if seen[start] || !kept(&board.obstacles[start]) {
                continue;
            }
            seen[start] = true;
            let net = board.obstacles[start].net;
            let mut stack = vec![start];
            let mut first_pin: Option<u32> = None;
            let mut plane = false;
            while let Some(o) = stack.pop() {
                plane |= into_plane(&board.obstacles[o]);
                let poly = &sub.shared.obs_polys[o];
                for q in sub.shared.obs_index.query(&poly.bbox()) {
                    let q = q as usize;
                    let other = &board.obstacles[q];
                    if other.net != net
                        || !other.layers.intersects(board.obstacles[o].layers)
                        || poly.dist_to_poly(&sub.shared.obs_polys[q]) > 0.0
                    {
                        continue;
                    }
                    if let Some(pj) = other.pin {
                        match first_pin {
                            None => first_pin = Some(pj as u32),
                            Some(f) => {
                                let (a, b) = (find(&mut parent, f), find(&mut parent, pj as u32));
                                if a != b {
                                    parent[a as usize] = b;
                                }
                            }
                        }
                    }
                    if kept(other) && !seen[q] {
                        seen[q] = true;
                        stack.push(q);
                    }
                }
            }
            if let (true, Some(f)) = (plane, first_pin) {
                on_plane.push(f);
            }
        }
        let groups: Vec<u32> = (0..n as u32).map(|x| find(&mut parent, x)).collect();
        let on_plane = on_plane.iter().map(|&p| groups[p as usize]).collect();
        (groups, on_plane)
    }

    fn m(&self, li: usize) -> &Mesh {
        &self.sub.m[li]
    }

    fn nl(&self) -> usize {
        self.sub.m.len()
    }

    pub fn site_point(&self, site: u32) -> Point {
        self.sub.shared.sites[site as usize].p
    }

    // ------------------------------------------------------------------ clearances

    /// Clearance a route of clearance `c` keeps from obstacle `obs`.
    pub fn eff_clearance(&self, obs: u32, c: f64, net: NetId) -> f64 {
        if obs == OUTSIDE {
            return self.board.rules.edge_clearance;
        }
        let o = &self.board.obstacles[obs as usize];
        match o.kind {
            ObstacleKind::Keepout { .. } => crate::board::KEEPOUT_GAP,
            ObstacleKind::Hole => self.board.hole_gap(net).max(o.clearance),
            _ => o.clearance.max(c).max(self.board.item_gap(net, o.net)),
        }
    }

    fn obs_net(&self, obs: u32) -> Option<NetId> {
        if obs == OUTSIDE {
            None
        } else {
            self.board.obstacles[obs as usize].net
        }
    }

    /// Via at mesh vertex v of layer li, if any. A via vertex without via (a via that was
    /// ripped up, or a split that was not used) is a phantom: tracks keep their clearance from
    /// the point, so that they never bunch up at a free vertex.
    fn via_at(&self, li: usize, v: u32) -> Option<ViaInfo> {
        let site = self.m(li).vert_site[v as usize];
        if site == NONE {
            return None;
        }
        Some(self.vias.get(&site).copied().unwrap_or(ViaInfo {
            net: NetId::MAX,
            r: 0.0,
            drill: 0.0,
            clearance: 0.0,
            users: 0,
        }))
    }

    fn via_need(board: &Board, via: &ViaInfo, it: &Item) -> f64 {
        if via.net == NetId::MAX {
            // phantom point: tracks on both sides together keep one clearance
            return it.c / 2.0 + it.w / 2.0;
        }
        if via.net == it.net {
            0.0
        } else {
            via.r + via.clearance.max(it.c).max(board.item_gap(it.net, Some(via.net))) + it.w / 2.0
        }
    }

    /// Distance from the edge end vertex to the centre line of a track of `it`.
    pub(crate) fn end_need(&self, li: usize, e: u32, end: usize, it: &Item) -> f64 {
        let m = self.m(li);
        let mut need: f64 = 0.0;
        for d in m.demands(e, end) {
            if self.obs_net(d.obs) == Some(it.net) {
                continue;
            }
            need = need.max((self.eff_clearance(d.obs, it.c, it.net) + it.w / 2.0) * d.factor as f64);
        }
        if let Some(via) = self.via_at(li, m.edges[e as usize][end]) {
            need = need.max(Self::via_need(self.board, &via, it));
        }
        need
    }

    /// Need of a track at vertex v (vertex demands only), for a new edge.
    fn vertex_need(&self, li: usize, v: u32, it: &Item) -> f64 {
        let m = self.m(li);
        let mut need: f64 = 0.0;
        for &t in &m.vert_tris[v as usize] {
            let o = m.status[t as usize];
            if o == NONE || self.obs_net(o) == Some(it.net) {
                continue;
            }
            need = need.max(self.eff_clearance(o, it.c, it.net) + it.w / 2.0);
        }
        if let Some(via) = self.via_at(li, v) {
            need = need.max(Self::via_need(self.board, &via, it));
        }
        need
    }

    fn item_of(&self, seg: u32) -> Item {
        let s = self.segs[seg as usize].as_ref().unwrap();
        Item {
            net: s.net,
            w: s.width,
            c: s.clearance,
            sides: [2, 2],
        }
    }

    /// Neck zones and width of connection ci: around its pins a wide track may narrow down.
    /// With a minimum width rule for the class, only inside exempt courtyards.
    pub fn necks(&self, ci: usize) -> (Vec<Rect>, f64) {
        let c = &self.conns[ci];
        let cls = self.board.class_of(c.net);
        let mut pins = vec![c.from];
        if let Goal::Pins(p) = &c.goal {
            pins.extend(p.iter().copied());
        }
        // near pins the track may narrow to what fits there: the pad width, and at fine pitch
        // the pitch to the next foreign pin of the part minus the clearance. As wide as
        // possible, this copper carries the net's current, but at most 0.4 mm (wider necks ask
        // more of the topology than it can keep near the zone border; measured over the
        // boards). Never below the board minimum.
        let clr = cls.clearance.max(self.board.rules.min_clearance);
        let pad_w = pins
            .iter()
            .map(|&p| self.board.pin_room(p, clr).min(0.4))
            .fold(f64::INFINITY, f64::min);
        let neck_w = self.board.rules.min_track_width.max(pad_w).min(cls.track_width);
        if neck_w >= cls.track_width - 1e-9 {
            return (Vec::new(), cls.track_width);
        }
        let mut zones = Vec::new();
        for &p in &pins {
            let pin = &self.board.pins[p];
            if cls.min_width.is_some() {
                // custom rules allow the neck where the track touches the part's courtyard
                if !cls.neck_refs.contains(&pin.footprint) {
                    continue;
                }
                if let Some(r) = self.board.courtyards.get(&pin.footprint)
                    && r.contains(pin.pos)
                {
                    zones.push(*r);
                }
            } else {
                // the whole fan-out of the part narrows alike (a zone per pin would leave
                // a narrow track next to a wide one at the pin pitch)
                let mut r = Rect::empty();
                r.add(pin.pos);
                let r = r.grow(pin.size.0.max(pin.size.1) / 2.0 + 0.8);
                zones.push(match self.board.courtyards.get(&pin.footprint) {
                    Some(c) if c.contains(pin.pos) => {
                        let c = c.grow(0.3);
                        Rect {
                            min: pt(c.min.x.min(r.min.x), c.min.y.min(r.min.y)),
                            max: pt(c.max.x.max(r.max.x), c.max.y.max(r.max.y)),
                        }
                    }
                    _ => r,
                });
            }
        }
        (zones, neck_w)
    }

    /// True if triangle t lies in kept tracks or vias (the search only gets there on its own
    /// net).
    fn on_kept(&self, li: usize, t: u32) -> bool {
        let st = self.m(li).status[t as usize];
        st != NONE
            && st != OUTSIDE
            && matches!(
                self.board.obstacles[st as usize].kind,
                ObstacleKind::Track | ObstacleKind::Via
            )
    }

    /// Extra cost of a step of length `step` ending at q, away from the pair partner's way.
    /// Nothing within the pitch of the pair (track width and gap) on the same layer; the way of
    /// the partner on another layer counts as [`OFF_LAYER`] farther.
    fn pull(&self, guide: Option<(&[Guide], f64)>, li: usize, q: Point, step: f64) -> f64 {
        let Some((g, pitch)) = guide else { return 0.0 };
        let d = guide_dist(g, li, q, 2.0 + pitch);
        self.cfg.pair_pull * step * ((d - pitch).max(0.0) / 2.0).min(1.0)
    }

    /// Extra cost of the step a-b on routing layer li for leaving the layer's preferred
    /// direction (even layers horizontal, odd ones vertical).
    fn across(&self, li: usize, a: Point, b: Point) -> f64 {
        if self.cfg.layer_dirs <= 0.0 {
            return 0.0;
        }
        let d = b - a;
        self.cfg.layer_dirs * if li % 2 == 0 { d.y.abs() } else { d.x.abs() }
    }

    /// Escape penalty for crossing edge e: an end vertex on a pad of another net that still
    /// has unrouted connections.
    fn escape_penalty(&self, li: usize, e: u32, net: NetId) -> f64 {
        if self.cfg.escape_cost <= 0.0 {
            return 0.0;
        }
        let m = self.m(li);
        for end in 0..2 {
            for d in m.demands(e, end) {
                if d.obs == OUTSIDE {
                    continue;
                }
                let o = &self.board.obstacles[d.obs as usize];
                if let Some(p) = o.pin
                    && o.net != Some(net)
                    && self.pending[p] > 0
                {
                    return self.cfg.escape_cost;
                }
            }
        }
        0.0
    }

    /// Extra room asked for near hot spots, for an edge a-b.
    fn hot_extra(&self, a: Point, b: Point) -> f64 {
        let mut x: f64 = 0.0;
        for &(c, extra) in self.cfg.hotspots.iter() {
            if point_seg_dist(c, a, b) < 1.0 {
                x = x.max(extra);
            }
        }
        x
    }

    /// Width of a track of `full` width with neck zones at edge e.
    fn width_at_edge(&self, li: usize, e: u32, full: f64, necks: &[Rect], neck_w: f64) -> f64 {
        if necks.is_empty() {
            return full;
        }
        let m = self.m(li);
        let [u, v] = m.edges[e as usize];
        let (a, b) = (m.verts[u as usize], m.verts[v as usize]);
        // narrow where the edge touches a zone: the realized track narrows on every piece that
        // touches a zone (that is what intersectsCourtyard() rules allow), and an exit from a
        // fine pitch pad crosses its first edges right next to the pad
        let hit = necks.iter().any(|r| r.touches_seg(a, b));
        if hit { neck_w } else { full }
    }

    /// Item of segment `seg` crossing edge e, with its sides relative to the two apexes.
    fn item_on(&self, li: usize, e: u32, seg: u32) -> Item {
        let mut it = self.item_of(seg);
        {
            let s = self.segs[seg as usize].as_ref().unwrap();
            if let Some(k) = s.crossings.iter().position(|c| c.e == e) {
                it.w = s.widths.get(k).copied().unwrap_or(s.width);
            }
        }
        for (k, &t) in self.m(li).edge_tris[e as usize].iter().enumerate() {
            if t != NONE {
                it.sides[k] = self.chord_side(li, e, t, seg);
            }
        }
        it
    }

    pub fn items_on(&self, li: usize, e: u32) -> Vec<Item> {
        self.cross[li][e as usize]
            .iter()
            .map(|&s| self.item_on(li, e, s))
            .collect()
    }

    /// Side of the apex of t the chord of `seg` in t passes, seen on edge e (see [`Item`]).
    fn chord_side(&self, li: usize, e: u32, t: u32, seg: u32) -> u8 {
        let Some(ch) = self.chords[li][t as usize].iter().find(|c| c.seg == seg) else {
            return 2;
        };
        let ke = self.m(li).local_edge(t, e) as u8;
        let other = if ch.a == CPos::Edge(ke) { ch.b } else { ch.a };
        self.side_of_pos(li, t, e, other)
    }

    /// Side (0/1 = towards edges[e][0]/[1]) for a chord from edge e to `other` in triangle t.
    fn side_of_pos(&self, li: usize, t: u32, e: u32, other: CPos) -> u8 {
        let m = self.m(li);
        let [u, v] = m.edges[e as usize];
        let tv = m.tris[t as usize];
        let x = match other {
            CPos::Edge(k) => {
                let (p, q) = (tv[k as usize], tv[(k as usize + 1) % 3]);
                if p == u || q == u {
                    u
                } else if p == v || q == v {
                    v
                } else {
                    return 2;
                }
            }
            CPos::Vertex(i) => tv[i as usize],
        };
        if x == u {
            0
        } else if x == v {
            1
        } else {
            2
        }
    }

    /// What the apex of triangle t (next to edge e) blocks on e: its position relative to the
    /// edge and the obstacles and via there, or None if it blocks nothing.
    fn apex_of(
        &self,
        li: usize,
        e: u32,
        k: usize,
        t: u32,
        (pu, dir): (Point, Point),
        extra: Option<ExtraApex>,
    ) -> Option<(f64, f64, &[u32], Option<ViaInfo>)> {
        let m = self.m(li);
        let [u, v] = m.edges[e as usize];
        let (pw, obs, via_here) = match extra {
            Some(x) if x.side == k => (x.p, &[][..], Some(x.via)),
            _ => {
                let w = *m.tris[t as usize].iter().find(|&&x| x != u && x != v).unwrap();
                let obs = m.vert_obs.get(w as usize).map_or(&[][..], |o| &o[..]);
                (m.verts[w as usize], obs, self.via_at(li, w))
            }
        };
        if obs.is_empty() && via_here.is_none() {
            return None;
        }
        Some(((pw - pu).cross(dir).abs(), (pw - pu).dot(dir), obs, via_here))
    }

    /// Half the stretch of the edge an apex at height h blocks for item `it`, if any.
    fn apex_half(&self, h: f64, obs: &[u32], via_here: Option<ViaInfo>, it: &Item) -> Option<f64> {
        let mut need: f64 = 0.0;
        for &o in obs {
            if self.obs_net(o) != Some(it.net) {
                need = need.max(self.eff_clearance(o, it.c, it.net) + it.w / 2.0);
            }
        }
        if let Some(via) = via_here {
            need = need.max(Self::via_need(self.board, &via, it));
        }
        if need <= 0.0 {
            return None;
        }
        let need = need + self.cfg.margin;
        if need <= h {
            return None;
        }
        Some((need * need - h * h).sqrt())
    }

    /// Lower and upper bounds of each item's centre line from the apex obstacles of the two
    /// triangles next to e (an obstacle or via at the apex blocks the middle of the edge).
    fn apex_bounds(&self, li: usize, e: u32, items: &[Item], extra: Option<ExtraApex>) -> Vec<(f64, f64)> {
        let m = self.m(li);
        let mut b = vec![(f64::NEG_INFINITY, f64::INFINITY); items.len()];
        let [u, v] = m.edges[e as usize];
        let pu = m.verts[u as usize];
        let pv = m.verts[v as usize];
        let len = pu.dist(pv);
        if len <= 0.0 {
            return b;
        }
        let dir = (pv - pu) * (1.0 / len);
        for (k, &t) in m.edge_tris[e as usize].iter().enumerate() {
            if t == NONE || m.status[t as usize] != NONE {
                continue;
            }
            let Some((h, proj, obs, via_here)) = self.apex_of(li, e, k, t, (pu, dir), extra) else {
                continue;
            };
            for (i, it) in items.iter().enumerate() {
                let side = it.sides[k];
                if side > 1 {
                    continue;
                }
                let Some(half) = self.apex_half(h, obs, via_here, it) else {
                    continue;
                };
                if side == 0 {
                    b[i].1 = b[i].1.min(proj - half);
                } else {
                    b[i].0 = b[i].0.max(proj + half);
                }
            }
        }
        b
    }

    pub(crate) fn gap(board: &Board, a: &Item, b: &Item) -> f64 {
        if a.net == b.net {
            return 0.0;
        }
        let mut c = a.c.max(b.c);
        if a.net != NetId::MAX && b.net != NetId::MAX && !board.pair_clearance.is_empty() {
            c = c.max(board.track_clearance(a.net, b.net));
        }
        a.w / 2.0 + c + b.w / 2.0
    }

    /// Pack items along a segment of length `len` with end needs; centre line intervals.
    #[allow(clippy::too_many_arguments)]
    fn pack_with(
        board: &Board,
        items: &[Item],
        len: f64,
        need_u: impl Fn(&Item) -> f64,
        need_v: impl Fn(&Item) -> f64,
        bounds: &[(f64, f64)],
        extra_gap: f64,
    ) -> Option<Vec<(f64, f64)>> {
        let n = items.len();
        let mut lo = vec![0.0; n];
        let mut hi = vec![0.0; n];
        for i in 0..n {
            let mut x = need_u(&items[i]).max(bounds.get(i).map_or(f64::NEG_INFINITY, |b| b.0));
            if i > 0 {
                x = x.max(lo[i - 1] + Self::gap(board, &items[i - 1], &items[i]) + extra_gap);
            }
            lo[i] = x;
        }
        for i in (0..n).rev() {
            let mut y = (len - need_v(&items[i])).min(bounds.get(i).map_or(f64::INFINITY, |b| b.1));
            if i + 1 < n {
                y = y.min(hi[i + 1] - Self::gap(board, &items[i], &items[i + 1]) - extra_gap);
            }
            hi[i] = y;
        }
        if (0..n).any(|i| lo[i] > hi[i] + 1e-9) {
            return None;
        }
        Some(
            lo.into_iter()
                .zip(hi)
                .map(|(a, b)| if a > b { ((a + b) / 2.0, (a + b) / 2.0) } else { (a, b) })
                .collect(),
        )
    }

    /// Centre line intervals of all items on edge e (u to v), or None if they do not fit.
    /// `extra_via` is a via about to be placed at site (vertex of this mesh).
    pub fn pack(
        &self,
        li: usize,
        e: u32,
        items: &[Item],
        extra_via: Option<(u32, &ViaInfo)>,
    ) -> Option<Vec<(f64, f64)>> {
        self.pack_ext(li, e, items, extra_via, None)
    }

    pub fn pack_ext(
        &self,
        li: usize,
        e: u32,
        items: &[Item],
        extra_via: Option<(u32, &ViaInfo)>,
        extra_apex: Option<ExtraApex>,
    ) -> Option<Vec<(f64, f64)>> {
        let m = self.m(li);
        let len = m.edge_len(e);
        let [u, v] = m.edges[e as usize];
        let margin = self.cfg.margin;
        let mut extra_apex = extra_apex;
        let mut via_vertex_here = NONE;
        if let Some((site, via)) = extra_via
            && let Some(&vv) = m.site_vert.get(&site)
        {
            via_vertex_here = vv;
            for (k, &t) in m.edge_tris[e as usize].iter().enumerate() {
                if t != NONE && m.tris[t as usize].contains(&vv) && vv != u && vv != v {
                    extra_apex = Some(ExtraApex {
                        side: k,
                        p: m.verts[vv as usize],
                        via: *via,
                    });
                }
            }
        }
        let extra_end = |vert: u32, it: &Item| -> f64 {
            match extra_via {
                Some((_, via)) if vert == via_vertex_here => Self::via_need(self.board, via, it),
                _ => 0.0,
            }
        };
        let bounds = self.apex_bounds(li, e, items, extra_apex);
        let hot = self.hot_extra(m.verts[u as usize], m.verts[v as usize]);
        Self::pack_with(
            self.board,
            items,
            len,
            |it| self.end_need(li, e, 0, it).max(extra_end(u, it)) + margin + hot,
            |it| self.end_need(li, e, 1, it).max(extra_end(v, it)) + margin + hot,
            &bounds,
            hot,
        )
    }

    fn edge_view(&self, li: usize, e: u32, new: &Item) -> EdgeView {
        let m = self.m(li);
        let [u, v] = m.edges[e as usize];
        let (pu, pv) = (m.verts[u as usize], m.verts[v as usize]);
        let items = self.items_on(li, e);
        let hot = self.hot_extra(pu, pv);
        let margin = self.cfg.margin;
        let mut bounds = vec![(f64::NEG_INFINITY, f64::INFINITY); items.len()];
        let mut apex = [(f64::INFINITY, f64::NEG_INFINITY); 2];
        let len = pu.dist(pv);
        if len > 0.0 {
            let dir = (pv - pu) * (1.0 / len);
            for (k, &t) in m.edge_tris[e as usize].iter().enumerate() {
                if t == NONE || m.status[t as usize] != NONE {
                    continue;
                }
                let Some((h, proj, obs, via_here)) = self.apex_of(li, e, k, t, (pu, dir), None) else {
                    continue;
                };
                for (i, it) in items.iter().enumerate() {
                    if it.sides[k] <= 1
                        && let Some(half) = self.apex_half(h, obs, via_here, it)
                    {
                        if it.sides[k] == 0 {
                            bounds[i].1 = bounds[i].1.min(proj - half);
                        } else {
                            bounds[i].0 = bounds[i].0.max(proj + half);
                        }
                    }
                }
                if let Some(half) = self.apex_half(h, obs, via_here, new) {
                    apex[k] = (proj - half, proj + half);
                }
            }
        }
        let placed = items
            .iter()
            .enumerate()
            .map(|(i, it)| Placed {
                need_u: self.end_need(li, e, 0, it) + margin + hot,
                need_v: self.end_need(li, e, 1, it) + margin + hot,
                bounds: bounds[i],
                gap_next: items.get(i + 1).map_or(0.0, |b| Self::gap(self.board, it, b)),
                gap_to_new: Self::gap(self.board, it, new),
                gap_from_new: Self::gap(self.board, new, it),
            })
            .collect();
        EdgeView {
            placed,
            len: m.edge_len(e),
            hot,
            need_u: self.end_need(li, e, 0, new) + margin + hot,
            need_v: self.end_need(li, e, 1, new) + margin + hot,
            apex,
        }
    }

    /// Interval for the searched track at slot j of an edge, with the given apex sides:
    /// `pack_with` over the placed items with the track inserted at j.
    fn slot_interval(view: &EdgeView, sides: [u8; 2], j: usize, lo: &mut Vec<f64>) -> Option<(f64, f64)> {
        let placed = &view.placed[..];
        let n = placed.len();
        let extra = view.hot;
        let mut bounds = (f64::NEG_INFINITY, f64::INFINITY);
        for k in 0..2 {
            match sides[k] {
                0 => bounds.1 = bounds.1.min(view.apex[k].0),
                1 => bounds.0 = bounds.0.max(view.apex[k].1),
                _ => {}
            }
        }
        lo.clear();
        let mut x = f64::NEG_INFINITY;
        for (i, p) in placed[..j].iter().enumerate() {
            let mut y = p.need_u.max(p.bounds.0);
            if i > 0 {
                y = y.max(x + placed[i - 1].gap_next + extra);
            }
            x = y;
            lo.push(x);
        }
        let mut y = view.need_u.max(bounds.0);
        if j > 0 {
            y = y.max(x + placed[j - 1].gap_to_new + extra);
        }
        x = y;
        lo.push(x);
        for (i, p) in placed[j..].iter().enumerate() {
            let g = if i == 0 {
                p.gap_from_new
            } else {
                placed[j + i - 1].gap_next
            };
            x = p.need_u.max(p.bounds.0).max(x + g + extra);
            lo.push(x);
        }
        let mut h = f64::INFINITY;
        for (i, p) in placed[j..].iter().enumerate().rev() {
            let mut y = (view.len - p.need_v).min(p.bounds.1);
            if j + i + 1 < n {
                y = y.min(h - p.gap_next - extra);
            }
            h = y;
            if lo[j + 1 + i] > h + 1e-9 {
                return None;
            }
        }
        let mut y = (view.len - view.need_v).min(bounds.1);
        if j < n {
            y = y.min(h - placed[j].gap_from_new - extra);
        }
        h = y;
        let a = lo[j];
        let res = (a, h);
        if a > h + 1e-9 {
            return None;
        }
        for (i, p) in placed[..j].iter().enumerate().rev() {
            let g = if i + 1 == j { p.gap_to_new } else { p.gap_next };
            h = (view.len - p.need_v).min(p.bounds.1).min(h - g - extra);
            if lo[i] > h + 1e-9 {
                return None;
            }
        }
        Some(if res.0 > res.1 {
            let m = (res.0 + res.1) / 2.0;
            (m, m)
        } else {
            res
        })
    }

    /// For a slot without an interval (see `slot_interval`), in a negotiation: by how many mm
    /// the items overflow the edge with the track at slot j, and a position for the track.
    fn slot_overflow(view: &EdgeView, sides: [u8; 2], j: usize, lo: &mut Vec<f64>) -> (f64, f64) {
        let placed = &view.placed[..];
        let n = placed.len();
        let extra = view.hot;
        let mut bounds = (f64::NEG_INFINITY, f64::INFINITY);
        for k in 0..2 {
            match sides[k] {
                0 => bounds.1 = bounds.1.min(view.apex[k].0),
                1 => bounds.0 = bounds.0.max(view.apex[k].1),
                _ => {}
            }
        }
        // positions 0..=n with the track at j; the gap between i and i + 1
        let gap = |i: usize| {
            extra
                + match i.cmp(&j) {
                    Ordering::Less if i + 1 == j => placed[i].gap_to_new,
                    Ordering::Less => placed[i].gap_next,
                    Ordering::Equal => placed[j].gap_from_new,
                    Ordering::Greater => placed[i - 1].gap_next,
                }
        };
        let item = |i: usize| if i < j { &placed[i] } else { &placed[i - 1] };
        lo.clear();
        for i in 0..=n {
            let own = if i == j {
                view.need_u.max(bounds.0)
            } else {
                item(i).need_u.max(item(i).bounds.0)
            };
            lo.push(if i > 0 { own.max(lo[i - 1] + gap(i - 1)) } else { own });
        }
        let (mut hi, mut over, mut at) = (f64::INFINITY, 0.0_f64, 0.0);
        for i in (0..=n).rev() {
            let own = if i == j {
                (view.len - view.need_v).min(bounds.1)
            } else {
                (view.len - item(i).need_v).min(item(i).bounds.1)
            };
            hi = if i < n { own.min(hi - gap(i)) } else { own };
            over = over.max(lo[i] - hi);
            if i == j {
                at = (lo[i] + hi) / 2.0;
            }
        }
        (over, at.clamp(0.0, view.len))
    }

    /// By how many mm the crossings of edge e overflow it (0 = they fit).
    fn edge_overflow(&self, li: usize, e: u32) -> f64 {
        let items = self.items_on(li, e);
        let m = self.m(li);
        let [u, v] = m.edges[e as usize];
        let len = m.edge_len(e);
        let margin = self.cfg.margin;
        let bounds = self.apex_bounds(li, e, &items, None);
        let hot = self.hot_extra(m.verts[u as usize], m.verts[v as usize]);
        let n = items.len();
        let mut lo = vec![0.0; n];
        for i in 0..n {
            let mut x = (self.end_need(li, e, 0, &items[i]) + margin + hot).max(bounds[i].0);
            if i > 0 {
                x = x.max(lo[i - 1] + Self::gap(self.board, &items[i - 1], &items[i]) + hot);
            }
            lo[i] = x;
        }
        let (mut hi, mut over) = (f64::INFINITY, 0.0_f64);
        for i in (0..n).rev() {
            let mut y = (len - (self.end_need(li, e, 1, &items[i]) + margin + hot)).min(bounds[i].1);
            if i + 1 < n {
                y = y.min(hi - Self::gap(self.board, &items[i], &items[i + 1]) - hot);
            }
            hi = y;
            over = over.max(lo[i] - hi);
        }
        over
    }

    // ------------------------------------------------------------------ chords

    /// Fraction along edge e (from edges[e][0]) of a crossing: the slot of a new crossing, or
    /// the position of `seg` in the list.
    fn edge_frac(&self, li: usize, e: u32, seg: Option<u32>, slot: Option<usize>) -> f64 {
        if self.soft_on {
            return self.soft_frac(li, e, seg, slot);
        }
        let list = &self.cross[li][e as usize];
        let n = list.len() as f64;
        match (slot, seg) {
            (Some(j), _) => (j as f64 + 0.5) / (n + 1.0),
            (None, Some(s)) => (list.iter().position(|&x| x == s).unwrap_or(0) as f64 + 1.0) / (n + 1.0),
            _ => 0.5,
        }
    }

    /// Cyclic boundary coordinate of a position in triangle t.
    fn coord(&self, t: u32, li: usize, p: CPos, seg: Option<u32>, slot: Option<usize>) -> f64 {
        match p {
            CPos::Vertex(i) => i as f64,
            CPos::Edge(k) => {
                let m = self.m(li);
                let e = m.tri_edges[t as usize][k as usize];
                let f = self.edge_frac(li, e, seg, slot);
                let forward = m.tris[t as usize][k as usize] == m.edges[e as usize][0];
                k as f64 + if forward { f } else { 1.0 - f }
            }
        }
    }

    /// Approximate point of a boundary position (for geometric chord tests).
    fn pos_point(&self, t: u32, li: usize, p: CPos, seg: Option<u32>, slot: Option<usize>) -> Point {
        let m = self.m(li);
        match p {
            CPos::Vertex(i) => m.verts[m.tris[t as usize][i as usize] as usize],
            CPos::Edge(k) => {
                let e = m.tri_edges[t as usize][k as usize];
                let f = self.edge_frac(li, e, seg, slot);
                let [u, v] = m.edges[e as usize];
                m.verts[u as usize].lerp(m.verts[v as usize], f)
            }
        }
    }

    /// Can a chord of `net` from `a` (slot sa) to `b` (slot sb) be added to triangle t?
    #[allow(clippy::too_many_arguments)]
    fn chord_ok(&self, t: u32, li: usize, net: NetId, a: CPos, sa: Option<usize>, b: CPos, sb: Option<usize>) -> bool {
        let list = &self.chords[li][t as usize];
        if list.is_empty() {
            return true;
        }
        let x = self.coord(t, li, a, None, sa);
        let y = self.coord(t, li, b, None, sb);
        let (lo, hi) = if x < y { (x, y) } else { (y, x) };
        for ch in list {
            if self.segs[ch.seg as usize].as_ref().unwrap().net == net {
                continue;
            }
            let c = self.coord(t, li, ch.a, Some(ch.seg), None);
            let d = self.coord(t, li, ch.b, Some(ch.seg), None);
            let inside = |z: f64| z > lo + 1e-12 && z < hi - 1e-12;
            let on = |z: f64| (z - lo).abs() < 1e-12 || (z - hi).abs() < 1e-12;
            if on(c) || on(d) {
                continue;
            }
            if inside(c) != inside(d) {
                return false;
            }
        }
        true
    }

    /// The chords of other nets in t by their boundary coordinates, for `chord_fits`.
    fn chord_coords(&self, t: u32, li: usize, net: NetId, out: &mut Vec<(f64, f64)>) {
        out.clear();
        for ch in &self.chords[li][t as usize] {
            if self.segs[ch.seg as usize].as_ref().unwrap().net != net {
                out.push((
                    self.coord(t, li, ch.a, Some(ch.seg), None),
                    self.coord(t, li, ch.b, Some(ch.seg), None),
                ));
            }
        }
    }

    /// `chord_ok` for a chord between boundary coordinates x and y.
    fn chord_fits(coords: &[(f64, f64)], x: f64, y: f64) -> bool {
        let (lo, hi) = if x < y { (x, y) } else { (y, x) };
        coords.iter().all(|&(c, d)| {
            let inside = |z: f64| z > lo + 1e-12 && z < hi - 1e-12;
            let on = |z: f64| (z - lo).abs() < 1e-12 || (z - hi).abs() < 1e-12;
            on(c) || on(d) || inside(c) == inside(d)
        })
    }

    /// How many of the chords a chord between boundary coordinates x and y crosses.
    fn chord_hits(coords: &[(f64, f64)], x: f64, y: f64) -> usize {
        let (lo, hi) = if x < y { (x, y) } else { (y, x) };
        coords
            .iter()
            .filter(|&&(c, d)| {
                let inside = |z: f64| z > lo + 1e-12 && z < hi - 1e-12;
                let on = |z: f64| (z - lo).abs() < 1e-12 || (z - hi).abs() < 1e-12;
                !(on(c) || on(d)) && inside(c) != inside(d)
            })
            .count()
    }

    /// Chords of other nets in t that a new chord from `a` (slot sa) to `b` crosses.
    fn chords_crossed(&self, t: u32, li: usize, net: NetId, a: CPos, sa: Option<usize>, b: CPos) -> usize {
        let mut coords = Vec::new();
        self.chord_coords(t, li, net, &mut coords);
        Self::chord_hits(
            &coords,
            self.coord(t, li, a, None, sa),
            self.coord(t, li, b, None, None),
        )
    }

    /// Chords of other nets in t that a half chord from interior point `p` to `b` crosses.
    fn half_chords_crossed(&self, t: u32, li: usize, net: NetId, p: Point, b: CPos, sb: Option<usize>) -> usize {
        let mut points = Vec::new();
        self.chord_points(t, li, net, &mut points);
        let q = self.pos_point(t, li, b, None, sb);
        points.iter().filter(|&&(c, d)| segs_intersect(p, q, c, d)).count()
    }

    /// The chords of other nets in t as point pairs, for `half_chord_fits`.
    fn chord_points(&self, t: u32, li: usize, net: NetId, out: &mut Vec<(Point, Point)>) {
        out.clear();
        for ch in &self.chords[li][t as usize] {
            if self.segs[ch.seg as usize].as_ref().unwrap().net != net {
                out.push((
                    self.pos_point(t, li, ch.a, Some(ch.seg), None),
                    self.pos_point(t, li, ch.b, Some(ch.seg), None),
                ));
            }
        }
    }

    /// Can a half chord from interior point `p` to boundary position `b` be added to t?
    fn half_chord_ok(&self, t: u32, li: usize, net: NetId, p: Point, b: CPos, sb: Option<usize>) -> bool {
        let list = &self.chords[li][t as usize];
        if list.is_empty() {
            return true;
        }
        let q = self.pos_point(t, li, b, None, sb);
        list.iter().all(|ch| {
            self.segs[ch.seg as usize].as_ref().unwrap().net == net || {
                let c = self.pos_point(t, li, ch.a, Some(ch.seg), None);
                let d = self.pos_point(t, li, ch.b, Some(ch.seg), None);
                !segs_intersect(p, q, c, d)
            }
        })
    }

    // ------------------------------------------------------------------ vias

    fn via_cell(p: Point) -> (i32, i32) {
        (p.x.floor() as i32, p.y.floor() as i32)
    }

    fn via_size(&self, net: NetId) -> (f64, f64, f64) {
        let c = self.board.class_of(net);
        (
            c.via_diameter / 2.0,
            c.via_drill,
            c.clearance.max(self.board.rules.min_clearance),
        )
    }

    /// How many vias a layer change of connection ci takes: one, or on a current path (between
    /// two pins that take the minimum width of their class, [`Board::current_min`]) as many as
    /// carry what that width carries ([`Board::vias_for`]).
    fn via_factor(&self, ci: usize) -> f64 {
        let c = &self.conns[ci];
        let Some(min) = self.board.current_min(c.net) else {
            return 1.0;
        };
        match &c.goal {
            Goal::Pins(goal) if !self.board.narrow_pin(c.from) && goal.iter().any(|&p| !self.board.narrow_pin(p)) => {
                self.board.vias_for(min, self.via_size(c.net).1) as f64
            }
            _ => 1.0,
        }
    }

    fn via_info(&self, net: NetId) -> ViaInfo {
        let (r, drill, clearance) = self.via_size(net);
        ViaInfo {
            net,
            r,
            drill,
            clearance,
            users: 0,
        }
    }

    /// Static and via spacing checks of a via of `net` at site `site`.
    fn site_clear(&self, site: u32, net: NetId) -> bool {
        let (r, drill, clr) = self.via_size(net);
        let s = &self.sub.shared.sites[site as usize];
        let p = s.p;
        for n in &s.nbrs {
            let o = &self.board.obstacles[n.obs as usize];
            let need = if o.net == Some(net) && o.kind != ObstacleKind::Hole {
                match o.hole {
                    Some(_) => drill / 2.0 + self.board.rules.hole_to_hole,
                    None => 0.0,
                }
            } else {
                r + self.eff_clearance(n.obs, clr, net) + self.cfg.margin
            };
            if (n.dist as f64) < need {
                return false;
            }
        }
        let edge = if s.edge.is_nan() {
            self.board.edge_dist(p)
        } else {
            s.edge
        };
        if edge < r + self.board.rules.edge_clearance {
            return false;
        }
        let (cx, cy) = Self::via_cell(p);
        for dx in -2..=2 {
            for dy in -2..=2 {
                if let Some(list) = self.via_cells.get(&(cx + dx, cy + dy)) {
                    for &w in list {
                        if w == site {
                            continue;
                        }
                        let o = &self.vias[&w];
                        let d = p.dist(self.site_point(w));
                        if o.drill == 0.0 {
                            // a branch point of a track: the tracks check it
                            continue;
                        }
                        let holes = drill / 2.0 + o.drill / 2.0 + self.board.rules.hole_to_hole;
                        let need = if o.net == net {
                            holes
                        } else {
                            (r + o.r + clr.max(o.clearance).max(self.board.item_gap(net, Some(o.net)))).max(holes)
                        };
                        if d < need {
                            return false;
                        }
                    }
                }
            }
        }
        true
    }

    /// Can a via of `net` be used at the existing via vertex of `site`? Some(true) if a via of
    /// the net is already there.
    fn via_ok_vertex(&self, site: u32, net: NetId) -> Option<bool> {
        if let Some(via) = self.vias.get(&site) {
            return if via.net == net { Some(true) } else { None };
        }
        if !self.site_clear(site, net) {
            return None;
        }
        if self.soft() {
            return Some(false);
        }
        self.vertex_fits(site, net).then_some(false)
    }

    /// Do the tracks around an unused via vertex keep their room with a via of `net`?
    fn vertex_fits(&self, site: u32, net: NetId) -> bool {
        let info = self.via_info(net);
        for li in 0..self.nl() {
            let m = self.m(li);
            // a via in a pad has no vertex on the pad's layer
            let Some(&v) = m.site_vert.get(&site) else { continue };
            for &t in &m.vert_tris[v as usize] {
                for &e in &m.tri_edges[t as usize] {
                    if self.cross[li][e as usize].is_empty() {
                        continue;
                    }
                    let items = self.items_on(li, e);
                    if self.pack(li, e, &items, Some((site, &info))).is_none() {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// Can a via of `net` be placed at the unused `site` (splitting its triangles)?
    fn via_ok_site(&self, site: u32, net: NetId) -> bool {
        if !self.sub.site_usable(site) || !self.site_clear(site, net) {
            return false;
        }
        self.soft() || self.site_fits(site, net)
    }

    /// Do the tracks around the unused `site` keep their room with a via of `net`?
    fn site_fits(&self, site: u32, net: NetId) -> bool {
        let p = self.site_point(site);
        let via = self.via_info(net);
        for li in 0..self.nl() {
            let m = self.m(li);
            let t = m.site_tri[site as usize];
            if t == PAD_SITE {
                continue;
            }
            let tv = m.tris[t as usize];
            let list = &self.chords[li][t as usize];
            if !list.is_empty() {
                // which new edge (s, v_i) each chord crosses and where
                let mut per_edge: [Vec<(f64, Item)>; 3] = [Vec::new(), Vec::new(), Vec::new()];
                for ch in list {
                    let it = self.item_of(ch.seg);
                    for (i, u) in self.rethread_hits(t, li, ch, p) {
                        per_edge[i].push((u, it));
                    }
                }
                for (i, hits) in per_edge.iter_mut().enumerate() {
                    if hits.is_empty() {
                        continue;
                    }
                    // order from v_i (u = 1) towards s (u = 0)
                    hits.sort_by(|a, b| b.0.total_cmp(&a.0));
                    let items: Vec<Item> = hits.iter().map(|h| h.1).collect();
                    let vi = tv[i];
                    let len = p.dist(m.verts[vi as usize]);
                    let margin = self.cfg.margin;
                    let ok = Self::pack_with(
                        self.board,
                        &items,
                        len,
                        |it| self.vertex_need(li, vi, it) + margin,
                        |it| Self::via_need(self.board, &via, it) + margin,
                        &[],
                        0.0,
                    );
                    if ok.is_none() {
                        return false;
                    }
                }
            }
            // the old edges get the via as apex of their new triangle
            for k in 0..3 {
                let e = m.tri_edges[t as usize][k];
                if self.cross[li][e as usize].is_empty() {
                    continue;
                }
                let side_idx = m.edge_tris[e as usize].iter().position(|&x| x == t).unwrap();
                let mut items = self.items_on(li, e);
                let chords = &self.chords[li][t as usize];
                for (it, &sg) in items.iter_mut().zip(&self.cross[li][e as usize]) {
                    it.sides[side_idx] = 2;
                    let Some(ch) = chords.iter().find(|c| c.seg == sg) else {
                        continue;
                    };
                    let hits = self.rethread_hits(t, li, ch, p);
                    let at_start = ch.a == CPos::Edge(k as u8);
                    let hit = if at_start { hits.first() } else { hits.last() };
                    let x = match hit {
                        Some(&(i, _)) => tv[i],
                        None => match if at_start { ch.b } else { ch.a } {
                            CPos::Vertex(i) => tv[i as usize],
                            CPos::Edge(_) => continue,
                        },
                    };
                    it.sides[side_idx] = if x == m.edges[e as usize][0] { 0 } else { 1 };
                }
                if self
                    .pack_ext(li, e, &items, None, Some(ExtraApex { side: side_idx, p, via }))
                    .is_none()
                {
                    return false;
                }
            }
        }
        true
    }

    /// Where an existing chord of triangle t crosses the new edges (s, v_i) if t is split at p:
    /// (i, u) with u the position along s -> v_i (0 at s, 1 at v_i), in chord order.
    fn rethread_hits(&self, t: u32, li: usize, ch: &Chord, p: Point) -> Vec<(usize, f64)> {
        let m = self.m(li);
        let a = self.pos_point(t, li, ch.a, Some(ch.seg), None);
        let b = self.pos_point(t, li, ch.b, Some(ch.seg), None);
        let tv = m.tris[t as usize];
        // the ray s -> v_i crosses the chord iff v_i and s lie on opposite sides of it
        let mut os = orient(a, b, p);
        if os == 0.0 {
            os = 1e-300;
        }
        let mut hits = Vec::new();
        for i in 0..3 {
            if ch.a == CPos::Vertex(i as u8) || ch.b == CPos::Vertex(i as u8) {
                continue;
            }
            let vi = m.verts[tv[i] as usize];
            let ov = orient(a, b, vi);
            if os * ov >= 0.0 {
                continue;
            }
            let (tc, u) = line_params(a, b, p, vi);
            hits.push((tc, i, u.clamp(1e-6, 1.0 - 1e-6)));
        }
        hits.sort_by(|x, y| x.0.total_cmp(&y.0));
        hits.into_iter().map(|(_, i, u)| (i, u)).collect()
    }

    /// Place a via vertex at `site` in every layer mesh: split the triangle that holds it and
    /// thread the existing chords through the new triangles. Returns per layer the old triangle
    /// and the new ones X_k (X_k holds old edge k).
    pub(crate) fn split_at(&mut self, site: u32) -> Vec<(u32, [u32; 3])> {
        let shared = self.sub.shared.clone();
        let mut res = Vec::new();
        for li in 0..self.nl() {
            let t = self.m(li).site_tri[site as usize];
            if t == PAD_SITE {
                // via in pad: no vertex on the pad's own layer
                self.sub.m[li].site_tri[site as usize] = NONE;
                res.push((NONE, [NONE; 3]));
                continue;
            }
            let p = self.m(li).site_p.get(&site).copied().unwrap_or(self.site_point(site));
            // geometry of the old chords, before the mesh changes
            let mut old: Vec<(Chord, Vec<(usize, f64)>, Point)> = Vec::new();
            for ch in std::mem::take(&mut self.chords[li][t as usize]) {
                let hits = self.rethread_hits(t, li, &ch, p);
                let a = self.pos_point(t, li, ch.a, Some(ch.seg), None);
                let b = self.pos_point(t, li, ch.b, Some(ch.seg), None);
                old.push((ch, hits, a.lerp(b, 1e-6)));
            }
            let old_tv = self.m(li).tris[t as usize];
            let old_edges = self.m(li).tri_edges[t as usize];
            let (_, x) = self.sub.m[li].split(t, site, p, &shared.sites);
            let ne = self.m(li).edges.len();
            self.cross[li].resize(ne, Vec::new());
            let nt = self.m(li).tris.len();
            self.chords[li].resize(nt, Vec::new());
            let mut on_edge: FastMap<u32, Vec<(f64, u32)>> = FastMap::default();
            for (ch, hits, start_pt) in old {
                let m = &self.sub.m[li];
                let new_edge = |k: usize| -> u32 { m.tri_edges[x[k] as usize][2] }; // E_k = (s, v_k)
                let mut cur = match ch.a {
                    CPos::Edge(k) => x[k as usize],
                    // the sub-triangle that has the chord's vertex and contains its start
                    CPos::Vertex(i) => *x
                        .iter()
                        .filter(|&&xt| m.tris[xt as usize].contains(&old_tv[i as usize]))
                        .find(|&&xt| {
                            let tv = m.tris[xt as usize];
                            let (a, b, c) = (
                                m.verts[tv[0] as usize],
                                m.verts[tv[1] as usize],
                                m.verts[tv[2] as usize],
                            );
                            orient(a, b, start_pt) >= -1e-12
                                && orient(b, c, start_pt) >= -1e-12
                                && orient(c, a, start_pt) >= -1e-12
                        })
                        .or_else(|| x.iter().find(|&&xt| m.tris[xt as usize].contains(&old_tv[i as usize])))
                        .unwrap_or(&x[0]),
                };
                // (0 if the vertex is missing: only after a degenerate rethreading, see below)
                let local_of_vertex =
                    |xt: u32, v: u32| m.tris[xt as usize].iter().position(|&q| q == v).unwrap_or(0) as u8;
                // a chord from vertex to vertex runs along an old edge: it stays in the new
                // triangle that holds both vertices, without crossings
                if let (CPos::Vertex(i), CPos::Vertex(j)) = (ch.a, ch.b)
                    && let Some(&xt) = x.iter().find(|&&xt| {
                        let tv = m.tris[xt as usize];
                        tv.contains(&old_tv[i as usize]) && tv.contains(&old_tv[j as usize])
                    })
                {
                    cur = xt;
                }
                let mut pos = match ch.a {
                    CPos::Edge(_) => CPos::Edge(0),
                    CPos::Vertex(i) => CPos::Vertex(local_of_vertex(cur, old_tv[i as usize])),
                };
                let mut hits = hits;
                if let (CPos::Vertex(_), CPos::Vertex(_)) = (ch.a, ch.b) {
                    hits.clear();
                }
                if hits.len() == 2 && !m.tri_edges[cur as usize].contains(&new_edge(hits[0].0)) {
                    hits.swap(0, 1);
                }
                // the walk must end in a triangle that holds the chord's end; if the geometric
                // side test was degenerate, take the direct way (any two of the new triangles
                // are neighbours)
                let holds_end = |xt: u32| match ch.b {
                    CPos::Edge(k) => xt == x[k as usize],
                    CPos::Vertex(i) => m.tris[xt as usize].contains(&old_tv[i as usize]),
                };
                let mut walk = Some(cur);
                for h in &hits {
                    walk = walk.and_then(|c| {
                        let e = new_edge(h.0);
                        m.tri_edges[c as usize].contains(&e).then(|| m.other_tri(e, c))
                    });
                }
                if !walk.is_some_and(holds_end) {
                    hits = if holds_end(cur) {
                        Vec::new()
                    } else {
                        (0..3)
                            .find(|&i| {
                                let e = new_edge(i);
                                m.tri_edges[cur as usize].contains(&e) && holds_end(m.other_tri(e, cur))
                            })
                            .map(|i| vec![(i, 0.5)])
                            .unwrap_or_default()
                    };
                }
                let mut new_cross: Vec<Crossing> = Vec::new();
                let mut new_chords: Vec<(u32, Chord)> = Vec::new();
                let mut new_pts: Vec<Point> = Vec::new();
                for (i, u) in hits {
                    let e = new_edge(i);
                    new_pts.push(p.lerp(m.verts[old_tv[i] as usize], u));
                    let k_out = m.local_edge(cur, e) as u8;
                    new_chords.push((
                        cur,
                        Chord {
                            seg: ch.seg,
                            a: pos,
                            b: CPos::Edge(k_out),
                        },
                    ));
                    new_cross.push(Crossing { e, from: cur });
                    on_edge.entry(e).or_default().push((u, ch.seg));
                    let next = m.other_tri(e, cur);
                    pos = CPos::Edge(m.local_edge(next, e) as u8);
                    cur = next;
                }
                let end = match ch.b {
                    CPos::Edge(_) => CPos::Edge(0),
                    CPos::Vertex(i) => CPos::Vertex(local_of_vertex(cur, old_tv[i as usize])),
                };
                new_chords.push((
                    cur,
                    Chord {
                        seg: ch.seg,
                        a: pos,
                        b: end,
                    },
                ));
                // splice the crossings into the segment
                let seg = self.segs[ch.seg as usize].as_mut().unwrap();
                let chain_pts = seg
                    .imported
                    .as_ref()
                    .map_or(0, |imp| self.board.soft.chains[imp.chain as usize].pts.len() as u32);
                let exit = match ch.b {
                    CPos::Edge(k) => seg
                        .crossings
                        .iter()
                        .position(|c| c.e == old_edges[k as usize] && c.from == t),
                    CPos::Vertex(_) => None,
                };
                if let Some(imp) = seg.imported.as_mut() {
                    imp.thread(exit.unwrap_or(seg.crossings.len()), &new_pts, chain_pts - 2);
                }
                match exit {
                    Some(ix) => {
                        seg.crossings[ix].from = cur;
                        let w = seg.widths.get(ix).copied().unwrap_or(seg.width);
                        for (j, c) in new_cross.into_iter().enumerate() {
                            seg.crossings.insert(ix + j, c);
                            seg.widths.insert(ix + j, w);
                        }
                    }
                    None => {
                        let w = seg.widths.last().copied().unwrap_or(seg.width);
                        for c in new_cross {
                            seg.crossings.push(c);
                            seg.widths.push(w);
                        }
                    }
                }
                seg.chord_tris.retain(|&q| q != t);
                for (xt, c) in new_chords {
                    if !seg.chord_tris.contains(&xt) {
                        seg.chord_tris.push(xt);
                    }
                    self.chords[li][xt as usize].push(c);
                }
            }
            // order on the new edges: from v_i (edges[e][0]) towards s
            for (e, mut list) in on_edge {
                list.sort_by(|a, b| b.0.total_cmp(&a.0));
                self.cross[li][e as usize] = list.into_iter().map(|(_, sg)| sg).collect();
            }
            res.push((t, x));
        }
        res
    }

    fn in_plane(&self, p: Point, net: NetId) -> bool {
        let (r, _, _) = self.via_size(net);
        self.board.planes.iter().any(|pl| pl.net == net && pl.takes_via(p, r))
    }

    /// A via or branch point of the soft copper: it stays for good.
    pub(crate) fn place_soft_via(&mut self, site: u32, n: &crate::import::SoftNode) {
        self.vias.insert(
            site,
            ViaInfo {
                net: n.net,
                r: n.r,
                drill: n.drill,
                clearance: n.clearance,
                users: 1,
            },
        );
        self.via_cells.entry(Self::via_cell(n.at)).or_default().push(site);
    }

    fn add_via(&mut self, site: u32, net: NetId) {
        if let Some(via) = self.vias.get_mut(&site) {
            via.users += 1;
            return;
        }
        let mut info = self.via_info(net);
        info.users = 1;
        self.vias.insert(site, info);
        self.via_cells
            .entry(Self::via_cell(self.site_point(site)))
            .or_default()
            .push(site);
    }

    fn remove_via(&mut self, site: u32) {
        let Some(via) = self.vias.get_mut(&site) else { return };
        via.users -= 1;
        if via.users == 0 {
            self.vias.remove(&site);
            let c = Self::via_cell(self.site_point(site));
            if let Some(list) = self.via_cells.get_mut(&c) {
                list.retain(|&x| x != site);
            }
        }
    }

    // ------------------------------------------------------------------ search

    fn tri_pin(&self, li: usize, t: u32) -> Option<usize> {
        let s = self.m(li).status[t as usize];
        if s == NONE || s == OUTSIDE {
            return None;
        }
        self.board.obstacles[s as usize].pin
    }

    /// A negotiation is on and capacity and planarity are soft.
    fn soft(&self) -> bool {
        self.prices.as_ref().is_some_and(|p| p.soft)
    }

    /// A* for one connection. Returns the node chain from start to goal.
    fn search(&self, ci: usize, buf: &mut SearchBuf) -> Option<Vec<Node>> {
        let conn = &self.conns[ci];
        let guide = self.partner.get(&conn.net).and_then(|&p| {
            let g = self.guides.get(&p)?;
            let (w, gap, _) = self.board.pair_shape(conn.net, p, 0);
            Some((g.as_slice(), w + gap))
        });
        let net = conn.net;
        // a via in a pad is a last resort, except for the nets of `GB_VIP_NETS`, with
        // `GB_VIP_LEARN` the nets that needed one before, and the nets of `cfg.vip_nets`
        // (experiments)
        let via_in_pad = self.cfg.via_in_pad
            || vip_net(&self.board.nets[net as usize].name)
            || vip_learned(net)
            || self.cfg.vip_nets.contains(&net);
        let cls = self.board.class_of(net);
        // a layer change on a current path takes as many vias as carry the current
        let via_cost = self.cfg.via_cost * self.via_factor(ci);
        let item = Item {
            net,
            w: cls.track_width,
            c: cls.clearance.max(self.board.rules.min_clearance),
            sides: [2, 2],
        };
        let (necks, neck_w) = self.necks(ci);
        let src = &self.board.pins[conn.from];
        let target_pts: Vec<Point> = match &conn.goal {
            Goal::Pins(p) => p.iter().map(|&p| self.board.pins[p].pos).collect(),
            Goal::Plane => Vec::new(),
        };
        let h = |p: Point| -> f64 {
            match target_pts[..] {
                [] => 0.0,
                [q] => q.dist(p),
                _ => {
                    // the exact distance only for those that can be the minimum
                    let d2 = target_pts.iter().map(|&q| (q - p).len2()).fold(f64::INFINITY, f64::min);
                    target_pts
                        .iter()
                        .filter(|&&q| (q - p).len2() <= d2 * (1.0 + 1e-9))
                        .map(|&q| q.dist(p))
                        .fold(f64::INFINITY, f64::min)
                }
            }
        };
        let is_target = |pin: usize| -> bool {
            match &conn.goal {
                Goal::Pins(p) => p.iter().any(|&q| self.pin_group[q] == self.pin_group[pin]),
                Goal::Plane => {
                    let pn = &self.board.pins[pin];
                    // a nearby through hole pin of the net reaches the plane as well
                    self.pin_group[pin] != self.pin_group[conn.from]
                        && pn.net == net
                        && pn.pos.dist(src.pos) < 3.0
                        && self
                            .board
                            .planes
                            .iter()
                            .any(|pl| pl.net == net && pn.layers.has(pl.layer) && pl.covers(pn.pos))
                }
            }
        };
        let nl = self.nl();
        let tappable = self.tappable(ci);

        // a search that reaches its limit leaves a large table: clearing that for every small
        // search would cost more than growing a new one now and then
        if buf.best.capacity() > 1 << 20 {
            buf.best = Best::default();
        }
        buf.best.clear();
        buf.nodes.clear();
        buf.open.clear();
        let SearchBuf { nodes, best, open } = buf;
        let push =
            |best: &mut Best, nodes: &mut Vec<Node>, open: &mut OpenHeap, kind: NK, pos: Point, g: f64, parent: u32| {
                // a node that a cheaper one replaces is stale: every later push that gets through
                // here is cheaper by more than 1e-9 than any node of its state before
                let at = nodes.len() as u32;
                match best.entry(key(&kind)) {
                    Entry::Occupied(mut b) => {
                        let (old, old_at) = *b.get();
                        if old <= g + 1e-9 {
                            return;
                        }
                        b.insert((g, at));
                        nodes[old_at as usize].stale = true;
                    }
                    Entry::Vacant(b) => {
                        b.insert((g, at));
                    }
                }
                let (seen, block_up, block_len) = match (kind, nodes.get(parent as usize)) {
                    (NK::Tri { t, li, .. }, Some(n)) if n.block_len == BLOCK => (tri_bit(t, li), parent, 1),
                    (NK::Tri { t, li, .. }, Some(n)) if n.block_len > 0 => {
                        (n.seen | tri_bit(t, li), n.block_up, n.block_len + 1)
                    }
                    (NK::Tri { t, li, .. }, _) => (tri_bit(t, li), u32::MAX, 1),
                    _ => (0, u32::MAX, 0),
                };
                let via_up = match nodes.get(parent as usize) {
                    Some(Node {
                        kind: NK::Site { .. } | NK::Vert { .. },
                        ..
                    }) => parent,
                    Some(n) => n.via_up,
                    None => u32::MAX,
                };
                nodes.push(Node {
                    kind,
                    pos,
                    g,
                    parent,
                    seen,
                    block_up,
                    block_len,
                    via_up,
                    stale: false,
                });
                // a way to the net right here: the pins of the goal say nothing about it
                let f = if matches!(kind, NK::GoalTap { .. }) {
                    g
                } else {
                    g + h(pos)
                };
                open.push(f, (nodes.len() - 1) as u32);
            };

        for (li, m) in self.sub.m.iter().enumerate() {
            if !src.layers.has(m.layer) {
                continue;
            }
            let group = self.pin_group[conn.from];
            // any pin of the group can start: overlapping pads, or pins on one piece of kept
            // copper
            let starts: Vec<(usize, Point)> = (0..self.board.pins.len())
                .filter(|&q| self.pin_group[q] == group && self.board.pins[q].layers.has(m.layer))
                .flat_map(|q| {
                    let pin = &self.board.pins[q];
                    let pos = if self.has_kept { pin.pos } else { src.pos };
                    pin.obstacles.iter().map(move |&o| (o, pos))
                })
                .collect();
            for &(o, pos) in &starts {
                for &t in m.obs_tris.get(&(o as u32)).map(|v| v.as_slice()).unwrap_or(&[]) {
                    push(
                        best,
                        nodes,
                        open,
                        NK::Tri {
                            t,
                            li: li as u8,
                            entry: 3,
                            slot: 0,
                        },
                        pos,
                        0.0,
                        u32::MAX,
                    );
                }
            }
        }

        let mut views: FastMap<(u8, u32), EdgeView> = FastMap::default();
        let mut scratch: Vec<f64> = Vec::new();
        let prices = self.prices.as_ref();
        let soft = prices.filter(|p| p.soft);
        let (mut coords, mut points): (Vec<(f64, f64)>, Vec<(Point, Point)>) = (Vec::new(), Vec::new());
        // crossings out of triangle t (layer lu) through local edge k
        #[allow(clippy::too_many_arguments)]
        let mut cross_out = |best: &mut Best,
                             nodes: &mut Vec<Node>,
                             open: &mut OpenHeap,
                             t: u32,
                             lu: usize,
                             k: u8,
                             chord_from: Option<(CPos, Option<usize>)>,
                             interior: Option<Point>,
                             node: &Node,
                             idx: u32| {
            let m = self.m(lu);
            let t2 = m.tri_nbrs[t as usize][k as usize];
            if t2 == NONE {
                return;
            }
            let s2 = m.status[t2 as usize];
            if !Substrate::passable(self.board, s2, net) || m.closed(t2, net) {
                return;
            }
            let free = m.status[t as usize] == NONE;
            let e = m.tri_edges[t as usize][k as usize];
            let k2 = m.local_edge(t2, e) as u8;
            if !free && s2 != NONE {
                push(
                    best,
                    nodes,
                    open,
                    NK::Tri {
                        t: t2,
                        li: lu as u8,
                        entry: 3,
                        slot: 0,
                    },
                    node.pos,
                    node.g,
                    idx,
                );
                return;
            }
            // never enter a triangle this path already passed on this layer (no loops)
            let bit = tri_bit(t2, lu as u8);
            let mut block = idx;
            while block != u32::MAX {
                let top = &nodes[block as usize];
                if top.seen & bit == bit {
                    let mut p = block;
                    while p != top.block_up {
                        let NK::Tri { t: tp, li: lp, .. } = nodes[p as usize].kind else {
                            break;
                        };
                        if tp == t2 && lp as usize == lu {
                            return;
                        }
                        p = nodes[p as usize].parent;
                    }
                }
                block = top.block_up;
            }
            let mut item = item;
            item.w = self.width_at_edge(lu, e, item.w, &necks, neck_w);
            let mut price = prices.map_or(0.0, |p| p.history[lu].get(e as usize).copied().unwrap_or(0.0));
            if free && let Some((cp, cs)) = chord_from {
                // the chord in t fixes the side of t's apex on the exit edge ...
                let side_idx = m.edge_tris[e as usize].iter().position(|&x| x == t).unwrap();
                item.sides[side_idx] = self.side_of_pos(lu, t, e, cp);
                // ... and on the entry edge, which must still fit
                if let (CPos::Edge(ke), Some(js)) = (cp, cs) {
                    let e_in = m.tri_edges[t as usize][ke as usize];
                    let mut it_in = item;
                    it_in.sides = [2, 2];
                    it_in.w = self.width_at_edge(lu, e_in, cls.track_width, &necks, neck_w);
                    let si = m.edge_tris[e_in as usize].iter().position(|&x| x == t).unwrap();
                    it_in.sides[si] = self.side_of_pos(lu, t, e_in, CPos::Edge(k));
                    let view = views
                        .entry((lu as u8, e_in))
                        .or_insert_with(|| self.edge_view(lu, e_in, &it_in));
                    if Self::slot_interval(view, it_in.sides, js, &mut scratch).is_none() {
                        let Some(p) = soft else { return };
                        price += p.crossing + p.overflow * Self::slot_overflow(view, it_in.sides, js, &mut scratch).0;
                    }
                }
            }
            let [u, v] = m.edges[e as usize];
            let (pu, pv) = (m.verts[u as usize], m.verts[v as usize]);
            let len = pu.dist(pv);
            let dir = (pv - pu) * (1.0 / len);
            let n = self.cross[lu][e as usize].len();
            let escape = self.escape_penalty(lu, e, net);
            let view = views
                .entry((lu as u8, e))
                .or_insert_with(|| self.edge_view(lu, e, &item));
            // the chords in t stay the same from slot to slot
            if free && chord_from.is_some() {
                self.chord_coords(t, lu, net, &mut coords);
            }
            if free && interior.is_some() {
                self.chord_points(t, lu, net, &mut points);
            }
            let from_coord = chord_from.map(|(cp, cs)| self.coord(t, lu, cp, None, cs));
            let node_at = |j: usize, lo: f64, hi: f64, price: f64| {
                let q = pu + dir * (node.pos - pu).dot(dir).clamp(lo, hi);
                // on kept copper of the net the way is already there
                let g = if self.on_kept(lu, t2) {
                    node.g
                } else {
                    let step = node.pos.dist(q);
                    node.g + step + self.across(lu, node.pos, q) + escape + self.pull(guide, lu, q, step) + price
                };
                let kind = NK::Tri {
                    t: t2,
                    li: lu as u8,
                    entry: k2,
                    slot: j as u16,
                };
                (kind, q, g)
            };
            // in a negotiation only the cheapest slot with a conflict counts: the search would
            // lose itself in the many ways through a bundle
            let mut cheapest: Option<(f64, usize, f64, f64)> = None;
            for j in 0..=n {
                let mut price = price;
                let mut conflict = false;
                if free {
                    if let Some(x) = from_coord {
                        let y = self.coord(t, lu, CPos::Edge(k), None, Some(j));
                        match soft {
                            Some(s) => {
                                let hits = Self::chord_hits(&coords, x, y);
                                price += s.crossing * hits as f64;
                                conflict = hits > 0;
                            }
                            None if !Self::chord_fits(&coords, x, y) => continue,
                            None => {}
                        }
                    }
                    if let Some(p) = interior {
                        let q = self.pos_point(t, lu, CPos::Edge(k), None, Some(j));
                        let hit = |&&(c, d): &&(Point, Point)| segs_intersect(p, q, c, d);
                        match soft {
                            Some(s) => {
                                let hits = points.iter().filter(hit).count();
                                price += s.crossing * hits as f64;
                                conflict |= hits > 0;
                            }
                            None if points.iter().any(|x| hit(&x)) => continue,
                            None => {}
                        }
                    }
                }
                let (lo, hi) = match (Self::slot_interval(view, item.sides, j, &mut scratch), soft) {
                    (Some(i), _) => i,
                    (None, Some(s)) => {
                        if s.solid && self.pack(lu, e, &[item], None).is_none() {
                            continue;
                        }
                        let (over, at) = Self::slot_overflow(view, item.sides, j, &mut scratch);
                        price += s.crossing + s.overflow * over;
                        conflict = true;
                        (at, at)
                    }
                    (None, None) => continue,
                };
                if let Some(p) = prices {
                    let room = p.reserve * (n + 2) as f64;
                    if hi - lo < room {
                        price += p.tight * (1.0 - (hi - lo) / room);
                    }
                }
                if conflict {
                    if cheapest.is_none_or(|c| price < c.0) {
                        cheapest = Some((price, j, lo, hi));
                    }
                    continue;
                }
                let (kind, q, g) = node_at(j, lo, hi, price);
                push(best, nodes, open, kind, q, g, idx);
            }
            if let Some((price, j, lo, hi)) = cheapest {
                let (kind, q, g) = node_at(j, lo, hi, price);
                push(best, nodes, open, kind, q, g, idx);
            }
        };

        let tracing = self.trace.lock().unwrap().is_some();
        // via checks depend on the routes and the net only, so one per site and search will do
        let mut site_ok: FastMap<u32, bool> = FastMap::default();
        // the sites of a triangle by distance (ties in the order of the triangle), and the vias
        // of a path
        let mut cand: Vec<(f64, u32, u32)> = Vec::new();
        let mut own_vias: Vec<Point> = Vec::new();
        let mut vertex_ok: FastMap<u32, Option<bool>> = FastMap::default();
        let mut via_fits: FastMap<u32, bool> = FastMap::default();
        let mut expansions = 0;
        let check_at = goal_check_at();
        while let Some(idx) = open.pop() {
            let node = nodes[idx as usize];
            if node.stale {
                continue;
            }
            expansions += 1;
            if expansions > self.cfg.max_expansions {
                return None;
            }
            // a long search: is the goal shut in? Then it can only fail (see `shut`)
            if expansions == check_at {
                crate::prof::add(crate::prof::Slot::GoalChecks, 1);
                if self.goal_shut_in(ci, item, &necks, neck_w, &tappable, via_in_pad, GOAL_CHECK_BUDGET) {
                    crate::prof::add(crate::prof::Slot::GoalShut, 1);
                    GOAL_SHUT_TEST.with(|c| c.set(c.get() + 1));
                    return None;
                }
            }
            if tracing && let Some(t) = self.trace.lock().unwrap().as_mut() {
                let li = match node.kind {
                    NK::Tri { li, .. } | NK::Vert { li, .. } | NK::Site { li, .. } => li,
                    _ => 0,
                };
                t.push((node.pos, li));
            }
            match node.kind {
                NK::GoalPin { .. } | NK::GoalVia { .. } | NK::GoalSite { .. } | NK::GoalTap { .. } => {
                    let mut chain = vec![node];
                    let mut p = node.parent;
                    while p != u32::MAX {
                        chain.push(nodes[p as usize]);
                        p = nodes[p as usize].parent;
                    }
                    chain.reverse();
                    return Some(chain);
                }
                NK::Tri { t, li, entry, slot } => {
                    let lu = li as usize;
                    let m = self.m(lu);
                    let status = m.status[t as usize];
                    let free = status == NONE;
                    let entry_pos = if entry < 3 {
                        Some((CPos::Edge(entry), Some(slot as usize)))
                    } else {
                        None
                    };
                    if let Some(pin) = self.tri_pin(lu, t)
                        && is_target(pin)
                    {
                        let q = self.board.pins[pin].pos;
                        push(
                            best,
                            nodes,
                            open,
                            NK::GoalPin { pin },
                            q,
                            node.g + node.pos.dist(q),
                            idx,
                        );
                        continue;
                    }
                    // right next to a track of the net that is not on this side of the gap
                    if free && entry < 3 && !tappable.is_empty() {
                        let e = m.tri_edges[t as usize][entry as usize];
                        let list = &self.cross[lu][e as usize];
                        let slot = slot as usize;
                        let next = [slot.checked_sub(1), Some(slot)];
                        if let Some(&seg) = next
                            .iter()
                            .flatten()
                            .filter_map(|&i| list.get(i))
                            .find(|s| tappable.contains(s))
                        {
                            push(best, nodes, open, NK::GoalTap { seg, e }, node.pos, node.g, idx);
                            continue;
                        }
                    }
                    // via in pad: a plane fan-out may end on a via inside the source pad
                    if !free
                        && via_in_pad
                        && let Some(pin) = self.tri_pin(lu, t)
                        && self.pin_group[pin] == self.pin_group[conn.from]
                        && let Some(list) = self.sub.shared.pad_sites.get(&(pin as u32))
                    {
                        for &site in list {
                            let q = self.site_point(site);
                            if pad_reuse() && conn.goal != Goal::Plane && !self.sub.site_usable(site) {
                                // a via in this pad was placed and ripped up: the site stays split,
                                // but its vertices on the other layers can take a via again
                                let ok = *vertex_ok.entry(site).or_insert_with(|| self.via_ok_vertex(site, net));
                                if let Some(existing) = ok {
                                    let vc = if existing { 0.1 } else { via_cost * 2.0 };
                                    for l2 in 0..nl {
                                        if l2 != lu && self.m(l2).site_vert.contains_key(&site) {
                                            push(
                                                best,
                                                nodes,
                                                open,
                                                NK::Vert { site, li: l2 as u8 },
                                                q,
                                                node.g + node.pos.dist(q) + vc,
                                                idx,
                                            );
                                        }
                                    }
                                }
                                continue;
                            }
                            let ok = *site_ok.entry(site).or_insert_with(|| self.via_ok_site(site, net));
                            if !self.sub.site_usable(site) || !ok {
                                continue;
                            }
                            if conn.goal == Goal::Plane {
                                if self.in_plane(q, net) {
                                    push(
                                        best,
                                        nodes,
                                        open,
                                        NK::GoalSite { site },
                                        q,
                                        node.g + node.pos.dist(q) + via_cost,
                                        idx,
                                    );
                                }
                            } else {
                                // leave the pad through a via to the other layers
                                for l2 in 0..nl {
                                    if l2 != lu && self.m(l2).site_tri[site as usize] != PAD_SITE {
                                        push(
                                            best,
                                            nodes,
                                            open,
                                            NK::Site { site, li: l2 as u8 },
                                            q,
                                            node.g + node.pos.dist(q) + via_cost * 2.0,
                                            idx,
                                        );
                                    }
                                }
                            }
                        }
                    }
                    // through hole copper of the net: switch layers for free
                    if !free && status != OUTSIDE && self.board.obstacles[status as usize].hole.is_some() {
                        // a kept via: the other layer starts at its centre
                        let o = &self.board.obstacles[status as usize];
                        let at = if let (ObstacleKind::Via, Shape::Circle { c, .. }) = (o.kind, &o.shape) {
                            *c
                        } else {
                            node.pos
                        };
                        for l2 in 0..nl {
                            if l2 == lu {
                                continue;
                            }
                            for &t2 in self.m(l2).obs_tris.get(&status).map(|v| v.as_slice()).unwrap_or(&[]) {
                                push(
                                    best,
                                    nodes,
                                    open,
                                    NK::Tri {
                                        t: t2,
                                        li: l2 as u8,
                                        entry: 3,
                                        slot: 0,
                                    },
                                    at,
                                    node.g,
                                    idx,
                                );
                            }
                        }
                    }
                    for k in 0..3u8 {
                        if k != entry {
                            cross_out(best, nodes, open, t, lu, k, entry_pos, None, &node, idx);
                        }
                    }
                    if !free {
                        continue;
                    }
                    // existing via vertices at the corners
                    for i in 0..3u8 {
                        let v = m.tris[t as usize][i as usize];
                        let site = m.vert_site[v as usize];
                        if site == NONE || site == STEINER {
                            continue;
                        }
                        let ok = *vertex_ok.entry(site).or_insert_with(|| self.via_ok_vertex(site, net));
                        let Some(existing) = ok else {
                            continue;
                        };
                        let mut vc = if existing { 0.1 } else { via_cost };
                        if let Some(s) = soft {
                            if let Some((ep, es)) = entry_pos {
                                vc += s.crossing * self.chords_crossed(t, lu, net, ep, es, CPos::Vertex(i)) as f64;
                            }
                            if !existing && !*via_fits.entry(site).or_insert_with(|| self.vertex_fits(site, net)) {
                                vc += s.crossing;
                            }
                        } else if let Some((ep, es)) = entry_pos
                            && !self.chord_ok(t, lu, net, ep, es, CPos::Vertex(i), None)
                        {
                            continue;
                        }
                        let q = m.verts[v as usize];
                        let d = node.pos.dist(q);
                        if conn.goal == Goal::Plane && self.in_plane(q, net) {
                            push(best, nodes, open, NK::GoalVia { site }, q, node.g + d + vc * 0.5, idx);
                            continue;
                        }
                        for l2 in 0..nl {
                            if l2 != lu && self.m(l2).site_vert.contains_key(&site) {
                                push(
                                    best,
                                    nodes,
                                    open,
                                    NK::Vert { site, li: l2 as u8 },
                                    q,
                                    node.g + d + vc,
                                    idx,
                                );
                            }
                        }
                    }
                    // new vias on sites inside the triangle (nearest few)
                    let sites = &m.tri_sites[t as usize];
                    if sites.is_empty() {
                        continue;
                    }
                    cand.clear();
                    cand.extend(
                        sites
                            .iter()
                            .filter(|&&s| match self.sub.shared.sites[s as usize].pad {
                                None => true,
                                // a via into a target pad ends the connection (last resort)
                                Some(p) => via_in_pad && is_target(p as usize),
                            })
                            .enumerate()
                            .map(|(i, &s)| (self.site_point(s).dist(node.pos), i as u32, s)),
                    );
                    let order = |a: &(f64, u32, u32), b: &(f64, u32, u32)| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1));
                    // only the few nearest are tried: put them in order a few at a time
                    let mut ordered = 0;
                    let mut tried = 0;
                    // vias of this path so far (hole to hole spacing within one connection)
                    own_vias.clear();
                    {
                        let mut p = node.via_up;
                        while p != u32::MAX {
                            if let NK::Site { site, .. } | NK::Vert { site, .. } = nodes[p as usize].kind {
                                own_vias.push(self.site_point(site));
                            }
                            p = nodes[p as usize].via_up;
                        }
                    }
                    let (_, own_drill, _) = self.via_size(net);
                    let hole_gap = own_drill + self.board.rules.hole_to_hole;
                    for at in 0..cand.len() {
                        if tried >= self.cfg.sites_per_tri {
                            break;
                        }
                        if at == ordered {
                            let rest = &mut cand[at..];
                            let k = rest.len().min(4);
                            if k < rest.len() {
                                rest.select_nth_unstable_by(k - 1, order);
                            }
                            rest[..k].sort_unstable_by(order);
                            ordered = at + k;
                        }
                        let (d, _, site) = cand[at];
                        let q = self.site_point(site);
                        if own_vias.iter().any(|v| v.dist(q) < hole_gap) {
                            continue;
                        }
                        let mut vc = via_cost;
                        if let Some(s) = soft {
                            if let Some((ep, es)) = entry_pos {
                                vc += s.crossing * self.half_chords_crossed(t, lu, net, q, ep, es) as f64;
                            }
                        } else if let Some((ep, es)) = entry_pos
                            && !self.half_chord_ok(t, lu, net, q, ep, es)
                        {
                            continue;
                        }
                        if !*site_ok.entry(site).or_insert_with(|| self.via_ok_site(site, net)) {
                            continue;
                        }
                        if let Some(s) = soft
                            && !*via_fits.entry(site).or_insert_with(|| self.site_fits(site, net))
                        {
                            vc += s.crossing;
                        }
                        tried += 1;
                        if self.sub.shared.sites[site as usize].pad.is_some()
                            || (conn.goal == Goal::Plane && self.in_plane(q, net))
                        {
                            push(
                                best,
                                nodes,
                                open,
                                NK::GoalSite { site },
                                q,
                                node.g + d + vc - via_cost * 0.5,
                                idx,
                            );
                            continue;
                        }
                        for l2 in 0..nl {
                            if l2 != lu {
                                push(
                                    best,
                                    nodes,
                                    open,
                                    NK::Site { site, li: l2 as u8 },
                                    q,
                                    node.g + d + vc,
                                    idx,
                                );
                            }
                        }
                    }
                }
                NK::Vert { site, li } => {
                    let lu = li as usize;
                    let m = self.m(lu);
                    let Some(&v) = m.site_vert.get(&site) else { continue };
                    for &t in &m.vert_tris[v as usize] {
                        let status = m.status[t as usize];
                        if !Substrate::passable(self.board, status, net) || m.closed(t, net) {
                            continue;
                        }
                        let i = m.tris[t as usize].iter().position(|&x| x == v).unwrap() as u8;
                        let k = (i + 1) % 3;
                        cross_out(
                            best,
                            nodes,
                            open,
                            t,
                            lu,
                            k,
                            Some((CPos::Vertex(i), None)),
                            None,
                            &node,
                            idx,
                        );
                    }
                }
                NK::Site { site, li } => {
                    let lu = li as usize;
                    let t = self.m(lu).site_tri[site as usize];
                    for k in 0..3u8 {
                        cross_out(best, nodes, open, t, lu, k, None, Some(node.pos), &node, idx);
                    }
                }
            }
        }
        None
    }

    // ------------------------------------------------------------------ commit / rip-up

    /// Commit a found chain. Returns false (after placing nothing but mesh splits) if the chain
    /// no longer fits the mesh after its own vias split triangles; the caller searches again.
    fn commit(&mut self, ci: usize, chain: &[Node]) -> bool {
        let mut chain = chain.to_vec();
        let net = self.conns[ci].net;
        // Place new vias first: split their triangles and rewrite the chain to the new mesh.
        let mut split_tris: Vec<(usize, u32)> = Vec::new(); // (layer, old triangle)
        for i in 0..chain.len() {
            let (site, li) = match chain[i].kind {
                NK::Site { site, li } => (site, Some(li)),
                NK::GoalSite { site } => (site, None),
                _ => continue,
            };
            if !self.sub.site_usable(site) {
                return false;
            }
            let per_layer = self.split_at(site);
            for (l, &(t, _)) in per_layer.iter().enumerate() {
                split_tris.push((l, t));
            }
            chain[i].kind = match li {
                Some(li) => NK::Vert { site, li },
                None => NK::GoalVia { site },
            };
            // the arriving triangle node: old t entered through edge k -> X_k entered through 0
            if i > 0
                && let NK::Tri {
                    t: tp,
                    li: lp,
                    entry,
                    slot,
                } = chain[i - 1].kind
                && entry < 3
                && tp == per_layer[lp as usize].0
            {
                chain[i - 1].kind = NK::Tri {
                    t: per_layer[lp as usize].1[entry as usize],
                    li: lp,
                    entry: 0,
                    slot,
                };
            }
        }
        // every consecutive pair of triangle nodes must still be adjacent
        for w in chain.windows(2) {
            if let (
                NK::Tri { t, li, .. },
                NK::Tri {
                    t: t2, li: l2, entry, ..
                },
            ) = (w[0].kind, w[1].kind)
                && li == l2
                && entry < 3
            {
                let m = self.m(li as usize);
                if !m.tri_edges[t as usize].contains(&m.tri_edges[t2 as usize][entry as usize]) {
                    return false;
                }
            }
            if let NK::Tri { t, li, .. } = w[0].kind
                && split_tris.contains(&(li as usize, t))
                && !matches!(w[1].kind, NK::Vert { .. } | NK::GoalVia { .. })
            {
                return false;
            }
            // a via and the triangles the chain arrives at it from and leaves it through must
            // still meet: another via of the chain may have split one of them
            if let (NK::Tri { t, li, .. }, NK::Vert { site, .. } | NK::GoalVia { site }) = (w[0].kind, w[1].kind) {
                let m = self.m(li as usize);
                if m.site_vert.get(&site).is_some_and(|v| !m.tris[t as usize].contains(v)) {
                    return false;
                }
            }
            if let (NK::Vert { site, li }, NK::Tri { t, li: l2, entry, .. }) = (w[0].kind, w[1].kind)
                && li == l2
                && entry < 3
            {
                let m = self.m(li as usize);
                let from = m.other_tri(m.tri_edges[t as usize][entry as usize], t);
                if m.site_vert
                    .get(&site)
                    .is_some_and(|v| from == NONE || !m.tris[from as usize].contains(v))
                {
                    return false;
                }
            }
        }
        let conn = self.conns[ci].clone();
        let cls = self.board.class_of(net);
        let (width, clearance) = (cls.track_width, cls.clearance.max(self.board.rules.min_clearance));
        let mut seg_ids = Vec::new();
        let mut via_ids = Vec::new();
        let (necks, neck_w) = self.necks(ci);

        // Split the chain into runs of triangle nodes on one layer.
        let mut i = 0;
        // the search may have started at another pin of the group
        let mut start = match chain.first().map(|n| n.kind) {
            Some(NK::Tri { t, li, .. }) if self.has_kept => match self.tri_pin(li as usize, t) {
                Some(p) => End::Pin(p),
                None => End::Point(chain[0].pos),
            },
            _ => End::Pin(conn.from),
        };
        while i < chain.len() {
            let NK::Tri { li, .. } = chain[i].kind else {
                if let NK::Vert { site, .. } = chain[i].kind {
                    start = End::Via(site);
                }
                i += 1;
                continue;
            };
            let mut j = i;
            let mut run: Vec<(u32, u8, u16)> = Vec::new();
            while j < chain.len() {
                match chain[j].kind {
                    NK::Tri { t, li: l, entry, slot } if l == li => run.push((t, entry, slot)),
                    _ => break,
                }
                j += 1;
            }
            let last_t = run.last().unwrap().0;
            let end = match chain.get(j).map(|n| n.kind) {
                Some(NK::Vert { site, .. }) | Some(NK::GoalVia { site }) => {
                    self.add_via(site, net);
                    via_ids.push(site);
                    End::Via(site)
                }
                Some(NK::GoalPin { pin }) => End::Pin(pin),
                Some(NK::GoalTap { seg, e }) => End::Tap(seg, e),
                // through hole layer change: the run ends at that pin
                // or at a kept via of the net
                _ => match self.tri_pin(li as usize, last_t) {
                    Some(p) => End::Pin(p),
                    None if self.has_kept => End::Point(chain.get(j).map_or(chain[j - 1].pos, |n| n.pos)),
                    None => End::Pin(conn.from),
                },
            };
            let seg = Seg {
                net,
                conn: ci as u32,
                li,
                width,
                clearance,
                start,
                end,
                crossings: Vec::new(),
                widths: Vec::new(),
                chord_tris: Vec::new(),
                necks: necks.clone(),
                neck_w,
                imported: None,
            };
            seg_ids.push(self.insert_seg(seg, &run));
            start = end;
            i = j;
        }
        self.conn_segs[ci] = seg_ids;
        self.conn_vias[ci] = via_ids;
        if self.partner.contains_key(&net) {
            let g = self.guides.entry(net).or_default();
            g.retain(|x| x.conn != ci);
            g.push(Guide::of(ci, &chain));
        }
        if vip_learn()
            && self.conns[ci].goal != Goal::Plane
            && self.conn_vias[ci]
                .iter()
                .any(|&s| self.sub.shared.sites[s as usize].pad.is_some())
        {
            learn_vip(net, &self.board.nets[net as usize].name);
        }
        true
    }

    /// Insert a segment: its chords (against the lists as the search saw them), then its
    /// crossings at their slots.
    fn insert_seg(&mut self, mut s: Seg, run: &[(u32, u8, u16)]) -> u32 {
        let id = self.segs.len() as u32;
        let li = s.li as usize;
        let m = &self.sub.m[li];
        let vert = |e: End| match e {
            End::Via(site) => m.site_vert.get(&site).copied(),
            End::Pin(_) | End::Point(_) | End::Tap(..) => None,
        };
        let vpos = |t: u32, v: u32| CPos::Vertex(m.tris[t as usize].iter().position(|&x| x == v).unwrap_or(0) as u8);
        let mut chords: Vec<(u32, Chord)> = Vec::new();
        // leaving a via vertex: the chord in the triangle before the first crossing
        if let (Some(v), Some(&(t0, e0, _))) = (vert(s.start), run.first())
            && e0 < 3
        {
            let e = m.tri_edges[t0 as usize][e0 as usize];
            let from = m.other_tri(e, t0);
            if from != NONE && m.status[from as usize] == NONE && m.tris[from as usize].contains(&v) {
                chords.push((
                    from,
                    Chord {
                        seg: id,
                        a: vpos(from, v),
                        b: CPos::Edge(m.local_edge(from, e) as u8),
                    },
                ));
            }
        }
        for (idx, &(t, entry, _)) in run.iter().enumerate() {
            if m.status[t as usize] != NONE {
                continue;
            }
            let a = if entry < 3 {
                CPos::Edge(entry)
            } else if let Some(v) = vert(s.start) {
                vpos(t, v)
            } else {
                continue;
            };
            let b = if let Some(&(t2, e2, _)) = run.get(idx + 1) {
                if e2 >= 3 {
                    continue;
                }
                CPos::Edge(m.local_edge(t, m.tri_edges[t2 as usize][e2 as usize]) as u8)
            } else if let Some(v) = vert(s.end) {
                vpos(t, v)
            } else {
                continue;
            };
            chords.push((t, Chord { seg: id, a, b }));
        }
        let mut crossings = Vec::new();
        for &(t, entry, slot) in run {
            if entry >= 3 {
                continue;
            }
            let e = m.tri_edges[t as usize][entry as usize];
            let from = m.other_tri(e, t);
            crossings.push((e, from, slot));
        }
        for (e, from, slot) in crossings {
            let w = self.width_at_edge(li, e, s.width, &s.necks, s.neck_w);
            s.widths.push(w);
            let list = &mut self.cross[li][e as usize];
            let j = (slot as usize).min(list.len());
            list.insert(j, id);
            s.crossings.push(Crossing { e, from });
        }
        for (t, ch) in chords {
            s.chord_tris.push(t);
            self.chords[li][t as usize].push(ch);
        }
        self.segs.push(Some(s));
        id
    }

    /// Take a connection out. What ends on one of its tracks loses its hold, so it goes as well;
    /// such connections are in `torn` for the caller to lay again.
    pub fn rip_up(&mut self, ci: usize) {
        let gone = self.rip_one(ci);
        if !self.cfg.taps {
            return;
        }
        let mut gone = gone;
        while let Some(sid) = gone.pop() {
            let ends_on = |cj: usize| {
                let on = |s: &u32| matches!(self.segs[*s as usize].as_ref().map(|x| x.end), Some(End::Tap(seg, _)) if seg == sid);
                self.conn_segs[cj].iter().any(on)
            };
            let hanging: Vec<usize> = (0..self.conns.len()).filter(|&cj| ends_on(cj)).collect();
            for cj in hanging {
                self.torn.push(cj);
                gone.extend(self.rip_one(cj));
            }
        }
    }

    fn rip_one(&mut self, ci: usize) -> Vec<u32> {
        let net = self.conns[ci].net;
        if let Some(g) = self.guides.get_mut(&net) {
            g.retain(|x| x.conn != ci);
        }
        let gone = self.conn_segs[ci].clone();
        for sid in std::mem::take(&mut self.conn_segs[ci]) {
            let Some(s) = self.segs[sid as usize].take() else {
                continue;
            };
            let li = s.li as usize;
            for c in &s.crossings {
                self.cross[li][c.e as usize].retain(|&x| x != sid);
            }
            for t in &s.chord_tris {
                self.chords[li][*t as usize].retain(|ch| ch.seg != sid);
            }
        }
        for v in std::mem::take(&mut self.conn_vias[ci]) {
            self.remove_via(v);
        }
        gone
    }

    // ------------------------------------------------------------------ driver

    /// Build the connection list: plane fan-outs and MST connections, in routing order.
    pub fn plan(&mut self) {
        let b = self.board;
        let mut conns: Vec<(i64, f64, Conn)> = Vec::new();
        // hints from the local patterns of the board
        let early = i64::MIN / 2 - (1 << 20);
        let mut leader: HashMap<usize, (usize, bool)> = HashMap::new();
        let mut decap_pair: HashSet<(usize, usize)> = HashSet::new();
        if let Some(pat) = &self.cfg.patterns {
            if self.cfg.share_fanouts {
                let only = std::env::var("GB_SHARE_KIND").ok();
                for s in pat
                    .shares
                    .iter()
                    .filter(|s| only.as_deref().is_none_or(|k| k == s.kind))
                {
                    for &o in &s.others {
                        leader.insert(o, (s.leader, s.kind == "decap" && self.cfg.decaps_first));
                    }
                }
            }
            if self.cfg.decaps_first {
                for d in &pat.decaps {
                    decap_pair.insert((d.supply_pin.min(d.ic_pin), d.supply_pin.max(d.ic_pin)));
                }
            }
        }
        for net in b.routable_nets() {
            let n = &b.nets[net as usize];
            let has_plane = b.planes.iter().any(|p| p.net == net);
            let prio = n.priority as i64;
            // one representative per group of overlapping pads
            let mut reps: Vec<usize> = Vec::new();
            for &p in &n.pins {
                if !reps.iter().any(|&r| self.pin_group[r] == self.pin_group[p]) {
                    reps.push(p);
                }
            }
            let reaches_plane = |p: usize| {
                self.plane_groups.contains(&self.pin_group[p])
                    || n.pins.iter().any(|&q| {
                        self.pin_group[q] == self.pin_group[p]
                            && b.planes
                                .iter()
                                .any(|pl| pl.net == net && b.pins[q].layers.has(pl.layer) && pl.covers(b.pins[q].pos))
                    })
            };
            if has_plane {
                for &p in &reps {
                    if reaches_plane(p) {
                        continue; // a through hole pad of the group reaches the plane itself
                    }
                    let mut key = if self.cfg.fanout_first {
                        i64::MIN / 2 + prio
                    } else {
                        prio
                    };
                    // a pin that shares the fan-out of a neighbour connects to it instead
                    let (goal, d) = match leader.get(&p) {
                        Some(&(l, decap)) if self.pin_group[l] != self.pin_group[p] => {
                            if decap {
                                key = early + prio;
                            }
                            (Goal::Pins(vec![l]), b.pins[l].pos.dist(b.pins[p].pos))
                        }
                        _ => (Goal::Plane, 0.0),
                    };
                    conns.push((key, d, Conn { net, from: p, goal }));
                }
                continue;
            }
            // Prim order: each new pin connects to the pins already in the tree. Pins too narrow
            // for the minimum width of the class join after all others, so that each ends a
            // branch and no current between two other pins passes its neck.
            let pins = &reps;
            if pins.len() < 2 {
                continue;
            }
            let narrow: Vec<bool> = pins.iter().map(|&p| b.narrow_pin(p)).collect();
            let first = narrow.iter().position(|&n| !n).unwrap_or(0);
            let mut in_tree = vec![false; pins.len()];
            in_tree[first] = true;
            let mut dist: Vec<f64> = pins
                .iter()
                .map(|&p| b.pins[p].pos.dist(b.pins[pins[first]].pos))
                .collect();
            for _ in 1..pins.len() {
                let wide_left = (0..pins.len()).any(|k| !in_tree[k] && !narrow[k]);
                let (k, d) = (0..pins.len())
                    .filter(|&k| !in_tree[k] && !(wide_left && narrow[k]))
                    .map(|k| (k, dist[k]))
                    .min_by(|a, b| a.1.total_cmp(&b.1))
                    .unwrap();
                let tree: Vec<usize> = (0..pins.len()).filter(|&j| in_tree[j]).map(|j| pins[j]).collect();
                // the capacitor at an IC pin is laid first, while the room at the pin is free
                let decap = tree
                    .iter()
                    .any(|&t| decap_pair.contains(&(pins[k].min(t), pins[k].max(t))));
                conns.push((
                    if decap { early + prio } else { prio },
                    d,
                    Conn {
                        net,
                        from: pins[k],
                        goal: Goal::Pins(tree),
                    },
                ));
                in_tree[k] = true;
                for j in 0..pins.len() {
                    dist[j] = dist[j].min(b.pins[pins[j]].pos.dist(b.pins[pins[k]].pos));
                }
            }
        }
        // stable order: priority, then short connections first; the seed perturbs lengths
        let seed = self.cfg.seed;
        let order = self.order_keys(&conns);
        let mut idx: Vec<usize> = (0..conns.len()).collect();
        idx.sort_by(|&a, &b| {
            let (ka, kb) = (&order[a], &order[b]);
            conns[a]
                .0
                .cmp(&conns[b].0)
                .then(ka.tier.cmp(&kb.tier))
                .then(ka.block.total_cmp(&kb.block))
                .then(ka.group.cmp(&kb.group))
                .then(ka.sweep.total_cmp(&kb.sweep))
                .then_with(|| {
                    let ja = jitter(seed, conns[a].2.from as u64) * conns[a].1;
                    let jb = jitter(seed, conns[b].2.from as u64) * conns[b].1;
                    ja.total_cmp(&jb)
                })
                .then(a.cmp(&b))
        });
        let mut slots: Vec<Option<Conn>> = conns.into_iter().map(|c| Some(c.2)).collect();
        self.conns = idx.iter().map(|&i| slots[i].take().unwrap()).collect();
        self.pair_up();
        self.conn_segs = vec![Vec::new(); self.conns.len()];
        self.conn_vias = vec![Vec::new(); self.conns.len()];
    }

    /// Sort keys of the planned connections (priority key, length, connection) below the
    /// priority: the tier (pairs, groups, the rest), then the block a connection belongs to with
    /// its place in it. Without `pairs_first` and `groups` every connection is its own block at
    /// its jittered length, which is the plain order.
    fn order_keys(&self, conns: &[(i64, f64, Conn)]) -> Vec<OrderKey> {
        let b = self.board;
        let seed = self.cfg.seed;
        let plain = |i: usize| {
            let (_, d, c) = &conns[i];
            OrderKey {
                tier: 2,
                block: jitter(seed, c.from as u64) * d,
                group: usize::MAX,
                sweep: 0.0,
            }
        };
        let mut keys: Vec<OrderKey> = (0..conns.len()).map(plain).collect();
        let signal = |c: &Conn| matches!(c.goal, Goal::Pins(_)) && !b.planes.iter().any(|p| p.net == c.net);
        // differential pairs: as two nets or as the one wide net of `--pairs`
        let mut pair_nets: HashSet<NetId> = HashSet::new();
        if self.cfg.pairs_first {
            for (p, n) in b.diff_pairs() {
                pair_nets.insert(p);
                pair_nets.insert(n);
            }
            for (i, n) in b.nets.iter().enumerate() {
                if b.classes[n.class as usize].name.starts_with("pair:") {
                    pair_nets.insert(i as NetId);
                }
            }
            for (i, (_, d, c)) in conns.iter().enumerate() {
                if signal(c) && pair_nets.contains(&c.net) {
                    // the longest first
                    keys[i].tier = 0;
                    keys[i].block = -jitter(seed, c.from as u64) * d;
                }
            }
        }
        if self.cfg.groups == GroupOrder::Off {
            return keys;
        }
        // group ends: parts with at least 4 pins on nets (ICs, connectors); passives and
        // SOT-23 do not end a group, so a pull-up on the way does not split it
        let mut count: HashMap<&str, usize> = HashMap::new();
        for p in &b.pins {
            *count.entry(p.footprint.as_str()).or_insert(0) += 1;
        }
        let end = |p: usize| count.get(b.pins[p].footprint.as_str()).copied().unwrap_or(0) >= 4;
        let mut by_pair: HashMap<(&str, &str), Vec<(usize, usize)>> = HashMap::new();
        for (i, (_, _, c)) in conns.iter().enumerate() {
            if !signal(c) || keys[i].tier == 0 || !end(c.from) {
                continue;
            }
            let Goal::Pins(tree) = &c.goal else { continue };
            let from = b.pins[c.from].pos;
            // the nearest pin of the tree on another part that ends groups
            let Some(&to) = tree
                .iter()
                .filter(|&&t| end(t) && b.pins[t].footprint != b.pins[c.from].footprint)
                .min_by(|&&x, &&y| b.pins[x].pos.dist(from).total_cmp(&b.pins[y].pos.dist(from)))
            else {
                continue;
            };
            let (fa, fb) = (b.pins[c.from].footprint.as_str(), b.pins[to].footprint.as_str());
            let (k, a_pin) = if fa < fb { ((fa, fb), c.from) } else { ((fb, fa), to) };
            by_pair.entry(k).or_default().push((i, a_pin));
        }
        let mut groups: Vec<((&str, &str), Vec<(usize, usize)>)> =
            by_pair.into_iter().filter(|(_, m)| m.len() >= 2).collect();
        groups.sort_by(|a, b| a.0.cmp(&b.0));
        for (gi, (_, members)) in groups.iter().enumerate() {
            // across the block: the pins of the first part along the normal of the line between
            // the two parts, from one side to the other; the seed picks the side
            let centre = |which: usize| {
                let pts: Vec<Point> = members
                    .iter()
                    .map(|&(i, a)| {
                        if which == 0 {
                            b.pins[a].pos
                        } else {
                            let c = &conns[i].2;
                            if a == c.from {
                                // the end on the other part: the tree pin nearest to it
                                match &c.goal {
                                    Goal::Pins(t) => t
                                        .iter()
                                        .map(|&q| b.pins[q].pos)
                                        .min_by(|x, y| x.dist(b.pins[a].pos).total_cmp(&y.dist(b.pins[a].pos)))
                                        .unwrap_or(b.pins[a].pos),
                                    _ => b.pins[a].pos,
                                }
                            } else {
                                b.pins[c.from].pos
                            }
                        }
                    })
                    .collect();
                let n = pts.len() as f64;
                pts.iter().fold(Point::default(), |s, &p| s + p) * (1.0 / n)
            };
            let (ca, cb) = (centre(0), centre(1));
            let axis = cb - ca;
            let len = axis.dist(Point::default()).max(1e-9);
            let normal = Point {
                x: -axis.y / len,
                y: axis.x / len,
            };
            let side = if (seed / 2) % 2 == 0 { 1.0 } else { -1.0 };
            let lens: Vec<f64> = members
                .iter()
                .map(|&(i, _)| jitter(seed, conns[i].2.from as u64) * conns[i].1)
                .collect();
            let block = match self.cfg.groups {
                GroupOrder::Long => -lens.iter().copied().fold(0.0, f64::max),
                GroupOrder::Inline => lens.iter().copied().fold(f64::MAX, f64::min),
                _ => lens.iter().copied().fold(f64::MAX, f64::min),
            };
            for &(i, a) in members {
                let k = &mut keys[i];
                k.tier = if self.cfg.groups == GroupOrder::Inline { 2 } else { 1 };
                k.block = block;
                k.group = gi;
                k.sweep = side * (b.pins[a].pos - ca).dot(normal);
            }
        }
        keys
    }

    /// Reorder the planned connections by a key (lower first; ties keep the plan order).
    pub fn reorder(&mut self, key: impl Fn(&Conn) -> f64) {
        let mut idx: Vec<usize> = (0..self.conns.len()).collect();
        idx.sort_by(|&a, &b| key(&self.conns[a]).total_cmp(&key(&self.conns[b])).then(a.cmp(&b)));
        self.conns = idx.iter().map(|&i| self.conns[i].clone()).collect();
        self.pair_up();
    }

    /// The nets of a differential pair go one after the other, so that nothing gets between.
    fn pair_up(&mut self) {
        if self.partner.is_empty() {
            return;
        }
        let mut out: Vec<Conn> = Vec::with_capacity(self.conns.len());
        let mut left: Vec<Option<Conn>> = self.conns.drain(..).map(Some).collect();
        for i in 0..left.len() {
            let Some(c) = left[i].take() else { continue };
            let partner = self.partner.get(&c.net).copied();
            out.push(c);
            if let Some(p) = partner {
                for o in left.iter_mut().skip(i + 1) {
                    if o.as_ref().is_some_and(|x| x.net == p) {
                        out.push(o.take().unwrap());
                        break;
                    }
                }
            }
        }
        self.conns = out;
    }

    /// Route all planned connections. Returns the number routed.
    fn conn_pins(&self, ci: usize) -> Vec<usize> {
        let c = &self.conns[ci];
        let mut p = vec![c.from];
        if let Goal::Pins(t) = &c.goal {
            p.extend(t.iter().copied());
        }
        p
    }

    pub fn run(&mut self) -> usize {
        let mut failed = Vec::new();
        for ci in 0..self.conns.len() {
            for p in self.conn_pins(ci) {
                self.pending[p] += 1;
            }
        }
        if self.cfg.negotiate > 0 && !self.soft_on {
            self.negotiate();
            self.pending.fill(0);
            failed = self.unrouted();
        } else {
            for ci in 0..self.conns.len() {
                let ok = self.route_one(ci);
                for p in self.conn_pins(ci) {
                    self.pending[p] -= 1;
                }
                if !ok {
                    failed.push(ci);
                }
            }
        }
        // A pin that was to share the fan-out of a neighbour and found no way to it fans out on
        // its own.
        for &ci in &failed {
            let c = &self.conns[ci];
            if matches!(c.goal, Goal::Pins(_)) && self.board.planes.iter().any(|p| p.net == c.net) {
                self.conns[ci].goal = Goal::Plane;
            }
        }
        // Fan-outs that found no plane: try a via in the pad, then connect to the nearest pin of
        // the net instead.
        // the second chance runs without hot spots: they must never cost a connection
        self.cfg.hotspots = Default::default();
        let vip = self.cfg.via_in_pad;
        self.cfg.via_in_pad = true;
        failed.retain(|&ci| !self.route_one(ci));
        self.cfg.via_in_pad = vip;
        for &ci in &failed {
            if self.conns[ci].goal == Goal::Plane {
                let c = &self.conns[ci];
                let pins: Vec<usize> = self.board.nets[c.net as usize]
                    .pins
                    .iter()
                    .copied()
                    .filter(|&p| p != c.from)
                    .collect();
                self.conns[ci].goal = Goal::Pins(pins);
            }
        }
        // Rip-up and reroute: route the failed connection first, then the ripped ones.
        for _pass in 0..self.cfg.ripup_passes {
            failed.retain(|&ci| self.conn_segs[ci].is_empty() && !self.route_one(ci));
            if failed.is_empty() {
                break;
            }
            let mut still = Vec::new();
            for &ci in &failed {
                let victims = self.blockers(ci);
                for &v in &victims {
                    self.rip_up(v);
                }
                let ok = self.route_one(ci);
                for &v in &victims {
                    if !self.route_one(v) {
                        still.push(v);
                    }
                }
                if !ok {
                    still.push(ci);
                }
            }
            still.sort_unstable();
            still.dedup();
            failed = still;
        }
        self.reduce_vias();
        self.conn_segs.iter().filter(|s| !s.is_empty()).count()
    }

    /// PathFinder on the topology. In a first pass every connection takes its way and may
    /// overfill edges and cross tracks of other nets, for a price. Then, round after round, the
    /// connections in a conflict are routed again, one at a time and seeing all the others: the
    /// price of a conflict rises from round to round, and where conflicts stay the history
    /// makes the edges dear for good, so that the connection with the fewest alternatives keeps
    /// the place. What is still in a conflict after the last round is ripped up, which leaves a
    /// legal topology; the rip-up passes of [`Router::run`] take over from there, with the
    /// history as a guide.
    pub fn negotiate(&mut self) {
        let env = |k: &str, d: f64| std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d);
        let log = std::env::var("GB_NEG_LOG").is_ok();
        let growth = env("GB_NEG_GROWTH", 1.5);
        let history = env("GB_NEG_HISTORY", 1.0);
        self.prices = Some(Prices {
            soft: true,
            overflow: env("GB_NEG_OVERFLOW", 2.0),
            crossing: env("GB_NEG_CROSSING", 1.5),
            history: self.cross.iter().map(|l| vec![0.0; l.len()]).collect(),
            tight: env("GB_NEG_TIGHT", 3.0),
            reserve: env("GB_NEG_RESERVE", 0.05),
            solid: false,
        });
        for ci in 0..self.conns.len() {
            self.route_one(ci);
        }
        let t0 = std::time::Instant::now();
        let (mut least, mut stale) = (usize::MAX, 0);
        for round in 1..=self.cfg.negotiate {
            let (conflicts, edges, crossings) = self.conflicts(history);
            let mut todo: Vec<usize> = (0..self.conns.len())
                .filter(|&ci| conflicts[ci] > 0 || self.conn_segs[ci].is_empty())
                .collect();
            if log {
                eprintln!(
                    "negotiation seed {} round {round}: {} connections to route, {edges} edges over, {crossings} crossings, {} unrouted, {:.1} s",
                    self.cfg.seed,
                    todo.len(),
                    self.unrouted().len(),
                    t0.elapsed().as_secs_f64()
                );
            }
            // conflicts that no price settles: a connection without any legal way, or a board
            // with more connections than room on its layers
            let total = edges + crossings;
            stale = if (total as f64) < least as f64 * 0.97 {
                0
            } else {
                stale + 1
            };
            least = least.min(total);
            if todo.is_empty() || round == self.cfg.negotiate || stale == MAX_STALE {
                break;
            }
            if let Some(p) = &mut self.prices {
                p.overflow *= growth;
                p.crossing *= growth;
            }
            todo.sort_by_key(|&ci| std::cmp::Reverse(conflicts[ci]));
            for ci in todo {
                if self.conn_segs[ci].is_empty() || self.conn_in_conflict(ci) {
                    self.rip_up(ci);
                    self.route_one(ci);
                }
            }
        }
        // the worst offenders go first, which may settle the conflicts of others
        let (conflicts, _, _) = self.conflicts(0.0);
        let mut order: Vec<usize> = (0..self.conns.len()).filter(|&ci| conflicts[ci] > 0).collect();
        order.sort_by_key(|&ci| std::cmp::Reverse(conflicts[ci]));
        if let Some(p) = &mut self.prices {
            p.soft = false;
        }
        for ci in order {
            if self.conn_in_conflict(ci) {
                self.rip_up(ci);
            }
        }
    }

    /// Conflicts of the current state, per connection: the edges over capacity it crosses and
    /// the tracks of other nets it crosses. Adds to the history of the edges in a conflict and
    /// returns the conflicts per connection, the edges over capacity and the crossings.
    fn conflicts(&mut self, history: f64) -> (Vec<u32>, usize, usize) {
        let mut per_conn = vec![0; self.conns.len()];
        let (mut edges, mut crossings) = (0, 0);
        let mut hist = self.prices.as_mut().map(|p| std::mem::take(&mut p.history));
        for li in 0..self.nl() {
            for e in 0..self.cross[li].len() {
                if self.cross[li][e].is_empty() {
                    continue;
                }
                let over = self.edge_overflow(li, e as u32);
                if over <= OVER_TOL {
                    continue;
                }
                edges += 1;
                for &s in &self.cross[li][e] {
                    per_conn[self.segs[s as usize].as_ref().unwrap().conn as usize] += 1;
                }
                if let Some(h) = &mut hist {
                    h[li].resize(self.cross[li].len(), 0.0);
                    h[li][e] += history * over.max(0.1);
                }
            }
            for t in 0..self.chords[li].len() {
                for (a, b) in self.tri_crossings(li, t as u32) {
                    crossings += 1;
                    per_conn[a] += 1;
                    per_conn[b] += 1;
                    if let Some(h) = &mut hist {
                        h[li].resize(self.cross[li].len(), 0.0);
                        for &e in &self.m(li).tri_edges[t] {
                            h[li][e as usize] += history * 0.1;
                        }
                    }
                }
            }
        }
        if let (Some(p), Some(h)) = (self.prices.as_mut(), hist) {
            p.history = h;
        }
        (per_conn, edges, crossings)
    }

    /// Pairs of connections with chords of different nets that cross in triangle t.
    fn tri_crossings(&self, li: usize, t: u32) -> Vec<(usize, usize)> {
        let list = &self.chords[li][t as usize];
        let mut out = Vec::new();
        if list.len() < 2 {
            return out;
        }
        let seg = |ch: &Chord| self.segs[ch.seg as usize].as_ref().unwrap();
        let at = |ch: &Chord, p: CPos| self.coord(t, li, p, Some(ch.seg), None);
        for (i, a) in list.iter().enumerate() {
            let (x, y) = (at(a, a.a), at(a, a.b));
            for b in &list[i + 1..] {
                let (sa, sb) = (seg(a), seg(b));
                if sa.net != sb.net && Self::chord_hits(&[(at(b, b.a), at(b, b.b))], x, y) > 0 {
                    out.push((sa.conn as usize, sb.conn as usize));
                }
            }
        }
        out
    }

    /// Is connection ci on an edge over capacity, or does it cross a track of another net?
    fn conn_in_conflict(&self, ci: usize) -> bool {
        self.conn_segs[ci].iter().any(|&sid| {
            let s = self.segs[sid as usize].as_ref().unwrap();
            let li = s.li as usize;
            s.crossings.iter().any(|c| self.edge_overflow(li, c.e) > OVER_TOL)
                || s.chord_tris
                    .iter()
                    .any(|&t| self.tri_crossings(li, t).iter().any(|&(a, b)| a == ci || b == ci))
        })
    }

    /// Connections with vias are routed again at a high via price. A connection keeps the
    /// result if it has fewer vias; otherwise it is routed again at the normal price.
    fn reduce_vias(&mut self) {
        let price = self.cfg.via_pass;
        if price <= 0.0 {
            return;
        }
        let normal = self.cfg.via_cost;
        for ci in 0..self.conns.len() {
            let before = self.conn_vias[ci].len();
            if before == 0 || self.conn_segs[ci].is_empty() {
                continue;
            }
            self.rip_up(ci);
            self.cfg.via_cost = normal * price;
            let ok = self.route_one(ci);
            self.cfg.via_cost = normal;
            if !ok || self.conn_vias[ci].len() >= before {
                if ok {
                    self.rip_up(ci);
                }
                self.route_one(ci);
            }
        }
    }

    /// Experiment `GB_VIP_EARLY=empty`: the nets of the connections that find no way on the
    /// empty board without a via in a pad, and one with it. Such a pad lies in a pocket on its
    /// layer that fixed copper closes, not the tracks of a run, so no routing order gets it out.
    /// Plane fan-outs are left out. Call it after [`Router::plan`], before anything is routed.
    pub fn pocket_nets(&mut self) -> Vec<NetId> {
        let vip = self.cfg.via_in_pad;
        let mut buf = std::mem::take(&mut self.search_buf);
        let mut nets: Vec<NetId> = Vec::new();
        for ci in 0..self.conns.len() {
            let net = self.conns[ci].net;
            if self.conns[ci].goal == Goal::Plane || nets.contains(&net) {
                continue;
            }
            self.cfg.via_in_pad = false;
            if self.search(ci, &mut buf).is_some() {
                continue;
            }
            self.cfg.via_in_pad = true;
            if self.search(ci, &mut buf).is_some() {
                nets.push(net);
            }
        }
        self.cfg.via_in_pad = vip;
        self.search_buf = buf;
        nets.sort_unstable();
        nets
    }

    pub fn route_one(&mut self, ci: usize) -> bool {
        for _ in 0..3 {
            let mut buf = std::mem::take(&mut self.search_buf);
            let found = self.search(ci, &mut buf);
            self.search_buf = buf;
            match found {
                Some(chain) => {
                    let validate = std::env::var("GB_VALIDATE_EACH").is_ok();
                    let before = if validate { self.validate().len() } else { 0 };
                    if self.commit(ci, &chain) {
                        if validate {
                            let v = self.validate();
                            if v.len() > before {
                                let kinds: Vec<String> = chain.iter().map(|n| format!("{:?}", n.kind)).collect();
                                eprintln!(
                                    "conn {ci} broke the state: {:?}\n  chain {}",
                                    &v[v.len().min(before)..],
                                    kinds.join(" ")
                                );
                            }
                        }
                        if self.detour_bridge(ci) {
                            // a pin that was to share the fan-out of a neighbour took vias or a
                            // long way to it: it fans out on its own instead
                            self.rip_up(ci);
                            if let Goal::Pins(t) = std::mem::replace(&mut self.conns[ci].goal, Goal::Plane) {
                                for p in t {
                                    self.pending[p] = self.pending[p].saturating_sub(1);
                                }
                            }
                            continue;
                        }
                        return true;
                    }
                }
                None => return false,
            }
        }
        false
    }

    /// True if connection ci joins a pin of a plane net to a neighbour that has the fan-out
    /// ([`RouterConfig::share_fanouts`]), and its route is no short way on one layer.
    fn detour_bridge(&self, ci: usize) -> bool {
        let c = &self.conns[ci];
        let Goal::Pins(t) = &c.goal else { return false };
        if !self.cfg.share_fanouts || !self.board.planes.iter().any(|p| p.net == c.net) {
            return false;
        }
        if !self.conn_vias[ci].is_empty() {
            return true;
        }
        let from = self.board.pins[c.from].pos;
        let straight = t
            .iter()
            .map(|&p| self.board.pins[p].pos.dist(from))
            .fold(f64::INFINITY, f64::min);
        let way: f64 = self.conn_points(ci).windows(2).map(|w| w[0].dist(w[1])).sum();
        way > 2.0 * straight + 1.0
    }

    /// Connections whose routes pass close to the straight line of a failed connection.
    pub fn blockers(&self, ci: usize) -> Vec<usize> {
        let c = &self.conns[ci];
        let a = self.board.pins[c.from].pos;
        let b = match &c.goal {
            Goal::Pins(p) => p
                .iter()
                .map(|&q| self.board.pins[q].pos)
                .min_by(|x, y| x.dist(a).total_cmp(&y.dist(a)))
                .unwrap_or(a),
            Goal::Plane => a,
        };
        let radius = 1.0 + a.dist(b) * 0.15;
        let mut out = Vec::new();
        for (cj, segs) in self.conn_segs.iter().enumerate() {
            if cj == ci || segs.is_empty() || self.conns[cj].net == c.net {
                continue;
            }
            let near = segs.iter().any(|&s| {
                let s = self.segs[s as usize].as_ref().unwrap();
                let m = self.m(s.li as usize);
                s.crossings.iter().any(|cr| {
                    let [u, v] = m.edges[cr.e as usize];
                    let mid = (m.verts[u as usize] + m.verts[v as usize]) * 0.5;
                    point_seg_dist(mid, a, b) < radius
                })
            });
            if near {
                out.push(cj);
            }
        }
        out.truncate(6);
        out
    }

    /// What really stands in the way of open connection `ci`, where [`Router::blockers`] can only
    /// guess from the straight line: it looks for its way as in a negotiation, through whatever
    /// is there at the price `crossing`, and names the connections it ran into, in the order along
    /// the way, with the points of the way. All on a copy: a via leaves its mark in the mesh even
    /// when it is gone, and a via in a pad needs its site unused. None if there is no way even so.
    pub(crate) fn way_through(&self, ci: usize, crossing: f64) -> Option<(Vec<usize>, Vec<Point>)> {
        self.way_through_vias(ci, crossing).map(|(hit, way, _)| (hit, way))
    }

    /// [`Router::way_through`], and how many vias the way takes.
    pub(crate) fn way_through_vias(&self, ci: usize, crossing: f64) -> Option<(Vec<usize>, Vec<Point>, usize)> {
        let mut r = self.clone();
        r.prices = Some(Prices {
            soft: true,
            overflow: 2.0 * crossing,
            crossing,
            history: vec![Vec::new(); self.nl()],
            tight: 0.0,
            reserve: 0.0,
            solid: true,
        });
        r.route_one(ci)
            .then(|| (r.in_the_way(ci), r.conn_points(ci), r.conn_vias[ci].len()))
    }

    /// What connection `ci`, laid through everything, runs into.
    fn in_the_way(&self, ci: usize) -> Vec<usize> {
        let net = self.conns[ci].net;
        let mut hit = Vec::new();
        for &sid in &self.conn_segs[ci] {
            let s = self.segs[sid as usize].as_ref().unwrap();
            let li = s.li as usize;
            for c in &s.crossings {
                self.on_full_edge(li, c.e, net, &mut hit);
            }
            for &t in &s.chord_tris {
                for (a, b) in self.tri_crossings(li, t) {
                    // soft copper (no connection) does not go
                    let soft = u32::MAX as usize;
                    if a == ci && b != soft && !hit.contains(&b) {
                        hit.push(b);
                    } else if b == ci && a != soft && !hit.contains(&a) {
                        hit.push(a);
                    }
                }
            }
        }
        // its vias take room from what passes them, also on the layers it does not use
        for &site in &self.conn_vias[ci] {
            for li in 0..self.nl() {
                let m = self.m(li);
                let Some(&v) = m.site_vert.get(&site) else { continue };
                for &t in &m.vert_tris[v as usize] {
                    for &e in &m.tri_edges[t as usize] {
                        self.on_full_edge(li, e, net, &mut hit);
                    }
                }
            }
        }
        hit
    }

    /// In how many pieces the routed copper holds the pins of `net` together. A connection may
    /// end on any via of its net, so one that is laid again can end on its own side of the net
    /// and leave it in two pieces, although every connection counts as routed.
    pub(crate) fn net_pieces(&self, net: NetId) -> usize {
        // a node is a group of pins, a via site or the plane of the net
        const PLANE: u64 = u64::MAX;
        let pin = |p: usize| self.pin_group[p] as u64;
        let site = |v: u32| (1 << 32) | v as u64;
        fn find(parent: &mut FastMap<u64, u64>, x: u64) -> u64 {
            let up = *parent.entry(x).or_insert(x);
            if up == x {
                return x;
            }
            let root = find(parent, up);
            parent.insert(x, root);
            root
        }
        let mut parent: FastMap<u64, u64> = FastMap::default();
        let mut join = |a: u64, b: u64| {
            let (a, b) = (find(&mut parent, a), find(&mut parent, b));
            parent.insert(a, b);
        };
        for (ci, c) in self.conns.iter().enumerate() {
            if c.net != net {
                continue;
            }
            for &sid in &self.conn_segs[ci] {
                let s = self.segs[sid as usize].as_ref().unwrap();
                let node = |e: End| match e {
                    End::Pin(p) => Some(pin(p)),
                    End::Via(v) => Some(site(v)),
                    End::Point(_) | End::Tap(..) => None,
                };
                let tapped = |e: End| match e {
                    End::Tap(sid, _) => self.segs[sid as usize].as_ref().and_then(|s| node(s.start)),
                    e => node(e),
                };
                let nodes = [tapped(s.start), tapped(s.end)];
                if let [Some(a), Some(b)] = nodes {
                    join(a, b);
                }
                // a via in a pad holds its pad
                for e in [s.start, s.end] {
                    if let End::Via(v) = e
                        && let Some(p) = self.sub.shared.sites[v as usize].pad
                    {
                        join(site(v), pin(p as usize));
                    }
                }
                // two fan-outs that end on each other reach no plane
                if c.goal == Goal::Plane && !matches!(s.end, End::Tap(..)) {
                    join(nodes[0].unwrap_or(pin(c.from)), PLANE);
                }
            }
        }
        let mut roots: Vec<u64> = self.board.nets[net as usize]
            .pins
            .iter()
            .map(|&p| find(&mut parent, pin(p)))
            .collect();
        roots.sort_unstable();
        roots.dedup();
        roots.len()
    }

    /// Add to `hit` what takes up an edge over capacity, but for `net`: the connections on it and
    /// those with a via at its ends or opposite of it.
    fn on_full_edge(&self, li: usize, e: u32, net: NetId, hit: &mut Vec<usize>) {
        if self.edge_overflow(li, e) <= OVER_TOL {
            return;
        }
        let mut add = |cj: usize| {
            if !hit.contains(&cj) {
                hit.push(cj);
            }
        };
        let others = self.cross[li][e as usize]
            .iter()
            .map(|&o| self.segs[o as usize].as_ref().unwrap());
        // soft copper is no connection that could be laid again
        others
            .filter(|o| o.net != net && o.imported.is_none())
            .for_each(|o| add(o.conn as usize));
        let m = self.m(li);
        for &t in m.edge_tris[e as usize].iter().filter(|&&t| t != NONE) {
            for &v in &m.tris[t as usize] {
                let site = m.vert_site[v as usize];
                if self.vias.get(&site).is_some_and(|via| via.net != net) {
                    (0..self.conns.len())
                        .filter(|&cj| self.conn_vias[cj].contains(&site))
                        .for_each(&mut add);
                }
            }
        }
    }

    /// The pieces of `net` that no open connection accounts for: 1 when its copper holds together
    /// as far as it is routed. An open connection is a piece more, but no split.
    pub(crate) fn net_splits(&self, net: NetId) -> usize {
        let open = (0..self.conns.len()).filter(|&ci| self.conns[ci].net == net && self.conn_segs[ci].is_empty());
        self.net_pieces(net).saturating_sub(open.count())
    }

    /// The segments connection `ci` may end on: of its net, and not on its side of the gap it
    /// closes. That goes for plane fan-outs too: a track of the net leads to a via into the plane.
    /// Pairs stay out, their tracks must run together to the end.
    fn tappable(&self, ci: usize) -> HashSet<u32> {
        let c = &self.conns[ci];
        if !self.cfg.taps || self.partner.contains_key(&c.net) {
            return HashSet::new();
        }
        let mine = self.net_side(c.net, c.from);
        (0..self.conns.len())
            .filter(|&cj| cj != ci && self.conns[cj].net == c.net)
            .flat_map(|cj| self.conn_segs[cj].iter().copied())
            .filter(|&sid| {
                let s = self.segs[sid as usize].as_ref().unwrap();
                !mine.contains(&s.start) && !mine.contains(&s.end)
            })
            .collect()
    }

    /// The ends of the routed segments of `net` that hang together with pin `from`.
    fn net_side(&self, net: NetId, from: usize) -> Vec<End> {
        let segs: Vec<&Seg> = (0..self.conns.len())
            .filter(|&cj| self.conns[cj].net == net)
            .flat_map(|cj| self.conn_segs[cj].iter())
            .filter_map(|&sid| self.segs[sid as usize].as_ref())
            .collect();
        let group = self.pin_group[from];
        let mut side: Vec<End> = vec![End::Pin(from)];
        loop {
            let holds = |side: &[End], e: End| {
                side.iter().any(|&x| match (x, e) {
                    (End::Pin(a), End::Pin(b)) => self.pin_group[a] == self.pin_group[b],
                    (End::Via(a), End::Via(b)) => a == b,
                    _ => false,
                }) || matches!(e, End::Pin(p) if self.pin_group[p] == group)
            };
            let before = side.len();
            for s in &segs {
                let end = match s.end {
                    // what ends on a segment hangs together with that segment
                    End::Tap(sid, _) => self.segs[sid as usize].as_ref().map_or(s.end, |o| o.start),
                    e => e,
                };
                match (holds(&side, s.start), holds(&side, end)) {
                    (true, false) => side.push(end),
                    (false, true) => side.push(s.start),
                    _ => {}
                }
            }
            if side.len() == before {
                return side;
            }
        }
    }

    /// Where connection `ci` runs: its pin and the middle of every mesh edge it crosses.
    pub(crate) fn conn_points(&self, ci: usize) -> Vec<Point> {
        let mut out = vec![self.board.pins[self.conns[ci].from].pos];
        for &sid in &self.conn_segs[ci] {
            let s = self.segs[sid as usize].as_ref().unwrap();
            let m = self.m(s.li as usize);
            out.extend(s.crossings.iter().map(|cr| {
                let [u, v] = m.edges[cr.e as usize];
                (m.verts[u as usize] + m.verts[v as usize]) * 0.5
            }));
        }
        out
    }

    pub fn unrouted(&self) -> Vec<usize> {
        (0..self.conns.len())
            .filter(|&ci| self.conn_segs[ci].is_empty())
            .collect()
    }

    /// Chords of triangle t (debugging aid).
    pub fn chords_debug(&self, li: usize, t: u32) -> Vec<String> {
        self.chords[li][t as usize]
            .iter()
            .map(|c| format!("{}:{:?}->{:?}", c.seg, c.a, c.b))
            .collect()
    }

    /// Consistency checks of the topological state (debugging aid).
    pub fn validate(&self) -> Vec<String> {
        let mut out = Vec::new();
        let mut over = 0;
        for li in 0..self.nl() {
            for (e, list) in self.cross[li].iter().enumerate() {
                if list.is_empty() {
                    continue;
                }
                let items = self.items_on(li, e as u32);
                if self.pack(li, e as u32, &items, None).is_none() {
                    over += 1;
                }
                for &s in list {
                    let seg = self.segs[s as usize].as_ref();
                    if seg.is_none_or(|sg| sg.li as usize != li || !sg.crossings.iter().any(|c| c.e as usize == e)) {
                        out.push(format!("edge {e} lists seg {s} which does not cross it"));
                    }
                }
            }
        }
        if over > 0 {
            out.push(format!("{over} edges over capacity"));
        }
        for (sid, s) in self.segs.iter().enumerate() {
            let Some(s) = s else { continue };
            let m = self.m(s.li as usize);
            for w in s.crossings.windows(2) {
                if m.other_tri(w[0].e, w[0].from) != w[1].from {
                    out.push(format!(
                        "seg {sid}: crossing chain broken at edge {} -> {}",
                        w[0].e, w[1].e
                    ));
                    break;
                }
            }
            for c in &s.crossings {
                if !self.cross[s.li as usize][c.e as usize].contains(&(sid as u32)) {
                    out.push(format!("seg {sid}: missing in list of edge {}", c.e));
                }
            }
        }
        out
    }
}

/// Intersection of the lines ab and cd: (parameter on ab, parameter on cd).
fn line_params(a: Point, b: Point, c: Point, d: Point) -> (f64, f64) {
    let r = b - a;
    let s = d - c;
    let den = r.cross(s);
    if den.abs() < 1e-300 {
        return (0.5, 0.5);
    }
    ((c - a).cross(s) / den, (c - a).cross(r) / den)
}

/// Intersection of segments ab and cd: (parameter on ab, parameter on cd).
#[allow(dead_code)]
pub(crate) fn seg_param_intersection(a: Point, b: Point, c: Point, d: Point) -> Option<(f64, f64)> {
    let r = b - a;
    let s = d - c;
    let den = r.cross(s);
    if den.abs() < 1e-15 {
        return None;
    }
    let t = (c - a).cross(s) / den;
    let u = (c - a).cross(r) / den;
    if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
        Some((t, u))
    } else {
        None
    }
}

fn jitter(seed: u64, x: u64) -> f64 {
    if seed == 0 {
        return 1.0;
    }
    let mut z = seed.wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ x.wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z ^= z >> 31;
    z = z.wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^= z >> 29;
    0.5 + (z % 1000) as f64 / 1000.0
}

/// Experiment: the nets named in `GB_VIP_NETS` (comma separated) may leave or enter a pad through
/// a via in it at any time, not only as the last resort of a run. Off when the variable is unset.
fn vip_net(name: &str) -> bool {
    static NETS: std::sync::OnceLock<Vec<String>> = std::sync::OnceLock::new();
    NETS.get_or_init(|| {
        std::env::var("GB_VIP_NETS")
            .map(|v| {
                v.split(',')
                    .map(|s| s.trim().to_string())
                    .filter(|s| !s.is_empty())
                    .collect()
            })
            .unwrap_or_default()
    })
    .iter()
    .any(|n| n == name)
}

/// Experiment: with `GB_PAD_REUSE=1` a via in the source pad that was ripped up can come back.
/// Its site stays split after the rip-up and is no longer usable as a site, so without this the
/// pad has no via-in-pad candidate left (see docs/MILESTONES.md, Corne `rR4.1`).
fn pad_reuse() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var("GB_PAD_REUSE").is_ok_and(|v| v == "1"))
}

/// `GB_VIP_LEARN=1` (experiment): a net whose connection got through with a via in a pad, the
/// last resort, may take one at any time from then on, as if `GB_VIP_NETS` named it. A pad in a
/// pocket then gets its via while the board around it is still open. Plane fan-outs are left out.
/// The nets are shared by all runs of the process, so with it on the normal search depends on
/// which run learns first and is no longer the same from run to run.
fn vip_learn() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var("GB_VIP_LEARN").is_ok_and(|v| v == "1"))
}

fn learned_vips() -> &'static std::sync::Mutex<HashSet<NetId>> {
    static NETS: std::sync::OnceLock<std::sync::Mutex<HashSet<NetId>>> = std::sync::OnceLock::new();
    NETS.get_or_init(Default::default)
}

fn vip_learned(net: NetId) -> bool {
    vip_learn() && learned_vips().lock().unwrap().contains(&net)
}

fn learn_vip(net: NetId, name: &str) {
    if learned_vips().lock().unwrap().insert(net) {
        eprintln!("via in pad learned: {name}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::{LayerSet, Net, Obstacle, ObstacleKind, Pin};
    use crate::geom::{Polygon, Shape};
    use crate::topo::SubstrateConfig;

    #[test]
    fn the_pair_guide_is_the_partners_way_on_its_layer() {
        // nodes 4 mm apart on routing layer 1, as the search leaves them in open space
        let g = Guide {
            conn: 0,
            pts: vec![(pt(0.0, 0.0), 1), (pt(4.0, 0.0), 1), (pt(8.0, 0.0), 1)],
        };
        let guides = [g];
        // between two nodes the way is right there, not 2 mm off at the nodes
        assert!((guide_dist(&guides, 1, pt(2.0, 0.3), 10.0) - 0.3).abs() < 1e-9);
        // on another layer right above, it counts farther
        assert!((guide_dist(&guides, 0, pt(2.0, 0.3), 10.0) - 0.3 - OFF_LAYER).abs() < 1e-9);
        assert_eq!(guide_dist(&guides, 1, pt(2.0, 50.0), 3.0), 3.0);
    }

    /// A board of small square pads: (footprint, net, x, y) per pin, on a 20 mm square.
    fn pad_board(pins: &[(&str, &str, f64, f64)]) -> Board {
        let mut b = crate::topo::tests::two_pad_board();
        b.outline = vec![Polygon::new(vec![pt(0., 0.), pt(20., 0.), pt(20., 20.), pt(0., 20.)])];
        b.nets.clear();
        b.pins.clear();
        b.obstacles.clear();
        for (i, &(fp, net, x, y)) in pins.iter().enumerate() {
            let n = match b.nets.iter().position(|n| n.name == net) {
                Some(n) => n,
                None => {
                    b.nets.push(Net {
                        name: net.into(),
                        class: 0,
                        pins: Vec::new(),
                        planes: LayerSet::default(),
                        priority: 0,
                    });
                    b.nets.len() - 1
                }
            };
            b.nets[n].pins.push(i);
            b.obstacles.push(Obstacle {
                shape: Shape::Polygon(crate::geom::roundrect_polygon(pt(x, y), 0.4, 0.4, 0.0, 0.0, 0.005)),
                layers: LayerSet::single(0),
                net: Some(n as NetId),
                kind: ObstacleKind::Pad,
                hole: None,
                clearance: 0.2,
                pin: Some(i),
                locked: true,
            });
            b.pins.push(Pin {
                name: format!("{fp}.{i}"),
                footprint: fp.into(),
                net: n as NetId,
                pos: pt(x, y),
                layers: LayerSet::single(0),
                obstacles: vec![i],
                size: (0.4, 0.4),
            });
        }
        b
    }

    fn planned_nets(b: &Board, cfg: RouterConfig) -> Vec<String> {
        let sub = Substrate::build(b, &SubstrateConfig::default());
        let mut r = Router::new(b, &sub, cfg);
        r.plan();
        r.conns.iter().map(|c| b.nets[c.net as usize].name.clone()).collect()
    }

    #[test]
    fn pairs_and_part_groups_move_forward_in_the_plan() {
        let mut pins = Vec::new();
        // a bus of four nets between two parts, 12 mm, given in a mixed order
        for net in ["C", "A", "D", "B"] {
            let y = 4.0 + ["A", "B", "C", "D"].iter().position(|&n| n == net).unwrap() as f64;
            pins.push(("U1", net, 3.0, y));
            pins.push(("U2", net, 15.0, y));
        }
        // two short connections between passives
        pins.extend([
            ("R1", "E", 3.0, 12.0),
            ("R2", "E", 5.0, 12.0),
            ("R3", "F", 3.0, 14.0),
            ("R4", "F", 6.0, 14.0),
        ]);
        // a pair, 14 mm
        pins.extend([
            ("J1", "L_P", 3.0, 17.0),
            ("J2", "L_P", 17.0, 17.0),
            ("J1", "L_N", 3.0, 18.0),
            ("J2", "L_N", 17.0, 18.0),
        ]);
        let b = pad_board(&pins);
        let plain = planned_nets(&b, RouterConfig::default());
        assert_eq!(plain, ["E", "F", "C", "A", "D", "B", "L_P", "L_N"]);
        let groups = RouterConfig {
            groups: GroupOrder::Short,
            ..Default::default()
        };
        // the bus as one block before the rest, in pin order across it
        assert_eq!(planned_nets(&b, groups), ["A", "B", "C", "D", "E", "F", "L_P", "L_N"]);
        let inline = RouterConfig {
            groups: GroupOrder::Inline,
            ..Default::default()
        };
        assert_eq!(planned_nets(&b, inline), ["E", "F", "A", "B", "C", "D", "L_P", "L_N"]);
        let pairs = RouterConfig {
            pairs_first: true,
            ..Default::default()
        };
        assert_eq!(planned_nets(&b, pairs), ["L_P", "L_N", "E", "F", "C", "A", "D", "B"]);
    }

    /// A search whose goal is shut in ends at once, with the same result: a dense board routed
    /// with the check at the first expansion of every search, and without it.
    #[test]
    fn a_goal_shut_in_ends_the_search_with_the_same_result() {
        let mut pins = Vec::new();
        let mut x: u64 = 7;
        let mut next = || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            (x % 1000) as f64 / 1000.0
        };
        // 40 nets of two pins on 12 x 12 mm, many of them crossing
        let names: Vec<String> = (0..40).map(|i| format!("N{i}")).collect();
        for n in &names {
            for fp in ["U1", "U2"] {
                let (u, v) = (next(), next());
                pins.push((fp, n.as_str(), 4.0 + 12.0 * u, 4.0 + 12.0 * v));
            }
        }
        let b = pad_board(&pins);
        let sub = Substrate::build(&b, &SubstrateConfig::default());
        let run = |check_at: usize| {
            GOAL_CHECK_TEST.with(|c| c.set(Some(check_at)));
            GOAL_SHUT_TEST.with(|c| c.set(0));
            let mut r = Router::new(&b, &sub, RouterConfig::default());
            r.plan();
            let routed = r.run();
            let geo = format!("{:?}", r.geometry());
            GOAL_CHECK_TEST.with(|c| c.set(None));
            (routed, geo, GOAL_SHUT_TEST.with(|c| c.get()))
        };
        let (routed, geo, _) = run(0);
        let (routed_checked, geo_checked, shut) = run(1);
        assert!(routed < 40, "the board should be too full: {routed}");
        assert!(shut > 0, "no goal was found shut in");
        assert_eq!(routed, routed_checked);
        assert!(geo == geo_checked, "the geometry differs");
    }

    /// The open list hands out the nodes by f, ties by index, also with pushes between pops,
    /// negative values and equal f.
    #[test]
    fn the_open_list_pops_in_total_order() {
        let mut h = OpenHeap::default();
        let mut reference: Vec<(f64, u32)> = Vec::new();
        let mut x: u64 = 12345;
        let mut next = || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            x
        };
        let mut out = Vec::new();
        let mut want = Vec::new();
        for idx in 0..5000u32 {
            let r = next();
            let f = match r % 5 {
                0 => 1.0,
                1 => -((r % 1000) as f64) * 0.25,
                _ => (r % 997) as f64 * 0.125,
            };
            h.push(f, idx);
            reference.push((f, idx));
            if next() % 3 == 0 {
                reference.sort_by(|a, b| b.0.total_cmp(&a.0).then(b.1.cmp(&a.1)));
                want.push(reference.pop().unwrap().1);
                out.push(h.pop().unwrap());
            }
        }
        reference.sort_by(|a, b| b.0.total_cmp(&a.0).then(b.1.cmp(&a.1)));
        while let Some(i) = h.pop() {
            out.push(i);
            want.push(reference.pop().unwrap().1);
        }
        assert_eq!(out, want);
        assert!(reference.is_empty());
    }

    #[test]
    fn routes_two_pads() {
        let b = crate::topo::tests::two_pad_board();
        let sub = Substrate::build(&b, &SubstrateConfig::default());
        let mut r = Router::new(&b, &sub, RouterConfig::default());
        r.plan();
        assert_eq!(r.conns.len(), 1);
        assert_eq!(r.run(), 1);
        let geo = r.geometry();
        assert_eq!(geo.tracks.len(), 1);
        let p = &geo.tracks[0].pts;
        // nearly straight: refined pad sides can leave a tiny kink before the relaxation
        let len: f64 = p.windows(2).map(|w| w[0].dist(w[1])).sum();
        assert!((len - 6.0).abs() < 0.01, "straight track expected: {p:?}");
        assert!(crate::check::check(&b, &geo).is_empty());
    }

    #[test]
    fn a_kept_via_into_the_plane_needs_no_fan_out() {
        let mut b = crate::topo::tests::two_pad_board();
        b.layers[1].is_plane = true;
        b.planes.push(crate::board::Plane::new(0, 1, b.outline[0].clone()));
        b.obstacles.push(crate::board::Obstacle {
            shape: Shape::Circle { c: pt(2., 5.), r: 0.3 },
            layers: b.all_layers(),
            net: Some(0),
            kind: ObstacleKind::Via,
            hole: Some(0.15),
            clearance: 0.2,
            pin: None,
            locked: false,
        });
        let sub = Substrate::build(&b, &SubstrateConfig::default());
        let mut r = Router::new(&b, &sub, RouterConfig::default());
        r.plan();
        assert_eq!(r.conns.len(), 1);
        assert_eq!(r.conns[0].from, 1);
    }

    #[test]
    fn slot_overflow_tells_how_much_is_missing() {
        let p = Placed {
            need_u: 0.1,
            need_v: 0.1,
            bounds: (f64::NEG_INFINITY, f64::INFINITY),
            gap_next: 0.4,
            gap_to_new: 0.4,
            gap_from_new: 0.4,
        };
        let view = |len| EdgeView {
            placed: vec![p, p],
            len,
            hot: 0.0,
            need_u: 0.1,
            need_v: 0.1,
            apex: [(f64::INFINITY, f64::NEG_INFINITY); 2],
        };
        let mut lo = Vec::new();
        // three tracks need 0.1 + 0.4 + 0.4 + 0.1 mm
        let tight = view(0.9);
        for j in 0..3 {
            assert!(Router::slot_interval(&tight, [2, 2], j, &mut lo).is_none());
            let (over, at) = Router::slot_overflow(&tight, [2, 2], j, &mut lo);
            assert!((over - 0.1).abs() < 1e-9, "slot {j}: {over}");
            assert!((0.0..=0.9).contains(&at));
        }
        let wide = view(1.0);
        assert!(Router::slot_interval(&wide, [2, 2], 1, &mut lo).is_some());
        assert!(Router::slot_overflow(&wide, [2, 2], 1, &mut lo).0.abs() < 1e-9);
    }

    /// Under a minimum width rule a fine pitch pin between two wide ones joins last: the two
    /// wide pins connect to each other, and the narrow one hangs on them as a branch.
    #[test]
    fn a_narrow_pin_is_never_on_the_way() {
        let mut b = crate::topo::tests::two_pad_board();
        b.classes[0].track_width = 0.75;
        b.classes[0].min_width = Some(0.69);
        b.classes[0].neck_refs = vec!["U1".into()];
        let mut other = b.nets[0].clone();
        other.name = "B".into();
        other.pins = vec![3];
        b.nets.push(other);
        // the narrow pin in the middle, with a pin of another net 0.5 mm next to it
        for (i, (x, net)) in [(5.0, 0), (5.5, 1)].into_iter().enumerate() {
            let mut o = b.obstacles[0].clone();
            o.shape = Shape::Polygon(crate::geom::roundrect_polygon(pt(x, 5.), 0.25, 0.8, 0.0, 0.0, 0.005));
            o.net = Some(net);
            o.pin = Some(2 + i);
            b.obstacles.push(o);
            let mut p = b.pins[0].clone();
            p.name = format!("U1.{}", i + 1);
            p.footprint = "U1".into();
            p.net = net;
            p.pos = pt(x, 5.);
            p.obstacles = vec![2 + i];
            p.size = (0.25, 0.8);
            b.pins.push(p);
        }
        b.nets[0].pins = vec![0, 2, 1];
        assert!(b.narrow_pin(2) && !b.narrow_pin(0) && !b.narrow_pin(1));
        let sub = Substrate::build(&b, &SubstrateConfig::default());
        let mut r = Router::new(&b, &sub, RouterConfig::default());
        r.plan();
        let goal = |from: usize| match &r.conns.iter().find(|c| c.from == from).unwrap().goal {
            Goal::Pins(p) => p.clone(),
            Goal::Plane => Vec::new(),
        };
        assert_eq!(r.conns.len(), 2);
        assert_eq!(goal(1), vec![0]);
        assert_eq!(goal(2), vec![0, 1]);
        // without the rule the nearest pin joins first, as always
        b.classes[0].min_width = None;
        let mut r = Router::new(&b, &sub, RouterConfig::default());
        r.plan();
        assert!(r.conns.iter().any(|c| c.from == 2 && c.goal == Goal::Pins(vec![0])));
    }

    /// Two nets that cross on the pad layer: a board where A runs left to right and B top to
    /// bottom.
    fn crossing_board() -> Board {
        let mut b = crate::topo::tests::two_pad_board();
        let mut net = b.nets[0].clone();
        net.name = "B".into();
        net.pins = vec![2, 3];
        b.nets.push(net);
        for (i, y) in [2.0, 8.0].into_iter().enumerate() {
            let mut o = b.obstacles[0].clone();
            o.shape = Shape::Polygon(crate::geom::roundrect_polygon(pt(5., y), 1.0, 1.0, 0.0, 0.0, 0.005));
            o.net = Some(1);
            o.pin = Some(2 + i);
            b.obstacles.push(o);
            let mut p = b.pins[0].clone();
            p.name = format!("Q{i}");
            p.footprint = p.name.clone();
            p.net = 1;
            p.pos = pt(5., y);
            p.obstacles = vec![2 + i];
            b.pins.push(p);
        }
        b
    }

    /// Net A from pad to pad on F.Cu, and a solder mask opening across the board between them.
    fn opening_board(net: Option<NetId>) -> Board {
        let mut b = crate::topo::tests::two_pad_board();
        let mut other = b.nets[0].clone();
        other.name = "B".into();
        other.pins = Vec::new();
        b.nets.push(other);
        b.obstacles.push(crate::board::Obstacle {
            shape: Shape::Polygon(crate::geom::Polygon::new(vec![
                pt(4.5, -1.),
                pt(5.5, -1.),
                pt(5.5, 11.),
                pt(4.5, 11.),
            ])),
            layers: crate::board::LayerSet::single(0),
            net,
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
        b
    }

    fn route_all(b: &Board) -> crate::realize::Geometry {
        let sub = Substrate::build(b, &SubstrateConfig::default());
        let mut r = Router::new(b, &sub, RouterConfig::default());
        r.plan();
        assert_eq!(r.run(), 1);
        r.geometry()
    }

    fn crosses_on_top(geo: &crate::realize::Geometry) -> bool {
        let top = geo.tracks.iter().filter(|t| t.layer == 0);
        top.flat_map(|t| t.pts.windows(2))
            .any(|w| w[0].x.min(w[1].x) < 5.0 && w[0].x.max(w[1].x) > 5.0)
    }

    /// The net of the copper in an opening routes through it with real track.
    #[test]
    fn its_own_net_routes_through_an_opening() {
        let b = opening_board(Some(0));
        let geo = route_all(&b);
        assert!(crosses_on_top(&geo));
        assert!(geo.vias.is_empty());
        assert!(crate::check::check(&b, &geo).is_empty());
    }

    /// For another net, or none, an opening is closed: tracks change the layer below it, and
    /// vias stay out.
    #[test]
    fn other_nets_stay_out_of_an_opening() {
        for net in [Some(1), None] {
            let b = opening_board(net);
            let geo = route_all(&b);
            assert!(!crosses_on_top(&geo));
            assert!(
                geo.vias
                    .iter()
                    .all(|v| v.drill == 0.0 || (v.at.x - 5.0).abs() > 0.5 + v.diameter / 2.0)
            );
            assert!(crate::check::check(&b, &geo).is_empty());
        }
    }

    /// The check reports copper of another net in an opening, and in the margin of a pad.
    #[test]
    fn the_check_sees_mask_openings() {
        let geo = route_all(&opening_board(Some(0)));
        let hit = crate::check::check(&opening_board(Some(1)), &geo);
        assert!(!hit.is_empty() && hit.iter().all(|v| v.what == "mask A~B" && v.nets == (0, Some(1))));
        // a pad of B beside the track, clear of the copper rule of 0.2 but with a margin of 0.5
        let mut b = opening_board(Some(0));
        b.obstacles.push(crate::board::Obstacle {
            shape: Shape::Circle { c: pt(5., 5.6), r: 0.2 },
            layers: crate::board::LayerSet::single(0),
            net: Some(1),
            kind: ObstacleKind::Pad,
            hole: None,
            clearance: 0.5,
            pin: None,
            locked: true,
        });
        let hit = crate::check::check(&b, &geo);
        assert!(
            !hit.is_empty()
                && hit
                    .iter()
                    .all(|v| v.what == "mask A~B" && (v.required - 0.5).abs() < 1e-9)
        );
    }

    #[test]
    fn a_negotiation_ends_legal() {
        let b = crossing_board();
        let sub = Substrate::build(&b, &SubstrateConfig::default());
        for rounds in [1, 8] {
            let cfg = RouterConfig {
                negotiate: rounds,
                ..Default::default()
            };
            let mut r = Router::new(&b, &sub, cfg);
            r.plan();
            assert_eq!(r.run(), 2, "{rounds} rounds");
            let (_, edges, crossings) = r.conflicts(0.0);
            assert_eq!((edges, crossings), (0, 0), "{rounds} rounds");
            assert!(crate::check::check(&b, &r.geometry()).is_empty());
        }
    }

    #[test]
    fn the_first_pass_of_a_negotiation_may_cross() {
        let b = crossing_board();
        let sub = Substrate::build(&b, &SubstrateConfig::default());
        let mut r = Router::new(&b, &sub, RouterConfig::default());
        r.plan();
        r.prices = Some(Prices {
            soft: true,
            overflow: 2.0,
            crossing: 0.5,
            history: r.cross.iter().map(|l| vec![0.0; l.len()]).collect(),
            tight: 0.0,
            reserve: 0.05,
            solid: false,
        });
        assert!(r.route_one(0) && r.route_one(1));
        assert_eq!(r.conflicts(0.0).2, 1);
    }
}
