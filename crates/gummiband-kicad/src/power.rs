//! Good practice around routing: current carrying capacity after IPC-2221, the narrowest
//! cross-section of a net's copper between its terminals (pours and vias included), and free
//! spots for ground stitching vias.
//!
//! IPC-2221: I = k * dT^0.44 * A^0.725 with A in mil², k = 0.048 for outer and 0.024 for inner
//! layers. A via barrel counts as an inner conductor with the plating as its thickness.

use std::collections::{HashMap, VecDeque};

use gummiband_core::board::{Board, LayerId, NetId, ObstacleKind};
use gummiband_core::geom::{Point, Polygon, Rect, Shape, pt};

use crate::read::KicadBoard;
use crate::sexpr::Node;

const MIL: f64 = 0.0254;
/// Copper thickness when the board has no stackup (1 oz).
pub const DEFAULT_COPPER_MM: f64 = gummiband_core::board::COPPER_MM;
/// Via plating thickness (IPC class 2 average).
pub const VIA_PLATING_MM: f64 = gummiband_core::board::PLATING_MM;

fn k(outer: bool) -> f64 {
    if outer { 0.048 } else { 0.024 }
}

/// Track width in mm that carries `amps` at a temperature rise of `dt` kelvin.
pub fn ipc_width(amps: f64, copper_mm: f64, outer: bool, dt: f64) -> f64 {
    if amps <= 0.0 {
        return 0.0;
    }
    let area_mil2 = (amps / (k(outer) * dt.powf(0.44))).powf(1.0 / 0.725);
    area_mil2 / (copper_mm / MIL) * MIL
}

/// Current in A that a track of `width` mm carries at a rise of `dt` kelvin.
pub fn ipc_amps(width: f64, copper_mm: f64, outer: bool, dt: f64) -> f64 {
    let area_mil2 = (width / MIL) * (copper_mm / MIL);
    k(outer) * dt.powf(0.44) * area_mil2.max(0.0).powf(0.725)
}

/// Current in A that one via barrel of `drill` mm carries at a rise of `dt` kelvin.
pub fn via_amps(drill: f64, dt: f64) -> f64 {
    let area_mm2 = std::f64::consts::PI * (drill + VIA_PLATING_MM) * VIA_PLATING_MM;
    k(false) * dt.powf(0.44) * (area_mm2 / (MIL * MIL)).powf(0.725)
}

/// Copper thickness of a layer from the stackup, 35 um without one.
pub fn copper_mm(b: &Board, layer: LayerId) -> f64 {
    b.copper_thickness(layer)
}

/// Outer copper layer (first or last of the board).
pub fn is_outer(b: &Board, layer: LayerId) -> bool {
    layer == 0 || layer as usize + 1 == b.layers.len()
}

/// Glob match with `*` (any run) and `?` (one character), case sensitive.
pub fn glob(pat: &str, s: &str) -> bool {
    let (p, t): (Vec<char>, Vec<char>) = (pat.chars().collect(), s.chars().collect());
    let (mut i, mut j, mut star, mut mark) = (0usize, 0usize, None, 0usize);
    while j < t.len() {
        if i < p.len() && (p[i] == '?' || p[i] == t[j]) {
            i += 1;
            j += 1;
        } else if i < p.len() && p[i] == '*' {
            star = Some(i);
            mark = j;
            i += 1;
        } else if let Some(s) = star {
            i = s + 1;
            mark += 1;
            j = mark;
        } else {
            return false;
        }
    }
    while i < p.len() && p[i] == '*' {
        i += 1;
    }
    i == p.len()
}

// -------------------------------------------------------------------------------- zones

/// A copper zone of the file on one layer: its filled polygons, or its outline if KiCad has
/// not filled it (`filled` false).
#[derive(Debug, Clone)]
pub struct ZoneFill {
    pub net: Option<NetId>,
    pub layer: LayerId,
    pub name: String,
    pub priority: i64,
    pub filled: bool,
    pub polys: Vec<Polygon>,
}

fn pts_of(n: &Node) -> Vec<Point> {
    n.child("pts")
        .map(|p| {
            p.children("xy")
                .filter_map(|xy| {
                    let a = xy.args();
                    Some(pt(a.first()?.as_f64()?, a.get(1)?.as_f64()?))
                })
                .collect()
        })
        .unwrap_or_default()
}

/// The copper zones (not rule areas) at the root of the board file.
pub fn zones(kb: &KicadBoard) -> Vec<ZoneFill> {
    let b = &kb.board;
    let by_number: HashMap<i64, &str> = kb
        .net_numbers
        .as_ref()
        .map(|m| m.iter().map(|(k, v)| (*v, k.as_str())).collect())
        .unwrap_or_default();
    let mut out = Vec::new();
    for z in kb.root.children("zone") {
        if z.child("keepout").is_some() {
            continue;
        }
        let name = z.child_str("net_name").map(str::to_string).or_else(|| {
            let a = z.child("net")?.args();
            let first = a.first()?;
            match (first.as_f64(), first.as_str()) {
                (Some(num), _) if !by_number.is_empty() => by_number.get(&(num as i64)).map(|s| s.to_string()),
                (_, Some(s)) => Some(s.to_string()),
                _ => None,
            }
        });
        let net = name.as_deref().filter(|n| !n.is_empty()).and_then(|n| b.net_id(n));
        let mut layers: Vec<&str> = Vec::new();
        if let Some(l) = z.child("layers") {
            layers.extend(l.args().iter().filter_map(|x| x.as_str()));
        }
        if let Some(l) = z.child("layer") {
            layers.extend(l.args().iter().filter_map(|x| x.as_str()));
        }
        let mut names: Vec<String> = Vec::new();
        for l in layers {
            match l {
                "*.Cu" | "F&B.Cu" => names.extend(kb.layer_names.iter().cloned()),
                _ => names.push(l.to_string()),
            }
        }
        let outline = z.child("polygon").map(pts_of).unwrap_or_default();
        let priority = z.child_f64("priority").unwrap_or(0.0) as i64;
        let zname = z.child_str("name").unwrap_or("").to_string();
        for lname in names {
            let Some(layer) = b.layer_id(&lname) else { continue };
            let polys: Vec<Polygon> = z
                .children("filled_polygon")
                .filter(|f| f.child_str("layer") == Some(lname.as_str()))
                .map(pts_of)
                .filter(|p| p.len() >= 3)
                .map(Polygon::new)
                .collect();
            let filled = !polys.is_empty();
            let polys = if filled {
                polys
            } else if outline.len() >= 3 {
                vec![Polygon::new(outline.clone())]
            } else {
                continue;
            };
            out.push(ZoneFill {
                net,
                layer,
                name: zname.clone(),
                priority,
                filled,
                polys,
            });
        }
    }
    out
}

// ------------------------------------------------------------------------------- raster

struct Grid {
    x0: f64,
    y0: f64,
    step: f64,
    nx: usize,
    ny: usize,
}

impl Grid {
    fn n(&self) -> usize {
        self.nx * self.ny
    }

    fn center(&self, i: usize) -> Point {
        pt(
            self.x0 + ((i % self.nx) as f64 + 0.5) * self.step,
            self.y0 + ((i / self.nx) as f64 + 0.5) * self.step,
        )
    }

    fn cell(&self, p: Point) -> Option<usize> {
        let c = ((p.x - self.x0) / self.step).floor();
        let r = ((p.y - self.y0) / self.step).floor();
        (c >= 0.0 && r >= 0.0 && (c as usize) < self.nx && (r as usize) < self.ny)
            .then(|| r as usize * self.nx + c as usize)
    }

    fn range(&self, r: Rect) -> (usize, usize, usize, usize) {
        let c0 = (((r.min.x - self.x0) / self.step).floor().max(0.0)) as usize;
        let r0 = (((r.min.y - self.y0) / self.step).floor().max(0.0)) as usize;
        let c1 = ((((r.max.x - self.x0) / self.step).ceil() + 1.0).max(0.0) as usize).min(self.nx);
        let r1 = ((((r.max.y - self.y0) / self.step).ceil() + 1.0).max(0.0) as usize).min(self.ny);
        (c0, r0, c1, r1)
    }

    fn fill_shape(&self, m: &mut [bool], s: &Shape) {
        let (c0, r0, c1, r1) = self.range(s.bbox());
        for r in r0..r1 {
            for c in c0..c1 {
                let i = r * self.nx + c;
                if !m[i] && s.dist_to_point(self.center(i)) <= 1e-9 {
                    m[i] = true;
                }
            }
        }
    }

    /// Even-odd scanline fill of a polygon.
    fn fill_poly(&self, m: &mut [bool], p: &Polygon) {
        let (c0, r0, c1, r1) = self.range(p.bbox());
        let mut xs = Vec::new();
        for r in r0..r1 {
            let y = self.y0 + (r as f64 + 0.5) * self.step;
            xs.clear();
            for (a, b) in p.edges() {
                if (a.y > y) != (b.y > y) {
                    xs.push(a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y));
                }
            }
            xs.sort_by(f64::total_cmp);
            for w in xs.chunks(2) {
                if w.len() < 2 {
                    break;
                }
                for c in c0..c1 {
                    let x = self.x0 + (c as f64 + 0.5) * self.step;
                    if x >= w[0] && x < w[1] {
                        m[r * self.nx + c] = true;
                    }
                }
            }
        }
    }
}

/// Euclidean distance (in cells) from every cell to the nearest cell where `m` is false
/// (Felzenszwalb and Huttenlocher); outside the raster counts as false.
#[allow(clippy::needless_range_loop)]
fn edt(m: &[bool], nx: usize, ny: usize) -> Vec<f32> {
    const INF: f64 = 1e18;
    fn dt1(f: &[f64], d: &mut [f64], v: &mut [usize], z: &mut [f64]) {
        let n = f.len();
        let mut k = 0usize;
        v[0] = 0;
        z[0] = -INF;
        z[1] = INF;
        for q in 1..n {
            loop {
                let p = v[k];
                let s = ((f[q] + (q * q) as f64) - (f[p] + (p * p) as f64)) / (2.0 * (q as f64 - p as f64));
                if s <= z[k] && k > 0 {
                    k -= 1;
                    continue;
                }
                if s <= z[k] {
                    // k == 0: replace the first parabola
                    v[0] = q;
                    z[0] = -INF;
                    z[1] = INF;
                    break;
                }
                k += 1;
                v[k] = q;
                z[k] = s;
                z[k + 1] = INF;
                break;
            }
        }
        k = 0;
        for q in 0..n {
            while z[k + 1] < q as f64 {
                k += 1;
            }
            let p = v[k];
            d[q] = (q as f64 - p as f64).powi(2) + f[p];
        }
    }
    // pad by one false cell on every side
    let (w, h) = (nx + 2, ny + 2);
    let mut g = vec![0f64; w * h];
    for r in 0..ny {
        for c in 0..nx {
            if m[r * nx + c] {
                g[(r + 1) * w + c + 1] = INF;
            }
        }
    }
    let len = w.max(h);
    let (mut f, mut d, mut v, mut z) = (vec![0f64; len], vec![0f64; len], vec![0usize; len], vec![0f64; len + 1]);
    for c in 0..w {
        for r in 0..h {
            f[r] = g[r * w + c];
        }
        dt1(&f[..h], &mut d[..h], &mut v, &mut z);
        for r in 0..h {
            g[r * w + c] = d[r];
        }
    }
    for r in 0..h {
        f[..w].copy_from_slice(&g[r * w..(r + 1) * w]);
        dt1(&f[..w], &mut d[..w], &mut v, &mut z);
        g[r * w..(r + 1) * w].copy_from_slice(&d[..w]);
    }
    let mut out = vec![0f32; nx * ny];
    for r in 0..ny {
        for c in 0..nx {
            out[r * nx + c] = g[(r + 1) * w + c + 1].sqrt() as f32;
        }
    }
    out
}

// --------------------------------------------------------------------------------- neck

/// A terminal: pins in parallel (any of them may carry the current).
pub type Terminal = Vec<usize>;

#[derive(Debug, Clone)]
pub struct NeckOptions {
    /// Width factor per copper layer relative to the reference layer (required width there
    /// divided by the required width on the reference layer); 0 leaves the layer out.
    pub layer_factor: Vec<f64>,
    /// Raster step in mm (made coarser on large nets).
    pub step: f64,
    /// Largest width searched, in mm on the reference layer.
    pub max_width: f64,
    /// How far from a terminal or via pad a band may end, at most (mm).
    pub end_reach: f64,
    /// Temperature rise and the copper of the reference layer, to turn a via into the width it
    /// stands for in the cross-section.
    pub temp_rise: f64,
    pub ref_copper_mm: f64,
    pub ref_outer: bool,
    /// Required width on the reference layer for the cross-section check (min cut up to twice
    /// that); 0 skips it.
    pub required: f64,
}

#[derive(Debug, Clone)]
pub struct Neck {
    /// Every source terminal reaches a sink through the copper of the net.
    pub connected: bool,
    /// Largest band width (reference layer mm) that still joins every source terminal to the
    /// sinks: the narrowest cross-section the current has to pass.
    pub width_mm: f64,
    /// Where the band breaks first: position and layer.
    pub at: Option<(Point, LayerId)>,
    /// Raster step used.
    pub step: f64,
    /// Zones of the net that were not filled (outline used instead: optimistic).
    pub unfilled_zones: usize,
    /// The current has to change layer (through vias or plated holes) at the measured width.
    pub changes_layer: bool,
    /// Smallest total cross-section (min cut, parallel paths and vias added up) between each
    /// source terminal and the sinks, in mm on the reference layer, up to the limit; None when
    /// not measured.
    pub cut_mm: Option<f64>,
    /// Where that cut lies, if it is below the limit.
    pub cut_at: Option<(Point, LayerId)>,
}

/// The narrowest cross-section of `net` between the source terminals and the sinks: pads,
/// tracks, vias and the zones of the net on the layers with a non-zero factor, vias and plated
/// holes joining the layers. Terminal pads and via pads are not eroded and a band may end
/// within half its width of them (copper attached to a pad end), at most `end_reach` away.
/// Port of the Lichtschnitzel `layout/necks.py`.
pub fn neck(
    kb: &KicadBoard,
    zf: &[ZoneFill],
    net: NetId,
    sources: &[Terminal],
    sinks: &[usize],
    o: &NeckOptions,
) -> Neck {
    let b = &kb.board;
    let nl = b.layers.len();
    let active: Vec<bool> = (0..nl)
        .map(|l| o.layer_factor.get(l).copied().unwrap_or(0.0) > 0.0)
        .collect();
    let obs: Vec<usize> = b
        .obstacles
        .iter()
        .enumerate()
        .filter(|(_, x)| {
            x.net == Some(net)
                && matches!(
                    x.kind,
                    ObstacleKind::Pad
                        | ObstacleKind::Track
                        | ObstacleKind::Via
                        | ObstacleKind::Zone
                        | ObstacleKind::Pour
                )
        })
        .map(|(i, _)| i)
        .collect();
    let zs: Vec<&ZoneFill> = zf
        .iter()
        .filter(|z| z.net == Some(net) && active[z.layer as usize])
        .collect();
    let mut bbox = Rect::empty();
    for &i in &obs {
        let r = b.obstacles[i].shape.bbox();
        bbox.add(r.min);
        bbox.add(r.max);
    }
    for z in &zs {
        for p in &z.polys {
            let r = p.bbox();
            bbox.add(r.min);
            bbox.add(r.max);
        }
    }
    let bbox = bbox.grow(1.0);
    let area = bbox.width() * bbox.height();
    let step = o.step.max((area / 1.5e6).sqrt());
    let g = Grid {
        x0: bbox.min.x,
        y0: bbox.min.y,
        step,
        nx: (bbox.width() / step).ceil() as usize + 1,
        ny: (bbox.height() / step).ceil() as usize + 1,
    };
    let n = g.n();
    let unfilled_zones = zs.iter().filter(|z| !z.filled).count();
    let layers: Vec<usize> = (0..nl).filter(|&l| active[l]).collect();
    let mut cu: Vec<Vec<bool>> = vec![Vec::new(); nl];
    let mut vdisk: Vec<Vec<bool>> = vec![Vec::new(); nl];
    let mut term: Vec<Vec<bool>> = vec![Vec::new(); nl];
    for &l in &layers {
        let mut m = vec![false; n];
        let mut v = vec![false; n];
        for &i in &obs {
            let x = &b.obstacles[i];
            if x.layers.has(l as LayerId) {
                g.fill_shape(&mut m, &x.shape);
                if x.kind == ObstacleKind::Via {
                    g.fill_shape(&mut v, &x.shape);
                }
            }
        }
        for z in zs.iter().filter(|z| z.layer as usize == l) {
            for p in &z.polys {
                g.fill_poly(&mut m, p);
            }
        }
        let mut t = vec![false; n];
        for &pin in sources.iter().flatten().chain(sinks) {
            for &oi in &b.pins[pin].obstacles {
                let x = &b.obstacles[oi];
                if x.layers.has(l as LayerId) {
                    g.fill_shape(&mut t, &x.shape);
                }
            }
        }
        for i in 0..n {
            t[i] &= m[i];
            v[i] &= m[i];
        }
        cu[l] = m;
        vdisk[l] = v;
        term[l] = t;
    }
    // vias and plated holes join the layers they span
    let mut joints: HashMap<usize, Vec<usize>> = HashMap::new();
    // what the barrels at a cell carry, as a width on the reference layer
    let mut joint_mm: HashMap<usize, f64> = HashMap::new();
    for &i in &obs {
        let x = &b.obstacles[i];
        if x.hole.is_none() || !matches!(x.kind, ObstacleKind::Via | ObstacleKind::Pad) {
            continue;
        }
        let c = x.shape.bbox().center();
        if let Some(cell) = g.cell(c) {
            let drill = 2.0 * x.hole.unwrap_or(0.0);
            let a = via_amps(drill, o.temp_rise);
            *joint_mm.entry(cell).or_default() += ipc_width(a, o.ref_copper_mm, o.ref_outer, o.temp_rise);
            let e = joints.entry(cell).or_default();
            for &l in &layers {
                if x.layers.has(l as LayerId) && !e.contains(&l) {
                    e.push(l);
                }
            }
        }
    }
    // the cells of each terminal
    let cells_of = |pins: &[usize]| -> Vec<(usize, usize)> {
        let mut out = Vec::new();
        for &pin in pins {
            for &oi in &b.pins[pin].obstacles {
                let x = &b.obstacles[oi];
                for &l in &layers {
                    if !x.layers.has(l as LayerId) {
                        continue;
                    }
                    let (c0, r0, c1, r1) = g.range(x.shape.bbox());
                    for r in r0..r1 {
                        for c in c0..c1 {
                            let i = r * g.nx + c;
                            if cu[l][i] && x.shape.dist_to_point(g.center(i)) <= 1e-9 {
                                out.push((l, i));
                            }
                        }
                    }
                }
            }
        }
        out
    };
    let src_cells: Vec<Vec<(usize, usize)>> = sources.iter().map(|t| cells_of(t)).collect();
    let sink_cells: Vec<(usize, usize)> = cells_of(sinks);
    let d_cu: Vec<Vec<f32>> = (0..nl)
        .map(|l| if active[l] { edt(&cu[l], g.nx, g.ny) } else { Vec::new() })
        .collect();
    let inv = |m: &[bool]| m.iter().map(|x| !x).collect::<Vec<bool>>();
    let d_term: Vec<Vec<f32>> = (0..nl)
        .map(|l| {
            if active[l] {
                edt(&inv(&term[l]), g.nx, g.ny)
            } else {
                Vec::new()
            }
        })
        .collect();
    let d_via: Vec<Vec<f32>> = (0..nl)
        .map(|l| {
            if active[l] {
                edt(&inv(&vdisk[l]), g.nx, g.ny)
            } else {
                Vec::new()
            }
        })
        .collect();
    let kept = |l: usize, i: usize, rad: f64| -> bool {
        if !cu[l][i] {
            return false;
        }
        let rl = (rad * o.layer_factor[l]) as f32;
        // a band may end within half its width of a pad, but at most `end_reach` away: a short
        // thin track between two close pads is a neck too
        let re = rl.min((o.end_reach / step) as f32);
        rad <= 0.0 || d_cu[l][i] > rl || d_term[l][i] <= re || d_via[l][i] <= re
    };
    let flood_with = |seeds: &[(usize, usize)], rad: f64, use_joints: bool| -> Vec<Vec<bool>> {
        let mut seen: Vec<Vec<bool>> = (0..nl)
            .map(|l| if active[l] { vec![false; n] } else { Vec::new() })
            .collect();
        let mut q = VecDeque::new();
        for &(l, i) in seeds {
            if kept(l, i, rad) && !seen[l][i] {
                seen[l][i] = true;
                q.push_back((l, i));
            }
        }
        while let Some((l, i)) = q.pop_front() {
            let (r, c) = (i / g.nx, i % g.nx);
            let mut next = [usize::MAX; 4];
            if c > 0 {
                next[0] = i - 1;
            }
            if c + 1 < g.nx {
                next[1] = i + 1;
            }
            if r > 0 {
                next[2] = i - g.nx;
            }
            if r + 1 < g.ny {
                next[3] = i + g.nx;
            }
            for j in next {
                if j != usize::MAX && !seen[l][j] && kept(l, j, rad) {
                    seen[l][j] = true;
                    q.push_back((l, j));
                }
            }
            if let Some(ls) = joints.get(&i).filter(|_| use_joints)
                && ls.contains(&l)
            {
                for &m in ls {
                    if !seen[m][i] && kept(m, i, rad) {
                        seen[m][i] = true;
                        q.push_back((m, i));
                    }
                }
            }
        }
        seen
    };
    let flood = |seeds: &[(usize, usize)], rad: f64| flood_with(seeds, rad, true);
    let joins_with = |rad: f64, use_joints: bool| -> bool {
        let seen = flood_with(&sink_cells, rad, use_joints);
        src_cells.iter().all(|t| t.iter().any(|&(l, i)| seen[l][i]))
    };
    let joins = |rad: f64| joins_with(rad, true);
    let mut out = Neck {
        connected: false,
        width_mm: 0.0,
        at: None,
        step,
        unfilled_zones,
        changes_layer: false,
        cut_mm: None,
        cut_at: None,
    };
    if src_cells.iter().any(Vec::is_empty) || sink_cells.is_empty() || !joins(0.0) {
        return out;
    }
    out.connected = true;
    if o.required > 0.0 {
        let cells: usize = layers.iter().map(|&l| cu[l].iter().filter(|x| **x).count()).sum();
        if cells <= 3_000_000 {
            let mut worst: Option<(f64, Option<(Point, LayerId)>)> = None;
            // copper that is a band of the required width anyway cannot be the cut: only narrower
            // copper in parallel and the via barrels are left to add up
            // (half a cell of tolerance, and grown by a cell so the band stays 4-connected
            // where it runs at an angle)
            let rad = (o.required / 2.0 / step - 0.5).max(0.0);
            let wide: Vec<Vec<bool>> = (0..nl)
                .map(|l| {
                    if !active[l] {
                        return Vec::new();
                    }
                    let core: Vec<bool> = (0..n).map(|i| kept(l, i, rad)).collect();
                    (0..n)
                        .map(|i| {
                            let (r, c) = (i / g.nx, i % g.nx);
                            cu[l][i]
                                && (core[i]
                                    || (c > 0 && core[i - 1])
                                    || (c + 1 < g.nx && core[i + 1])
                                    || (r > 0 && core[i - g.nx])
                                    || (r + 1 < g.ny && core[i + g.nx]))
                        })
                        .collect()
                })
                .collect();
            for t in &src_cells {
                let r = min_cut(
                    &g,
                    &layers,
                    &cu,
                    &wide,
                    &o.layer_factor,
                    &joints,
                    &joint_mm,
                    t,
                    &sink_cells,
                    2.0 * o.required,
                );
                if worst.as_ref().is_none_or(|w| r.0 < w.0) {
                    worst = Some(r);
                }
            }
            if let Some((w, at)) = worst {
                out.cut_mm = Some(w);
                out.cut_at = at;
            }
        }
    }
    let (mut lo, mut hi) = (0.0f64, o.max_width);
    if joins(hi / 2.0 / step) {
        out.width_mm = hi;
        out.changes_layer = !joins_with(0.9 * hi / 2.0 / step, false);
        return out;
    }
    while hi - lo > step / 4.0 {
        let w = (lo + hi) / 2.0;
        if joins(w / 2.0 / step) {
            lo = w;
        } else {
            hi = w;
        }
    }
    out.width_mm = lo;
    out.changes_layer = !joins_with(0.9 * lo / 2.0 / step, false);
    // the neck: copper kept at lo but not at hi, between what the sinks and the sources reach at hi
    let (rlo, rhi) = (lo / 2.0 / step, hi / 2.0 / step);
    let from_sinks = flood(&sink_cells, rhi);
    let src_all: Vec<(usize, usize)> = src_cells.iter().flatten().copied().collect();
    let from_src = flood(&src_all, rhi);
    let mut best: Option<(f32, usize, usize)> = None;
    for &l in &layers {
        let da = edt(&inv(&from_sinks[l]), g.nx, g.ny);
        let db = edt(&inv(&from_src[l]), g.nx, g.ny);
        for i in 0..n {
            if kept(l, i, rlo) && !kept(l, i, rhi) {
                let s = da[i] + db[i];
                if best.is_none_or(|(bs, _, _)| s < bs) {
                    best = Some((s, l, i));
                }
            }
        }
    }
    out.at = best.map(|(_, l, i)| (g.center(i), l as LayerId));
    out
}

// ------------------------------------------------------------------------------- min cut

const NIL: u32 = u32::MAX;
const INF: i64 = i64::MAX / 4;

/// Max flow (Dinic) on a graph with integer capacities.
struct Flow {
    head: Vec<u32>,
    to: Vec<u32>,
    next: Vec<u32>,
    cap: Vec<i64>,
}

impl Flow {
    fn new(n: usize) -> Self {
        Flow {
            head: vec![NIL; n],
            to: Vec::new(),
            next: Vec::new(),
            cap: Vec::new(),
        }
    }

    fn edge(&mut self, u: usize, v: usize, c: i64) {
        for (a, b, k) in [(u, v, c), (v, u, 0)] {
            self.to.push(b as u32);
            self.next.push(self.head[a]);
            self.cap.push(k);
            self.head[a] = (self.to.len() - 1) as u32;
        }
    }

    fn levels(&self, s: usize) -> Vec<i32> {
        let mut lv = vec![-1i32; self.head.len()];
        lv[s] = 0;
        let mut q = VecDeque::from([s]);
        while let Some(u) = q.pop_front() {
            let mut e = self.head[u];
            while e != NIL {
                let v = self.to[e as usize] as usize;
                if self.cap[e as usize] > 0 && lv[v] < 0 {
                    lv[v] = lv[u] + 1;
                    q.push_back(v);
                }
                e = self.next[e as usize];
            }
        }
        lv
    }

    /// One augmenting path in the level graph (iterative, current arcs in `it`).
    fn augment(&mut self, s: usize, t: usize, lim: i64, lv: &mut [i32], it: &mut [u32]) -> i64 {
        let mut stack: Vec<u32> = Vec::new();
        let mut u = s;
        loop {
            if u == t {
                let f = stack.iter().map(|&e| self.cap[e as usize]).min().unwrap_or(0).min(lim);
                for &e in &stack {
                    self.cap[e as usize] -= f;
                    self.cap[(e ^ 1) as usize] += f;
                }
                return f;
            }
            let mut moved = false;
            while it[u] != NIL {
                let e = it[u] as usize;
                let v = self.to[e] as usize;
                if self.cap[e] > 0 && lv[v] == lv[u] + 1 {
                    stack.push(e as u32);
                    u = v;
                    moved = true;
                    break;
                }
                it[u] = self.next[e];
            }
            if !moved {
                if u == s {
                    return 0;
                }
                lv[u] = -1;
                let e = stack.pop().unwrap() as usize;
                u = self.to[e ^ 1] as usize;
                it[u] = self.next[it[u] as usize];
            }
        }
    }

    fn max_flow(&mut self, s: usize, t: usize, limit: i64) -> i64 {
        let mut flow = 0;
        while flow < limit {
            let mut lv = self.levels(s);
            if lv[t] < 0 {
                break;
            }
            let mut it = self.head.clone();
            loop {
                let f = self.augment(s, t, limit - flow, &mut lv, &mut it);
                if f == 0 {
                    break;
                }
                flow += f;
                if flow >= limit {
                    break;
                }
            }
        }
        flow
    }
}

/// Smallest cross-section between a source terminal and the sinks: a node cut on the raster of
/// the copper (4-neighbourhood, so a cut at an angle counts a little short, on the safe side),
/// every cell weighted with its layer's factor, vias and plated holes carrying what their
/// barrels carry. Returns the width in mm on the reference layer (capped at `limit`) and where
/// the cut lies.
#[allow(clippy::too_many_arguments, clippy::needless_range_loop)]
fn min_cut(
    g: &Grid,
    layers: &[usize],
    cu: &[Vec<bool>],
    wide: &[Vec<bool>],
    factor: &[f64],
    joints: &HashMap<usize, Vec<usize>>,
    joint_mm: &HashMap<usize, f64>,
    src: &[(usize, usize)],
    sinks: &[(usize, usize)],
    limit: f64,
) -> (f64, Option<(Point, LayerId)>) {
    const S: f64 = 16.0;
    let n = g.n();
    let mut id: HashMap<(usize, usize), usize> = HashMap::new();
    let mut cells: Vec<(usize, usize)> = Vec::new();
    for &l in layers {
        for i in 0..n {
            if cu[l][i] {
                id.insert((l, i), cells.len());
                cells.push((l, i));
            }
        }
    }
    let k = cells.len();
    let nj = joints.len();
    let (s, t) = (2 * k + 2 * nj, 2 * k + 2 * nj + 1);
    let mut f = Flow::new(2 * k + 2 * nj + 2);
    let terminal: std::collections::HashSet<(usize, usize)> = src.iter().chain(sinks).copied().collect();
    for (j, &(l, i)) in cells.iter().enumerate() {
        // the barrel itself limits a via, not the raster cell at its centre
        let c = if terminal.contains(&(l, i))
            || joints.contains_key(&i)
            || wide.get(l).is_some_and(|w| w.get(i) == Some(&true))
        {
            INF
        } else {
            ((S / factor[l].max(1e-6)).round() as i64).max(1)
        };
        f.edge(2 * j, 2 * j + 1, c);
        let (r, col) = (i / g.nx, i % g.nx);
        if col + 1 < g.nx
            && let Some(&m) = id.get(&(l, i + 1))
        {
            f.edge(2 * j + 1, 2 * m, INF);
            f.edge(2 * m + 1, 2 * j, INF);
        }
        if r + 1 < g.ny
            && let Some(&m) = id.get(&(l, i + g.nx))
        {
            f.edge(2 * j + 1, 2 * m, INF);
            f.edge(2 * m + 1, 2 * j, INF);
        }
    }
    // a barrel is a node of its own: what it carries limits the current between any two of
    // its layers, whatever the copper of the layers in between
    let joint_cells: Vec<usize> = joints.keys().copied().collect();
    for (q, &i) in joint_cells.iter().enumerate() {
        let ls = &joints[&i];
        let c = ((joint_mm.get(&i).copied().unwrap_or(0.0) / g.step) * S)
            .round()
            .max(1.0) as i64;
        let (vin, vout) = (2 * k + 2 * q, 2 * k + 2 * q + 1);
        f.edge(vin, vout, c);
        for &l in ls {
            if let Some(&a) = id.get(&(l, i)) {
                f.edge(2 * a + 1, vin, INF);
                f.edge(vout, 2 * a, INF);
            }
        }
    }
    for &c in src {
        if let Some(&j) = id.get(&c) {
            f.edge(s, 2 * j, INF);
        }
    }
    for &c in sinks {
        if let Some(&j) = id.get(&c) {
            f.edge(2 * j + 1, t, INF);
        }
    }
    let lim = ((limit / g.step) * S).ceil() as i64;
    let flow = f.max_flow(s, t, lim);
    let width = flow as f64 / S * g.step;
    if flow >= lim {
        return (limit, None);
    }
    // the cut: cells whose entry the source side reaches and whose exit it does not
    let lv = f.levels(s);
    let mut cut: Vec<(usize, usize)> = cells
        .iter()
        .enumerate()
        .filter(|(j, _)| lv[2 * j] >= 0 && lv[2 * j + 1] < 0)
        .map(|(_, c)| *c)
        .collect();
    // saturated barrels: reported on the first layer of the via, at its centre
    for (q, &i) in joint_cells.iter().enumerate() {
        if lv[2 * k + 2 * q] >= 0 && lv[2 * k + 2 * q + 1] < 0 {
            cut.push((joints[&i].iter().copied().min().unwrap_or(0), i));
        }
    }
    if cut.is_empty() {
        return (width, None);
    }
    let mut sum = Point { x: 0.0, y: 0.0 };
    for &(_, i) in &cut {
        sum = sum + g.center(i);
    }
    let mid = sum * (1.0 / cut.len() as f64);
    let best = cut
        .iter()
        .min_by(|a, b| g.center(a.1).dist(mid).total_cmp(&g.center(b.1).dist(mid)))
        .copied()
        .unwrap();
    (width, Some((g.center(best.1), best.0 as LayerId)))
}

// ------------------------------------------------------------------------------ stitching

#[derive(Debug, Clone)]
pub struct StitchOptions {
    pub net: NetId,
    /// Grid pitch in mm (0: no grid).
    pub pitch: f64,
    /// Spacing along the board edge in mm (0: no edge row).
    pub edge_pitch: f64,
    /// Distance of the edge row from the outline, mm.
    pub edge_inset: f64,
    pub diameter: f64,
    pub drill: f64,
    /// Extra room around a via beyond the clearance, so the pour reaches it with some width.
    pub margin: f64,
    /// Keep vias out of footprint courtyards.
    pub avoid_courtyards: bool,
}

/// Where a via of one net fits: clear of copper and holes of other nets (on every layer, the
/// via goes through), of rule areas that forbid vias, of other nets' zones on routing layers,
/// of the board edge, of vias placed before and (optionally) of courtyards.
pub struct ViaRoom<'a> {
    b: &'a Board,
    net: NetId,
    r: f64,
    drill: f64,
    own: f64,
    margin: f64,
    avoid_courtyards: bool,
    /// Courtyards a via may sit in all the same (the part the vias belong to).
    own_parts: Vec<String>,
    foreign_zones: Vec<&'a ZoneFill>,
}

impl<'a> ViaRoom<'a> {
    pub fn new(
        b: &'a Board,
        zf: &'a [ZoneFill],
        net: NetId,
        diameter: f64,
        drill: f64,
        margin: f64,
        avoid_courtyards: bool,
    ) -> ViaRoom<'a> {
        ViaRoom {
            b,
            net,
            r: diameter / 2.0,
            drill,
            own: b.clearance(Some(net), None),
            margin,
            avoid_courtyards,
            own_parts: Vec::new(),
            foreign_zones: zf
                .iter()
                .filter(|z| z.net != Some(net) && !b.layers[z.layer as usize].is_plane)
                .collect(),
        }
    }

    /// Vias may sit in the courtyards of these parts.
    pub fn allow_in(mut self, parts: Vec<String>) -> Self {
        self.own_parts = parts;
        self
    }

    fn edge_dist(&self, p: Point) -> f64 {
        self.b
            .outline
            .iter()
            .map(|q| q.boundary_dist(p))
            .fold(f64::INFINITY, f64::min)
    }

    /// Center distance two vias of the net keep.
    pub fn pitch(&self) -> f64 {
        2.0 * self.r + self.b.rules.hole_to_hole.max(self.own)
    }

    pub fn free(&self, p: Point, placed: &[Point]) -> bool {
        let (b, r) = (self.b, self.r);
        if !b.inside(p) || self.edge_dist(p) < b.rules.edge_clearance + r {
            return false;
        }
        if placed.iter().any(|q| q.dist(p) < self.pitch()) {
            return false;
        }
        if self.avoid_courtyards
            && b.courtyards
                .iter()
                .any(|(part, c)| !self.own_parts.contains(part) && c.grow(r).contains(p))
        {
            return false;
        }
        for x in &b.obstacles {
            if let Some(h) = x.hole {
                let c = x.shape.bbox().center();
                if c.dist(p) - h - self.drill / 2.0 < b.rules.hole_to_hole {
                    return false;
                }
            }
            if x.net == Some(self.net) {
                continue;
            }
            let need = match x.kind {
                ObstacleKind::Keepout { vias, mask, .. } => {
                    if !vias || mask {
                        continue;
                    }
                    r
                }
                _ => r + x.clearance.max(self.own) + self.margin,
            };
            if x.shape.dist_to_point(p) < need {
                return false;
            }
        }
        for z in &self.foreign_zones {
            if z.polys.iter().any(|q| q.dist_to_point(p) < r + self.own + self.margin) {
                return false;
            }
        }
        true
    }
}

/// Free spots for stitching vias of `o.net`: a grid over the board and a row along the edge,
/// each spot free by [`ViaRoom`]; a blocked grid spot tries eight neighbours at a quarter pitch.
pub fn stitch_spots(b: &Board, zf: &[ZoneFill], o: &StitchOptions) -> Vec<Point> {
    let room = ViaRoom::new(b, zf, o.net, o.diameter, o.drill, o.margin, o.avoid_courtyards);
    let free = |p: Point, placed: &[Point]| room.free(p, placed);
    let mut bbox = Rect::empty();
    for p in &b.outline {
        let q = p.bbox();
        bbox.add(q.min);
        bbox.add(q.max);
    }
    let mut placed: Vec<Point> = Vec::new();
    if o.edge_pitch > 0.0 {
        for poly in &b.outline {
            let mut carry = 0.0;
            for (a, c) in poly.edges() {
                let len = a.dist(c);
                if len < 1e-9 {
                    continue;
                }
                let d = (c - a) * (1.0 / len);
                let nrm = pt(-d.y, d.x);
                let mut s = carry;
                while s < len {
                    let base = a + d * s;
                    for side in [1.0, -1.0] {
                        let p = base + nrm * (side * o.edge_inset);
                        if b.inside(p) {
                            // corners: the next edge starts where this one ended
                            let spaced = placed.iter().all(|q| q.dist(p) >= o.edge_pitch / 2.0);
                            if spaced && free(p, &placed) {
                                placed.push(p);
                            }
                            break;
                        }
                    }
                    s += o.edge_pitch;
                }
                carry = s - len;
            }
        }
    }
    if o.pitch > 0.0 && bbox.width() > 0.0 {
        let existing: Vec<Point> = b
            .obstacles
            .iter()
            .filter(|x| x.net == Some(o.net) && x.kind == ObstacleKind::Via)
            .map(|x| x.shape.bbox().center())
            .collect();
        let mut y = bbox.min.y + o.pitch / 2.0;
        while y < bbox.max.y {
            let mut x = bbox.min.x + o.pitch / 2.0;
            while x < bbox.max.x {
                let p = pt(x, y);
                let q = o.pitch / 4.0;
                let near_own = existing.iter().chain(placed.iter()).any(|e| e.dist(p) < o.pitch / 2.0);
                if !near_own {
                    let tries = [
                        pt(0.0, 0.0),
                        pt(q, 0.0),
                        pt(-q, 0.0),
                        pt(0.0, q),
                        pt(0.0, -q),
                        pt(q, q),
                        pt(-q, q),
                        pt(q, -q),
                        pt(-q, -q),
                    ];
                    if let Some(s) = tries.iter().map(|d| p + *d).find(|s| free(*s, &placed)) {
                        placed.push(s);
                    }
                }
                x += o.pitch;
            }
            y += o.pitch;
        }
    }
    placed
}

// --------------------------------------------------------------------------- power pours

/// Convex hull of points (counter clockwise in the math sense, clockwise on the y-down board).
pub fn convex_hull(pts: &[Point]) -> Vec<Point> {
    let mut p: Vec<Point> = pts.to_vec();
    p.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    p.dedup_by(|a, b| a.dist(*b) < 1e-9);
    if p.len() < 3 {
        return p;
    }
    let cross = |o: Point, a: Point, b: Point| (a - o).cross(b - o);
    let mut h: Vec<Point> = Vec::with_capacity(2 * p.len());
    for pass in 0..2 {
        let start = h.len();
        let it: Box<dyn Iterator<Item = &Point>> = if pass == 0 {
            Box::new(p.iter())
        } else {
            Box::new(p.iter().rev())
        };
        for &q in it {
            while h.len() >= start + 2 && cross(h[h.len() - 2], h[h.len() - 1], q) <= 1e-12 {
                h.pop();
            }
            h.push(q);
        }
        h.pop();
    }
    h
}

/// The part of a convex polygon with n·x <= h.
fn clip(poly: &[Point], n: Point, h: f64) -> Vec<Point> {
    let mut out = Vec::with_capacity(poly.len() + 1);
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        let (da, db) = (n.dot(a) - h, n.dot(b) - h);
        if da <= 0.0 {
            out.push(a);
        }
        if (da < 0.0 && db > 0.0) || (da > 0.0 && db < 0.0) {
            out.push(a.lerp(b, da / (da - db)));
        }
    }
    out
}

fn nearest_on(polys: &[Polygon], p: Point) -> Point {
    let mut best = (f64::INFINITY, p);
    for q in polys {
        for (a, b) in q.edges() {
            let x = seg_nearest(p, a, b);
            let d = x.dist(p);
            if d < best.0 {
                best = (d, x);
            }
        }
    }
    best.1
}

fn seg_nearest(p: Point, a: Point, b: Point) -> Point {
    let d = b - a;
    let l2 = d.len2();
    if l2 < 1e-18 {
        return a;
    }
    a + d * ((p - a).dot(d) / l2).clamp(0.0, 1.0)
}

/// A pour laid out before routing.
#[derive(Debug, Clone)]
pub struct Pour {
    pub outline: Vec<Point>,
    /// What cut the hull back: "pad U4.10 (net)", "board edge", ...
    pub cut_by: Vec<String>,
    /// Indices of the terminals the pour no longer touches.
    pub lost: Vec<usize>,
}

/// The outline of a pour of `net` on `layer` over the terminals (pad shapes, via disks as
/// polygons): their convex hull grown by `margin`, cut back by straight lines wherever copper,
/// holes, rule areas or reserved zones of other nets or the board edge come closer than their
/// clearance. Each cut runs across the line from the obstacle to the nearest point of the spine
/// (the lines between the terminal groups the pour joins; without spine: the nearest terminal),
/// so the pour gives way on the side of the obstacle only; a terminal it no longer touches is in
/// `lost`.
pub fn pour_outline(
    b: &Board,
    net: NetId,
    layer: LayerId,
    terminals: &[Polygon],
    spine: &[(Point, Point)],
    margin: f64,
) -> Option<Pour> {
    let anchor = |c: Point| -> Point {
        let on_spine = spine
            .iter()
            .map(|&(a, e)| seg_nearest(c, a, e))
            .min_by(|x, y| x.dist(c).total_cmp(&y.dist(c)));
        match on_spine {
            Some(t) if t.dist(c) > 1e-3 => t,
            _ => nearest_on(terminals, c),
        }
    };
    let pts: Vec<Point> = terminals.iter().flat_map(|t| t.pts.iter().copied()).collect();
    let hull = convex_hull(&pts);
    if hull.len() < 3 {
        return None;
    }
    let mut poly = Polygon::new(hull).grown(margin).pts;
    let own = b.clearance(Some(net), None);
    let mut cut_by = Vec::new();
    // obstacles grown by their clearance, nearest first (they cut the most)
    let mut obs: Vec<(f64, Polygon, String)> = Vec::new();
    let names: HashMap<usize, String> = b
        .pins
        .iter()
        .flat_map(|p| p.obstacles.iter().map(move |&o| (o, p.name.clone())))
        .collect();
    let bbox = Polygon::new(poly.clone()).bbox().grow(2.0);
    for (i, x) in b.obstacles.iter().enumerate() {
        if !x.shape.bbox().overlaps(&bbox) {
            continue;
        }
        let hole = x.hole.filter(|_| x.net != Some(net));
        let on = x.layers.has(layer) && x.net != Some(net);
        let gap = match x.kind {
            ObstacleKind::Keepout { tracks, mask, .. } => {
                if !tracks || mask || !on {
                    continue;
                }
                0.0
            }
            _ if on => x.clearance.max(own).max(b.item_gap(net, x.net)),
            // a hole of another net through the layer
            _ if hole.is_some() => b.rules.hole_clearance.max(own),
            _ => continue,
        };
        let shape = x.shape.to_polygon(0.01).grown(gap + 0.01);
        let what = match (&names.get(&i), x.kind) {
            (Some(n), _) => format!("pad {n}"),
            (None, ObstacleKind::Track) => "track".to_string(),
            (None, ObstacleKind::Via) => "via".to_string(),
            (None, ObstacleKind::Zone) => "zone".to_string(),
            (None, ObstacleKind::Pour) => "pour".to_string(),
            (None, ObstacleKind::Keepout { .. }) => "rule area".to_string(),
            (None, ObstacleKind::Pad) => "pad".to_string(),
            (None, ObstacleKind::Hole) => "hole".to_string(),
            (None, ObstacleKind::Graphic) => "copper graphic".to_string(),
        };
        let what = match x.net {
            Some(n) => format!("{what} ({})", b.nets[n as usize].name),
            None => what,
        };
        let c = shape.bbox().center();
        obs.push((anchor(c).dist(c), shape, what));
    }
    obs.sort_by(|a, b| a.0.total_cmp(&b.0));
    for (_, shape, what) in &obs {
        if poly.len() < 3 || Polygon::new(poly.clone()).dist_to_poly(shape) > 0.0 {
            continue;
        }
        let c = shape.bbox().center();
        let n = (c - anchor(c)).norm();
        if !n.x.is_finite() || n.len() < 0.5 {
            continue;
        }
        let h = shape.pts.iter().map(|q| n.dot(*q)).fold(f64::INFINITY, f64::min);
        poly = clip(&poly, n, h);
        cut_by.push(what.clone());
    }
    // the board edge: cut along every outline side the pour comes near, inside by the clearance
    let inner: Point = pts.iter().fold(pt(0.0, 0.0), |s, q| s + *q) * (1.0 / pts.len() as f64);
    for o in &b.outline {
        for (a, c) in o.edges() {
            if poly.len() < 3 || Polygon::new(poly.clone()).dist_to_seg(a, c) >= b.rules.edge_clearance {
                continue;
            }
            let d = (c - a).norm();
            let mut n = pt(-d.y, d.x);
            if n.dot(inner - a) > 0.0 {
                n = n * -1.0;
            }
            poly = clip(&poly, n, n.dot(a) - b.rules.edge_clearance);
            cut_by.push("board edge".to_string());
        }
    }
    if poly.len() < 3 {
        return None;
    }
    let out = Polygon::new(poly.clone());
    let lost = terminals
        .iter()
        .enumerate()
        .filter(|(_, t)| out.dist_to_poly(t) > 0.0)
        .map(|(i, _)| i)
        .collect();
    cut_by.dedup();
    Some(Pour {
        outline: poly,
        cut_by,
        lost,
    })
}

/// Spots for `count` vias of `net` next to a pad (its copper polygons): on rings around the pad,
/// the first one just off the pad (the hole stays out of the pad, a pour joins them), each spot
/// free by `room`. Fewer when there is no room.
pub fn via_field(room: &ViaRoom, pad: &[Polygon], count: usize, existing: &[Point]) -> Vec<Point> {
    let pts: Vec<Point> = pad.iter().flat_map(|t| t.pts.iter().copied()).collect();
    let hull = Polygon::new(convex_hull(&pts));
    let pitch = room.pitch();
    let mut placed: Vec<Point> = existing.to_vec();
    let start = placed.len();
    for ring in 0..4 {
        if placed.len() - start >= count {
            break;
        }
        let ring_poly = hull.grown(room.r + 0.1 + ring as f64 * pitch);
        for (a, c) in ring_poly.edges() {
            let len = a.dist(c);
            let steps = (len / pitch).floor().max(1.0) as usize;
            for k in 0..steps {
                let p = a.lerp(c, (k as f64 + 0.5) / steps as f64);
                if placed.len() - start < count && room.free(p, &placed) {
                    placed.push(p);
                }
            }
        }
    }
    placed.split_off(start)
}

#[cfg(test)]
mod tests {
    #[test]
    fn hull_and_clip() {
        let sq = [
            pt(0.0, 0.0),
            pt(2.0, 0.0),
            pt(2.0, 2.0),
            pt(0.0, 2.0),
            pt(1.0, 1.0),
            pt(1.0, 0.0),
        ];
        let h = convex_hull(&sq);
        assert_eq!(h.len(), 4);
        assert!((Polygon::new(h.clone()).signed_area().abs() - 4.0).abs() < 1e-9);
        // keep x <= 0.5
        let c = clip(&h, pt(1.0, 0.0), 0.5);
        assert!((Polygon::new(c).signed_area().abs() - 1.0).abs() < 1e-9);
    }

    use super::*;

    #[test]
    fn ipc_matches_the_lichtschnitzel_table() {
        // current-paths.md: 4.1 A outer 2 oz 0.69 mm, inner 1 oz 3.59 mm, 8 A outer 1.74 mm
        assert!((ipc_width(4.1, 0.07, true, 20.0) - 0.69).abs() < 0.01);
        assert!((ipc_width(4.1, 0.035, false, 20.0) - 3.59).abs() < 0.02);
        assert!((ipc_width(8.0, 0.07, true, 20.0) - 1.74).abs() < 0.01);
        let w = ipc_width(3.0, 0.035, true, 10.0);
        assert!((ipc_amps(w, 0.035, true, 10.0) - 3.0).abs() < 1e-9);
        // a 0.3 mm via carries a bit more than the 1 A the Lichtschnitzel layout assumes
        let v = via_amps(0.3, 20.0);
        assert!(v > 1.0 && v < 1.5, "{v}");
    }

    #[test]
    fn glob_patterns() {
        assert!(glob("*VBUS_P", "/Power/VBUS_P"));
        assert!(glob("*/SW_?", "/Power/SW_A"));
        assert!(!glob("*/SW_?", "/Power/SW_AB"));
        assert!(glob("GND", "GND"));
        assert!(!glob("GND", "AGND"));
    }

    #[test]
    fn edt_of_a_bar() {
        // 7 cells wide bar: the middle cell is 4 cells from the nearest outside cell
        let (nx, ny) = (20, 7);
        let m = vec![true; nx * ny];
        let d = edt(&m, nx, ny);
        assert!((d[3 * nx + 10] - 4.0).abs() < 1e-6);
        assert!((d[10] - 1.0).abs() < 1e-6);
    }
}

#[cfg(test)]
mod cut_tests {
    use super::*;

    #[test]
    fn min_cut_of_a_strip_is_its_width() {
        // 100 x 40 cells at 0.05 mm; copper band rows 10..24 (14 cells = 0.7 mm) on layer 0
        let g = Grid {
            x0: 0.0,
            y0: 0.0,
            step: 0.05,
            nx: 100,
            ny: 40,
        };
        let mut m = vec![false; g.n()];
        for r in 10..24 {
            for c in 0..100 {
                m[r * 100 + c] = true;
            }
        }
        let cu = vec![m];
        let src: Vec<(usize, usize)> = (10..24).map(|r| (0, r * 100)).collect();
        let dst: Vec<(usize, usize)> = (10..24).map(|r| (0, r * 100 + 99)).collect();
        let (w, at) = min_cut(
            &g,
            &[0],
            &cu,
            &[vec![false; 4000]],
            &[1.0],
            &HashMap::new(),
            &HashMap::new(),
            &src,
            &dst,
            2.0,
        );
        assert!((w - 0.7).abs() < 1e-6, "{w}");
        assert!(at.is_some());
    }
}
