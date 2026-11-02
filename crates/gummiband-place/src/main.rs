//! `gummiband-place`: alternative placements of the Lichtschnitzel controller (new outline, fixed
//! mechanical parts on the edge, the rest placed by simulated annealing, inner planes regenerated).

use std::collections::HashMap;
use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use clap::{Parser, ValueEnum};
use gummiband_core::geom::{Point, Polygon, pt};
use gummiband_kicad::place_write::{self, Patch, kicad_rot};
use gummiband_kicad::sexpr::Node;
use gummiband_kicad::{ReadOptions, read_board};

use gummiband_place::anneal::{Placer, Problem, Raster, Rng, State};
use gummiband_place::model::{Design, R};
use gummiband_place::planes;
use gummiband_place::variant::{self, FixedSet, Outline, Quad, Shape, Unit};

#[derive(Clone, Copy, ValueEnum)]
enum ShapeArg {
    Rect,
    Square,
    Circle,
}

#[derive(Parser)]
#[command(
    name = "gummiband-place",
    about = "Alternative placements of the Lichtschnitzel controller"
)]
struct Cli {
    /// Input board (placed, unrouted).
    board: PathBuf,
    #[arg(short, long)]
    output: PathBuf,
    #[arg(long, value_enum, default_value = "rect")]
    shape: ShapeArg,
    /// Rectangle width / square size in mm.
    #[arg(long, default_value_t = 85.0)]
    width: f64,
    /// Rectangle height in mm.
    #[arg(long, default_value_t = 55.0)]
    height: f64,
    /// Circle diameter in mm.
    #[arg(long, default_value_t = 90.0)]
    diameter: f64,
    #[arg(long, default_value_t = 1)]
    seed: u64,
    /// Annealing moves.
    #[arg(long, default_value_t = 3_000_000)]
    iters: usize,
    /// Spacing between courtyards in mm.
    #[arg(long, default_value_t = 0.2)]
    gap: f64,
    /// Courtyards of the movable parts keep this distance from the outline.
    #[arg(long, default_value_t = 0.5)]
    margin: f64,
    /// Plug whose PTC and output capacitor arrangement all plugs get.
    #[arg(long, default_value = "J202")]
    canonical_plug: String,
    /// Do not regenerate In2: drop its zones (those nets are then routed).
    #[arg(long)]
    drop_in2: bool,
}

const FIXED: &[&str] = &["J1", "U10", "H1", "H2", "H3", "H4"];
const RAIL_A: &[&str] = &["J200", "J201", "J202", "J203"];
const RAIL_B: &[&str] = &["J204", "J205", "J206", "J207"];
/// Power stages from the original layout (boards.py, SiC437 template): IC, inductor, input and
/// driver capacitors, feedback, boot and mode resistors are kept as rigid macros. The output
/// capacitors move on their own (pulled to the inductor output).
const BUCK_A: &[&str] = &[
    "U4", "L1", "C40", "C41", "C42", "C43", "C44", "C49", "C50", "R41", "R42", "R43", "R44", "R45",
];
const BUCK_B: &[&str] = &[
    "U5", "L2", "C60", "C61", "C62", "C63", "C64", "C69", "C70", "R61", "R62", "R63", "R64", "R65",
];

fn refs(d: &Design, names: &[&str]) -> Result<Vec<usize>> {
    names
        .iter()
        .map(|n| {
            d.by_ref
                .get(*n)
                .copied()
                .with_context(|| format!("part {n} not on the board"))
        })
        .collect()
}

/// Parts other than the plug that sit on the plug's 5 V output net (its PTC and output cap).
fn plug_attachments(d: &Design, plug: usize) -> Vec<usize> {
    let p = &d.parts[plug];
    let mut best: Vec<usize> = Vec::new();
    for pad in &p.pads {
        let Some(n) = pad.net else { continue };
        if d.nets[n] == "GND" {
            continue;
        }
        let mut others: Vec<usize> = Vec::new();
        for (i, q) in d.parts.iter().enumerate() {
            if i != plug && q.pads.iter().any(|x| x.net == Some(n)) && !others.contains(&i) {
                others.push(i);
            }
        }
        // the output net: only small passives (PTC F*, capacitor C*) on it
        if others.len() >= 2
            && others
                .iter()
                .all(|&i| d.parts[i].reference.starts_with('F') || d.parts[i].reference.starts_with('C'))
        {
            best = others;
        }
    }
    best.sort_by_key(|&i| d.parts[i].reference.clone());
    best
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let kb = read_board(&cli.board, &ReadOptions::default())?;
    let d = Design::from_root(&kb.root);
    let shape = match cli.shape {
        ShapeArg::Rect => Shape::Rect {
            w: cli.width,
            h: cli.height,
        },
        ShapeArg::Square => Shape::Rect {
            w: cli.width,
            h: cli.width,
        },
        ShapeArg::Circle => Shape::Circle { d: cli.diameter },
    };

    // ---- fixed parts and macros
    let fixed = refs(&d, FIXED)?;
    let canon = *d.by_ref.get(&cli.canonical_plug).context("canonical plug")?;
    let canon_att = plug_attachments(&d, canon);
    let canon_unit = Unit::anchored(&d, "canon", canon, &canon_att);
    let plug_unit = |plug: usize| -> Unit {
        let att = plug_attachments(&d, plug);
        let mut u = Unit::anchored(&d, &d.parts[plug].reference, plug, &att);
        // same arrangement as the canonical plug: match attachments by reference letter
        for m in u.members.iter_mut().skip(1) {
            let letter = d.parts[m.part].reference.chars().next();
            if let Some(cm) = canon_unit
                .members
                .iter()
                .skip(1)
                .find(|c| d.parts[c.part].reference.chars().next() == letter)
            {
                m.rel = cm.rel;
                m.rel_rot = cm.rel_rot;
            }
        }
        u
    };
    let rail_a: Vec<Unit> = refs(&d, RAIL_A)?.into_iter().map(plug_unit).collect();
    let rail_b: Vec<Unit> = refs(&d, RAIL_B)?.into_iter().map(plug_unit).collect();
    let fx = FixedSet {
        usb: fixed[0],
        module: fixed[1],
        holes: fixed[2..].to_vec(),
        rail_a: rail_a.clone(),
        rail_b: rail_b.clone(),
    };
    let arr = variant::arrange(&d, &fx, &shape, cli.gap.min(0.2))?;
    let outline = arr.outline;
    let mut is_fixed = vec![false; d.parts.len()];
    for u in &arr.units {
        for m in &u.members {
            is_fixed[m.part] = true;
        }
    }

    // ---- movable units
    let mut units: Vec<Unit> = Vec::new();
    let mut taken = is_fixed.clone();
    for (name, list) in [("buck A", BUCK_A), ("buck B", BUCK_B)] {
        let ps = refs(&d, list)?;
        if ps.iter().any(|&p| taken[p]) {
            bail!("{name} overlaps the fixed parts");
        }
        for &p in &ps {
            taken[p] = true;
        }
        units.push(Unit::rigid(&d, name, &ps));
    }
    for i in 0..d.parts.len() {
        if !taken[i] && !d.parts[i].court.is_empty() {
            units.push(Unit::single(&d, i));
        }
    }

    // ---- nets: weights by class and plane
    let class_of: HashMap<&str, &str> = kb
        .board
        .nets
        .iter()
        .map(|n| (n.name.as_str(), kb.board.classes[n.class as usize].name.as_str()))
        .collect();
    let zone_nets = |layer: &str| -> Vec<String> {
        kb.root
            .args()
            .iter()
            .filter(|z| z.head() == Some("zone") && z.child("keepout").is_none())
            .filter(|z| z.child_str("layer") == Some(layer))
            .filter_map(|z| z.child("net").and_then(|n| n.args().last()?.as_str()).map(String::from))
            .collect()
    };
    let in1_nets = zone_nets("In1.Cu");
    let in2_nets = zone_nets("In2.Cu");
    let weight = |n: usize| -> f64 {
        let name = d.nets[n].as_str();
        if name.starts_with("unconnected-") || in1_nets.iter().any(|x| x == name) {
            return 0.0;
        }
        if in2_nets.iter().any(|x| x == name) {
            return 1.0;
        }
        match class_of.get(name).copied().unwrap_or("Default") {
            "Input" | "USB90" | "Output" => 2.0,
            "Supply3V3" => 0.6,
            "LEDData" => 1.2,
            _ => 1.0,
        }
    };
    let mut fixed_world: HashMap<usize, (Point, f64)> = HashMap::new();
    for u in &arr.units {
        for m in &u.members {
            fixed_world.insert(m.part, u.member_pose(m));
        }
    }
    let mut fixed_pins = Vec::new();
    for (&pi, &(pos, rot)) in &fixed_world {
        for pad in &d.parts[pi].pads {
            if let Some(n) = pad.net {
                fixed_pins.push((n, pos + kicad_rot(pad.local, rot)));
            }
        }
    }
    fixed_pins.sort_by(|a, b| (a.0, a.1.x, a.1.y).partial_cmp(&(b.0, b.1.x, b.1.y)).unwrap());

    // ---- decoupling capacitors: nearest IC pin of the same net in the original layout
    let gnd = d.net_id("GND");
    let mut decap_pairs = Vec::new();
    let mut decap_report = Vec::new();
    for (ci, c) in d.parts.iter().enumerate() {
        if !c.reference.starts_with('C') || c.pads.len() != 2 || taken[ci] {
            continue;
        }
        let Some(gi) = c.pads.iter().position(|p| p.net.is_some() && p.net == gnd) else {
            continue;
        };
        let pi = 1 - gi;
        let Some(n) = c.pads[pi].net else { continue };
        let cp = c.pad_world(&c.pads[pi]);
        let mut best: Option<(f64, usize, usize)> = None;
        for (ui, u) in d.parts.iter().enumerate() {
            if !(u.reference.starts_with('U') || u.reference.starts_with('L')) {
                continue;
            }
            for (k, pad) in u.pads.iter().enumerate() {
                if pad.net == Some(n) {
                    let dd = u.pad_world(pad).dist(cp);
                    if best.is_none_or(|b| dd < b.0) {
                        best = Some((dd, ui, k));
                    }
                }
            }
        }
        if let Some((dd, ui, k)) = best
            && dd <= 10.0
        {
            let target = match fixed_world.get(&ui) {
                Some(&(pos, rot)) => Err(pos + kicad_rot(d.parts[ui].pads[k].local, rot)),
                None => Ok((ui, k)),
            };
            decap_pairs.push(((ci, pi), target, 3.0));
            decap_report.push(format!("{}->{}", c.reference, d.parts[ui].reference));
        }
    }

    // ---- problem
    let fixed_quads: Vec<Quad> = arr.units.iter().flat_map(|u| u.quads(&d)).collect();
    let raster = Raster::new(&outline, &fixed_quads, cli.margin, cli.gap, 0.1);
    let (prob, _part_pins) = Problem::new(&d, &units, &weight, &fixed_pins, &decap_pairs, raster, cli.gap);
    let ob = kb.board.bbox();
    let core = arr.core;
    let map = |p: Point| -> Point {
        pt(
            core.x0 + (p.x - ob.min.x) / ob.width() * core.w(),
            core.y0 + (p.y - ob.min.y) / ob.height() * core.h(),
        )
    };
    let state = State {
        pos: units.iter().map(|u| map(u.pos)).collect(),
        k: vec![0; units.len()],
    };
    let mut pl = Placer::new(&prob, state);
    let mut rng = Rng::new(cli.seed);
    let t = std::time::Instant::now();
    let size = core.w().max(core.h());
    println!(
        "{}: {} fixed parts, {} movable units ({} parts), {} decoupling pairs",
        outline.describe(),
        is_fixed.iter().filter(|&&f| f).count(),
        units.len(),
        units.iter().map(|u| u.members.len()).sum::<usize>(),
        decap_pairs.len()
    );
    {
        let b = outline.bbox();
        let free = b.area() - prob.raster.blocked(&b);
        let mov: f64 = units
            .iter()
            .flat_map(|u| u.members.iter())
            .map(|m| d.parts[m.part].court_area())
            .sum();
        println!(
            "free area {:.0} mm2, movable courtyards {:.0} mm2 ({:.0} % of the free area)",
            free,
            mov,
            100.0 * mov / free
        );
    }
    println!("start: wire {:.0}, overlap {:.1} mm2", pl.wire(), pl.total_overlap());
    pl.anneal(&mut rng, cli.iters, 30.0, 0.05, 0.3, 400.0, size / 3.0, 0.2, false);
    println!(
        "anneal: wire {:.0}, overlap {:.2} mm2 ({:.1} s)",
        pl.wire(),
        pl.total_overlap(),
        t.elapsed().as_secs_f64()
    );
    let failed = pl.legalize(0.25, 40.0).len();
    println!(
        "legalize: wire {:.0}, overlap {:.3} mm2, {} units without a spot",
        pl.wire(),
        pl.total_overlap(),
        failed
    );
    for u in 0..units.len() {
        let o = pl.overlap(u);
        if o > 1e-9 {
            let others: Vec<String> = (0..units.len())
                .filter(|&v| v != u && pl.pair_overlap(u, v) > 0.0)
                .map(|v| units[v].name.clone())
                .collect();
            println!(
                "  overlap {}: {:.3} (fixed {:.3}) with {:?} at {:?} k {}",
                units[u].name,
                o,
                pl.fixed_overlap(u),
                others,
                pl.s.pos[u],
                pl.s.k[u]
            );
        }
    }
    pl.anneal(&mut rng, cli.iters / 3, 0.8, 0.01, 0.0, 0.0, 2.0, 0.1, true);
    println!(
        "polish: wire {:.0}, overlap {:.3} mm2 ({:.1} s)",
        pl.wire(),
        pl.total_overlap(),
        t.elapsed().as_secs_f64()
    );
    if failed > 0 || pl.total_overlap() > 1e-6 {
        bail!("no legal placement: the outline is too small");
    }

    // ---- final poses
    let mut pose: Vec<(Point, f64)> = d.parts.iter().map(|p| (p.pos, p.rot)).collect();
    for u in &arr.units {
        for m in &u.members {
            pose[m.part] = u.member_pose(m);
        }
    }
    for (ui, u) in units.iter().enumerate() {
        let mut t = u.clone();
        t.pos = pl.s.pos[ui];
        t.rot = u.rot + 90.0 * pl.s.k[ui] as f64;
        for m in &t.members {
            pose[m.part] = t.member_pose(m);
        }
    }
    let pose: Vec<(Point, f64)> = pose
        .into_iter()
        .map(|(p, r)| ((p * 1e4).round_xy() * 1e-4, place_write::norm_angle(r)))
        .collect();

    // ---- exact check: courtyard separation, movable courtyards inside the outline with margin
    let quads: Vec<(usize, Quad)> = d
        .parts
        .iter()
        .enumerate()
        .flat_map(|(i, p)| {
            let (pos, rot) = pose[i];
            p.court.iter().map(move |r| (i, Quad { pos, rot, r: *r }))
        })
        .collect();
    let mut min_sep = (f64::INFINITY, String::new());
    for a in 0..quads.len() {
        for b in a + 1..quads.len() {
            if quads[a].0 == quads[b].0 {
                continue;
            }
            if !quads[a]
                .1
                .bbox()
                .grow(1.0)
                .union(&R::empty())
                .overlaps_r(&quads[b].1.bbox())
            {
                continue;
            }
            let s = quads[a].1.separation(&quads[b].1);
            if s < min_sep.0 {
                min_sep = (
                    s,
                    format!("{}-{}", d.parts[quads[a].0].reference, d.parts[quads[b].0].reference),
                );
            }
        }
    }
    let mut outside = Vec::new();
    for (i, q) in &quads {
        if is_fixed[*i] {
            continue;
        }
        if q.corners().iter().any(|c| !outline.inside(*c, cli.margin - 1e-6)) {
            outside.push(d.parts[*i].reference.clone());
        }
    }
    println!(
        "check: smallest courtyard separation {:.3} mm ({}), outside the margin: {:?}",
        min_sep.0, min_sep.1, outside
    );
    if min_sep.0 < 0.0 || !outside.is_empty() {
        bail!("placement check failed");
    }
    let court_area: f64 = d.parts.iter().map(|p| p.court_area()).sum();
    println!(
        "board area {:.0} mm2, courtyards {:.0} mm2 ({:.0} %, antenna keepout included)",
        outline.area(),
        court_area,
        100.0 * court_area / outline.area()
    );

    // ---- part transforms (original -> new)
    let xf = |i: usize| {
        let (p0, r0) = (d.parts[i].pos, d.parts[i].rot);
        let (p1, r1) = pose[i];
        move |q: Point| p1 + kicad_rot(q - p0, r1 - r0)
    };
    let orig_quads: Vec<(usize, Quad)> = d
        .parts
        .iter()
        .enumerate()
        .flat_map(|(i, p)| {
            p.court.iter().map(move |r| {
                (
                    i,
                    Quad {
                        pos: p.pos,
                        rot: p.rot,
                        r: *r,
                    },
                )
            })
        })
        .collect();
    let owner_at = |c: Point, grow: f64| -> Option<usize> {
        // the part whose courtyard holds `c`; the largest one wins (plug labels, antenna keepout)
        orig_quads
            .iter()
            .filter(|(_, q)| q.contains(c, grow))
            .max_by(|a, b| a.1.r.area().partial_cmp(&b.1.r.area()).unwrap())
            .map(|(i, _)| *i)
    };

    // ---- inner planes
    let mut keepouts: Vec<Polygon> = Vec::new();
    for (i, p) in d.parts.iter().enumerate() {
        for (pts, layers) in &p.keepouts {
            if layers.iter().any(|l| l == "In2.Cu" || l == "*.Cu" || l == "*.In.Cu") {
                let f = xf(i);
                keepouts.push(Polygon::new(pts.iter().map(|&q| f(q)).collect()));
            }
        }
    }
    let in2_pins: Vec<Vec<Point>> = in2_nets
        .iter()
        .map(|name| {
            let mut v = Vec::new();
            for (i, p) in d.parts.iter().enumerate() {
                for pad in &p.pads {
                    if pad.net.map(|n| d.nets[n].as_str()) == Some(name.as_str()) {
                        v.push(pose[i].0 + kicad_rot(pad.local, pose[i].1));
                    }
                }
            }
            v
        })
        .collect();
    let split = planes::split(&outline, &in2_pins, &keepouts, 1.0, 2, 0.25);
    for n in &split.notes {
        let mut t = n.clone();
        for (i, name) in in2_nets.iter().enumerate().rev() {
            t = t.replace(&format!("net {i}"), name);
        }
        println!("In2: {t}");
    }
    for (name, r) in in2_nets.iter().zip(&split.regions) {
        match r {
            Some(p) if !cli.drop_in2 => {
                let a = Polygon::new(p.clone()).signed_area().abs();
                println!("In2 {name}: {:.0} mm2, {} vertices", a, p.len());
            }
            _ => println!("In2 {name}: no plane (routed)"),
        }
    }

    // ---- write
    let mut patch = Patch::new(cli.seed ^ 0x1234);
    for (i, p) in d.parts.iter().enumerate() {
        let (pos, rot) = pose[i];
        if pos.dist(p.pos) > 1e-6 || (place_write::norm_angle(rot - p.rot)).abs() > 1e-6 {
            patch.move_footprint(&kb.root.args()[p.node], pos, rot);
        }
    }
    for n in kb.root.args() {
        match n.head() {
            Some("zone") => {
                let layer = n.child_str("layer");
                if n.child("keepout").is_some() {
                    let pts: Vec<Point> = zone_pts(n);
                    let mut zb = R::empty();
                    for p in &pts {
                        zb = zb.union(&R {
                            x0: p.x,
                            y0: p.y,
                            x1: p.x,
                            y1: p.y,
                        });
                    }
                    let owner = orig_quads
                        .iter()
                        .map(|(i, q)| (*i, q.bbox().overlap(&zb)))
                        .filter(|x| x.1 > 0.0)
                        .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                        .map(|x| x.0);
                    match owner {
                        Some(i) => {
                            let f = xf(i);
                            patch.transform_zone(n, &f);
                            println!(
                                "keepout {} follows {}",
                                n.child_str("name").unwrap_or("?"),
                                d.parts[i].reference
                            );
                        }
                        None => println!("keepout {} stays", n.child_str("name").unwrap_or("?")),
                    }
                } else if layer == Some("In1.Cu") {
                    patch.replace_zone_outline(n, &outline.polygon());
                } else if layer == Some("In2.Cu") {
                    let name = n.child("net").and_then(|x| x.args().last()?.as_str()).unwrap_or("");
                    let k = in2_nets.iter().position(|x| x == name);
                    match k.and_then(|k| split.regions[k].as_ref()) {
                        Some(p) if !cli.drop_in2 => patch.replace_zone_outline(n, p),
                        _ => patch.remove(n),
                    }
                }
            }
            Some("gr_line") | Some("gr_rect") | Some("gr_circle") | Some("gr_arc") | Some("gr_poly")
                if n.child_str("layer") == Some("Edge.Cuts") =>
            {
                patch.remove(n);
            }
            Some("gr_text") => {
                if let Some(at) = n.child_f64s("at")
                    && at.len() >= 2
                    && let Some(i) = owner_at(pt(at[0], at[1]), 2.0)
                {
                    let f = xf(i);
                    patch.transform_text(n, &f, pose[i].1 - d.parts[i].rot);
                }
            }
            _ => {}
        }
    }
    let edge = match outline {
        Outline::Rect { x0, y0, x1, y1 } => place_write::edge_rect(&mut patch, pt(x0, y0), pt(x1, y1)),
        Outline::Circle { c, r } => place_write::edge_circle(&mut patch, c, r),
    };
    patch.append(edge);
    let text = patch.apply(&kb.text, &kb.root);
    if let Some(dir) = cli.output.parent() {
        std::fs::create_dir_all(dir)?;
    }
    std::fs::write(&cli.output, text)?;
    for ext in ["kicad_pro", "kicad_dru"] {
        let src = cli.board.with_extension(ext);
        if src.exists() {
            std::fs::copy(&src, cli.output.with_extension(ext))?;
        }
    }
    println!("decoupling pairs: {}", decap_report.join(" "));
    println!("wrote {}", cli.output.display());
    Ok(())
}

fn zone_pts(z: &Node) -> Vec<Point> {
    z.child("polygon")
        .and_then(|p| p.child("pts"))
        .map(|p| {
            p.children("xy")
                .filter_map(|n| {
                    let a = n.args();
                    Some(pt(a.first()?.as_f64()?, a.get(1)?.as_f64()?))
                })
                .collect()
        })
        .unwrap_or_default()
}

trait RoundXy {
    fn round_xy(self) -> Point;
}
impl RoundXy for Point {
    fn round_xy(self) -> Point {
        pt(self.x.round(), self.y.round())
    }
}

trait Overlaps {
    fn overlaps_r(&self, o: &R) -> bool;
}
impl Overlaps for R {
    fn overlaps_r(&self, o: &R) -> bool {
        self.x0 <= o.x1 && o.x0 <= self.x1 && self.y0 <= o.y1 && o.y0 <= self.y1
    }
}
