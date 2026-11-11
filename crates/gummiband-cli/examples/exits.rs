//! How the tracks of a routed board leave the pads of tight rows (`gummiband_core::exit`).
//! Routes the board with the default settings (parts stay where they are unless `--move-parts`
//! is given) and writes one JSON object: the result (routed,
//! vias, length, CPU time, the own check in pieces and spots) and, for every track end at a pad
//! that the exit rule covers, where it leaves the pad and how near it comes to the next pads.
//! `GB_EXITS=0` routes without exits; the pads and their exit points are then those the rule
//! would lay, so that both runs are measured alike.
//! Usage: exits BOARD OUT.json [--seed N] [--open-end SECONDS] [--move-parts] [--board OUT.kicad_pcb]
//!        [--runs N] [--generations N] [--geometry-generations N]
use std::path::Path;
use std::time::Instant;

use gummiband_app::{Hooks, RouteOptions, route_file};
use gummiband_core::board::Board;
use gummiband_core::exit;
use gummiband_core::geom::{Point, Shape, seg_seg_dist};
use gummiband_core::realize::{Elem, Geometry, Track, flatten};
use serde_json::json;

#[repr(C)]
#[derive(Default)]
struct RUsage {
    utime: [i64; 2],
    stime: [i64; 2],
    rest: [i64; 14],
}

unsafe extern "C" {
    fn getrusage(who: i32, usage: *mut RUsage) -> i32;
}

fn cpu_seconds() -> f64 {
    let mut u = RUsage::default();
    unsafe { getrusage(0, &mut u) };
    (u.utime[0] + u.stime[0]) as f64 + (u.utime[1] + u.stime[1]) as f64 * 1e-6
}

/// A way from a pin: points, and for each piece between them its width and whether a new line
/// or arc of the file begins there.
struct Way {
    pts: Vec<Point>,
    widths: Vec<f64>,
    /// Lengths of the lines and arcs, in order.
    elems: Vec<f64>,
}

fn way_of(t: &Track, from_end: bool) -> Way {
    let mut w = Way {
        pts: Vec::new(),
        widths: Vec::new(),
        elems: Vec::new(),
    };
    let n = t.elems.len();
    for k in 0..n {
        let e = t.elems[if from_end { n - 1 - k } else { k }];
        let (a, b) = match e {
            Elem::Line(a, b) | Elem::Arc(a, _, b) => (a, b),
        };
        let mut pts = flatten(std::slice::from_ref(&e));
        if from_end {
            pts.reverse();
        }
        w.elems.push(pts.windows(2).map(|x| x[0].dist(x[1])).sum());
        let width = t.piece_width(a, b);
        if w.pts.is_empty() {
            w.pts.push(pts[0]);
        }
        for &p in &pts[1..] {
            w.pts.push(p);
            w.widths.push(width);
        }
    }
    w
}

/// The way from `pin` along the track that starts there, and on along the track that goes on
/// from the end of an exit piece.
fn ways_from(geo: &Geometry, net: u32, layer: u8, pin: Point, exits: &[Point]) -> Vec<Way> {
    let at = |t: &Track, p: Point| -> Option<bool> {
        if t.net != net || t.layer != layer || t.pts.len() < 2 {
            None
        } else if t.pts[0].dist(p) < 1e-6 {
            Some(false)
        } else if t.pts[t.pts.len() - 1].dist(p) < 1e-6 {
            Some(true)
        } else {
            None
        }
    };
    let mut out = Vec::new();
    for (ti, t) in geo.tracks.iter().enumerate() {
        let Some(from_end) = at(t, pin) else { continue };
        let mut w = way_of(t, from_end);
        let last = *w.pts.last().unwrap();
        if t.pts.len() == 2 && exits.iter().any(|e| e.dist(last) < 1e-6) {
            let next = geo
                .tracks
                .iter()
                .enumerate()
                .find_map(|(tj, u)| (tj != ti).then(|| at(u, last).map(|f| way_of(u, f))).flatten());
            if let Some(n) = next {
                w.pts.extend_from_slice(&n.pts[1..]);
                w.widths.extend(n.widths);
                w.elems.extend(n.elems);
            }
        }
        out.push(w);
    }
    out
}

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let flag = |name: &str| {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1).cloned())
    };
    let d = RouteOptions::default();
    let opts = RouteOptions {
        seed: Some(flag("--seed").map_or(Ok(1), |s| s.parse())?),
        runs: flag("--runs").map_or(Ok(d.runs), |s| s.parse())?,
        generations: flag("--generations").map_or(Ok(d.generations), |s| s.parse())?,
        geometry_generations: flag("--geometry-generations").map_or(Ok(d.geometry_generations), |s| s.parse())?,
        move_parts: if args.iter().any(|a| a == "--move-parts") {
            d.move_parts
        } else {
            Some(false)
        },
        open_end_seconds: flag("--open-end").map_or(Ok(d.open_end_seconds), |s| s.parse())?,
        ..d
    };
    let board = Path::new(&args[1]);
    let name = board.file_stem().unwrap().to_string_lossy().to_string();
    let scratch = std::env::temp_dir().join(format!("exits-{}-{name}.kicad_pcb", std::process::id()));
    let out = flag("--board").map_or(scratch.clone(), |p| p.into());
    let (t0, c0) = (Instant::now(), cpu_seconds());
    let outcome = route_file(board, &out, &opts, &Hooks::default())?;
    let (wall, cpu) = (t0.elapsed().as_secs_f64(), cpu_seconds() - c0);
    if flag("--board").is_some() {
        outcome.write()?;
    }
    let routed = &outcome.routed;
    let b: &Board = &routed.board;
    let geo = &routed.res.geometry;
    let st = &routed.res.stats;
    let clear: Vec<_> = routed.violations.iter().filter(|v| !v.is_width()).cloned().collect();
    let spots = gummiband_core::check::spots(&clear);
    let short = geo
        .tracks
        .iter()
        .flat_map(|t| t.elems.iter())
        .filter(|e| {
            flatten(std::slice::from_ref(*e))
                .windows(2)
                .map(|x| x[0].dist(x[1]))
                .sum::<f64>()
                < 0.05
        })
        .count();

    let plans = exit::plan(b);
    let mut ends = Vec::new();
    for p in &plans {
        let pin = &b.pins[p.pin];
        let Shape::Polygon(pad) = b.bare_shape(p.obs) else {
            continue;
        };
        let layer = pin.layers.iter().find(|&l| b.routing_layers().has(l)).unwrap();
        let f = p.frame;
        let points: Vec<Point> = p.exits.iter().map(|e| e.at).collect();
        // the middle of the part, to tell the way out from the way in under the part
        let mates: Vec<Point> = b
            .pins
            .iter()
            .filter(|q| q.footprint == pin.footprint)
            .map(|q| q.pos)
            .collect();
        let centre = mates.iter().fold(Point::default(), |a, &q| a + q) * (1.0 / mates.len() as f64);
        for w in ways_from(geo, pin.net, layer, pin.pos, &points) {
            // where the way leaves the pad: the first point outside, in steps of 2 µm
            let mut left = None;
            'walk: for s in w.pts.windows(2) {
                let n = (s[0].dist(s[1]) / 0.002).ceil().max(1.0) as usize;
                for k in 1..=n {
                    let q = s[0].lerp(s[1], k as f64 / n as f64);
                    if !pad.contains(q) {
                        left = Some((q, (s[1] - s[0]).norm()));
                        break 'walk;
                    }
                }
            }
            let Some((q, heading)) = left else {
                ends.push(json!({"pin": pin.name, "left": "stays on the pad"}));
                continue;
            };
            let (along, across) = ((q - pin.pos).dot(f.u), (q - pin.pos).dot(f.u.perp()));
            let (side, dir, len) = if along >= f.fwd - 3e-3 {
                ("end", f.u, f.fwd)
            } else if along <= -f.back + 3e-3 {
                ("end", f.u * -1.0, f.back)
            } else if across.abs() >= f.half - 3e-3 {
                (
                    "side",
                    if along >= 0.0 { f.u } else { f.u * -1.0 },
                    if along >= 0.0 { f.fwd } else { f.back },
                )
            } else {
                (
                    "corner",
                    if along >= 0.0 { f.u } else { f.u * -1.0 },
                    if along >= 0.0 { f.fwd } else { f.back },
                )
            };
            let planned = p.exits.iter().find(|e| e.dir.dot(dir) > 0.5);
            // how far beyond the end of the pad the exit point lies (0.1 where none is laid)
            let y = planned.map_or(0.1, |e| (e.at - pin.pos).dot(dir) - len);
            let reach = len + y;
            // sideways offset where the way passes distance d along the middle line, and the
            // angle to it there
            let pass = |d: f64| -> Option<(f64, f64)> {
                w.pts.windows(2).find_map(|s| {
                    let (a, c) = ((s[0] - pin.pos).dot(dir), (s[1] - pin.pos).dot(dir));
                    (a <= d && c > d).then(|| {
                        let x = s[0] + (s[1] - s[0]) * ((d - a) / (c - a));
                        let h = (s[1] - s[0]).norm();
                        (
                            dir.cross(x - pin.pos).abs(),
                            gummiband_core::math::asin(dir.cross(h).abs().min(1.0)).to_degrees(),
                        )
                    })
                })
            };
            // the largest offset from the middle line up to the exit point
            let mut off_max: f64 = 0.0;
            let mut gone = 0.0;
            for s in w.pts.windows(2) {
                let (a, c) = ((s[0] - pin.pos).dot(dir), (s[1] - pin.pos).dot(dir));
                for (x, d) in [(s[0], a), (s[1], c)] {
                    if d <= reach + 1e-9 {
                        off_max = off_max.max(dir.cross(x - pin.pos).abs());
                    }
                }
                if a <= reach && c > reach {
                    let x = s[0] + (s[1] - s[0]) * ((reach - a) / (c - a));
                    off_max = off_max.max(dir.cross(x - pin.pos).abs());
                }
                // up to where the way first comes as far out as the exit point
                gone += s[0].dist(s[1]);
                if c >= reach - 1e-9 || gone > reach + 2.0 {
                    break;
                }
            }
            // room left to the pads beside this one, after the clearance: on the way up to the
            // exit point, and on the 0.5 mm after it
            let (mut near_piece, mut near_after) = (f64::INFINITY, f64::INFINITY);
            let mut gone = 0.0;
            for (s, &width) in w.pts.windows(2).zip(&w.widths) {
                for &q in &p.beside {
                    let Some(c) = exit::gap_to(b, pin.net, &b.obstacles[q]) else {
                        continue;
                    };
                    let room = b.bare_shape(q).dist_to_seg(s[0], s[1]) - width / 2.0 - c;
                    if gone < reach - 1e-6 {
                        near_piece = near_piece.min(room);
                    } else {
                        near_after = near_after.min(room);
                    }
                }
                gone += s[0].dist(s[1]);
                if gone > reach + 0.5 {
                    break;
                }
            }
            // a short way that ends on another pad of the net: two pads joined directly
            let total: f64 = w.pts.windows(2).map(|s| s[0].dist(s[1])).sum();
            let last = *w.pts.last().unwrap();
            let joins_pad = total < 2.0
                && b.pins.iter().enumerate().any(|(q, other)| {
                    q != p.pin
                        && other.net == pin.net
                        && other
                            .obstacles
                            .iter()
                            .any(|&o| b.bare_shape(o).dist_to_point(last) <= 0.0)
                });
            // lines and arcs that begin before the exit point or within 0.15 mm after it
            let mut begun = 0.0;
            let mut pieces = Vec::new();
            for &l in &w.elems {
                if begun > reach + 0.15 {
                    break;
                }
                pieces.push(l);
                begun += l;
            }
            // would the straight exit do here: the line to the exit point, and from there to
            // where the way is 0.3 mm further on; what is in its way
            let mut trial = json!(null);
            if let Some(e) = planned {
                let mut gone = 0.0;
                let mut join = *w.pts.last().unwrap();
                for s in w.pts.windows(2) {
                    gone += s[0].dist(s[1]);
                    if gone >= reach + 0.3 {
                        join = s[1];
                        break;
                    }
                }
                let legs = [(pin.pos, e.at), (e.at, join)];
                let half = p.track_width / 2.0;
                let mut worst = [f64::INFINITY; 4];
                for (i, &(x, z)) in legs.iter().enumerate() {
                    if i == 1 {
                        for (q, o) in b.obstacles.iter().enumerate() {
                            if !o.layers.has(layer) {
                                continue;
                            }
                            let Some(c) = exit::gap_to(b, pin.net, o) else { continue };
                            let room = b.bare_shape(q).dist_to_seg(x, z) - half - c;
                            let k = if p.beside.contains(&q) { 0 } else { 1 };
                            worst[k] = worst[k].min(room);
                        }
                    }
                    for t in geo.tracks.iter().filter(|t| t.net != pin.net && t.layer == layer) {
                        let c = b.clearance(Some(pin.net), Some(t.net));
                        for s in t.pts.windows(2) {
                            let room = seg_seg_dist(x, z, s[0], s[1]) - half - t.width / 2.0 - c;
                            worst[2] = worst[2].min(room);
                        }
                    }
                    for v in geo.vias.iter().filter(|v| v.net != pin.net) {
                        let c = b.clearance(Some(pin.net), Some(v.net));
                        let room = gummiband_core::geom::point_seg_dist(v.at, x, z) - half - v.diameter / 2.0 - c;
                        worst[3] = worst[3].min(room);
                    }
                }
                let turn = gummiband_core::math::acos(((join - e.at).norm().dot(dir)).clamp(-1.0, 1.0)).to_degrees();
                trial = json!({"row_pad": worst[0], "other_copper": worst[1], "track": worst[2], "via": worst[3], "turn_deg": turn});
            }
            ends.push(json!({
                "pin": pin.name,
                "pin_at": [pin.pos.x, pin.pos.y],
                "left": side,
                "outward": dir.dot(pin.pos - centre),
                "joins_pad": joins_pad,
                "has_exit": planned.is_some(),
                "heading_deg": gummiband_core::math::acos(heading.dot(dir).clamp(-1.0, 1.0)).to_degrees(),
                "y": y,
                "spare": p.spare,
                "off_max": off_max,
                "at_pad_end": pass(len - 1e-6).map(|x| json!({"off": x.0, "angle": x.1})),
                "at_exit": pass(reach - 1e-6).map(|x| json!({"off": x.0, "angle": x.1})),
                "near_piece": if near_piece.is_finite() { json!(near_piece) } else { json!(null) },
                "near_after": if near_after.is_finite() { json!(near_after) } else { json!(null) },
                "pieces": pieces,
                "way": w.pts.iter().take(10).map(|q| [q.x, q.y]).collect::<Vec<_>>(),
                "trial": trial,
            }));
        }
    }
    let report = json!({
        "board": name,
        "seed": routed.seed,
        "exits_on": gummiband_core::knobs::EXITS.on(),
        "wall_s": wall,
        "cpu_s": cpu,
        "conns": st.conns,
        "routed": st.routed,
        "vias": st.vias,
        "length": st.length,
        "violations": clear.len(),
        "spots": spots.len(),
        "shorts": spots.iter().filter(|s| s.short).count(),
        "width_findings": routed.violations.len() - clear.len(),
        "pieces_under_50um": short,
        "tracks": geo.tracks.len(),
        "pads": plans.len(),
        "pads_with_exit": plans.iter().filter(|p| !p.exits.is_empty()).count(),
        "pads_two_exits": plans.iter().filter(|p| p.exits.len() == 2).count(),
        "exit_y": plans.iter().flat_map(|p| p.exits.iter().map(|e| {
            let pin = &b.pins[p.pin];
            let len = if e.dir.dot(p.frame.u) > 0.0 { p.frame.fwd } else { p.frame.back };
            (e.at - pin.pos).dot(e.dir) - len
        })).collect::<Vec<f64>>(),
        "spare": plans.iter().map(|p| p.spare).collect::<Vec<f64>>(),
        "tight_pads": routed.tight_pads(),
        "violation_list": clear.iter().map(|v| json!({
            "what": v.what, "at": [v.at.x, v.at.y], "actual": v.actual, "required": v.required,
            "nets": [b.nets[v.nets.0 as usize].name.clone(), v.nets.1.map_or(String::new(), |n| b.nets[n as usize].name.clone())],
        })).collect::<Vec<_>>(),
        "ends": ends,
    });
    std::fs::write(&args[2], serde_json::to_string(&report)?)?;
    let _ = std::fs::remove_file(&scratch);
    println!(
        "{name} seed {}: {}/{} routed, {} violations at {} spots, {} vias, {:.0} mm, {:.1} s cpu, {} ends at {} pads",
        routed.seed,
        st.routed,
        st.conns,
        clear.len(),
        spots.len(),
        st.vias,
        st.length,
        cpu,
        report["ends"].as_array().map_or(0, |a| a.len()),
        plans.len()
    );
    Ok(())
}
