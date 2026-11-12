//! The band of a track with the radius along the edge against the band with the radius of the
//! stack (`GB_BAND`, `Router::stack_band`), on one topology.
//! Routes the board (parts stay where they are) and writes the result of the pipeline as it
//! ran, with the knob as the environment sets it. Then it builds the router of the winner again
//! and realises its topology both ways: how many tracks have a band, which wraps gave way and
//! why, the own check of the two constructions, and what one relaxation with a given number of
//! rounds leaves of each.
//! Usage: band_stage BOARD OUT.json [--seed N] [--runs N] [--generations N]
//!        [--geometry-generations N] [--rounds 100,200,400]
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use std::sync::atomic::Ordering;
use std::time::Instant;

use gummiband_app::{Hooks, RouteOptions, route_file};
use gummiband_core::board::Board;
use gummiband_core::check::{self, Violation};
use gummiband_core::geom::{Point, point_seg_dist};
use gummiband_core::realize::{Elem, Geometry, PathInfo, flatten};
use gummiband_core::relax::{RelaxConfig, relax};
use serde_json::{Value, json};

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

fn elem_len(e: &Elem) -> f64 {
    flatten(std::slice::from_ref(e))
        .windows(2)
        .map(|x| x[0].dist(x[1]))
        .sum()
}

/// Lines, arcs, pieces under 50 µm and the length of a geometry.
fn shape(geo: &Geometry) -> Value {
    let (mut lines, mut arcs, mut short, mut length) = (0, 0, 0, 0.0);
    for e in geo.tracks.iter().flat_map(|t| t.elems.iter()) {
        match e {
            Elem::Line(..) => lines += 1,
            Elem::Arc(..) => arcs += 1,
        }
        let l = elem_len(e);
        length += l;
        if l < 0.05 {
            short += 1;
        }
    }
    json!({"tracks": geo.tracks.len(), "lines": lines, "arcs": arcs, "pieces_under_50um": short, "length": length})
}

/// Pairs of tracks of different nets on one layer whose centre lines cross.
fn crossings(geo: &Geometry) -> Vec<(usize, usize, Point)> {
    let boxes: Vec<(Point, Point)> = geo
        .tracks
        .iter()
        .map(|t| {
            let lo = t
                .pts
                .iter()
                .fold(gummiband_core::geom::pt(f64::INFINITY, f64::INFINITY), |a, p| {
                    gummiband_core::geom::pt(a.x.min(p.x), a.y.min(p.y))
                });
            let hi = t.pts.iter().fold(
                gummiband_core::geom::pt(f64::NEG_INFINITY, f64::NEG_INFINITY),
                |a, p| gummiband_core::geom::pt(a.x.max(p.x), a.y.max(p.y)),
            );
            (lo, hi)
        })
        .collect();
    let mut out = Vec::new();
    for (i, a) in geo.tracks.iter().enumerate() {
        for (j, b) in geo.tracks.iter().enumerate().skip(i + 1) {
            if a.net == b.net || a.layer != b.layer {
                continue;
            }
            let ((alo, ahi), (blo, bhi)) = (boxes[i], boxes[j]);
            if alo.x > bhi.x || blo.x > ahi.x || alo.y > bhi.y || blo.y > ahi.y {
                continue;
            }
            let hit = a.pts.windows(2).find_map(|p| {
                b.pts
                    .windows(2)
                    .any(|q| gummiband_core::geom::segs_intersect(p[0], p[1], q[0], q[1]))
                    .then_some(p[0])
            });
            if let Some(at) = hit {
                out.push((i, j, at));
            }
        }
    }
    out
}

fn clearance(board: &Board, geo: &Geometry) -> Vec<Violation> {
    check::check(board, geo).into_iter().filter(|v| !v.is_width()).collect()
}

fn missing_band(d: f64) -> &'static str {
    if d < 0.005 {
        "<5"
    } else if d < 0.02 {
        "5-20"
    } else if d < 0.05 {
        "20-50"
    } else {
        ">50"
    }
}

/// The own check of a geometry in pieces and spots.
fn counts(v: &[Violation]) -> Value {
    let spots = check::spots(v);
    json!({"pieces": v.len(), "spots": spots.len(), "shorts": spots.iter().filter(|s| s.short).count()})
}

/// How the band of a track came about, as a class: no bend, no wrap gave way, some gave way
/// and some are left, none is left, then the funnel polyline by its reason.
fn class_of(i: &PathInfo) -> &'static str {
    match i.fallback {
        0 if i.bends == 0 => "straight",
        0 if i.gave_way.iter().sum::<usize>() == 0 => "band",
        0 if i.wraps > 0 => "band with corners",
        0 => "corners only",
        1 => "polyline: no tangent",
        2 => "polyline: leaves triangles",
        3 => "polyline: misses portal",
        _ => "polyline: arcs off",
    }
}

/// The classes from the best to the worst, for the worse track of a pair.
const CLASSES: [&str; 8] = [
    "straight",
    "band",
    "band with corners",
    "corners only",
    "polyline: no tangent",
    "polyline: leaves triangles",
    "polyline: misses portal",
    "polyline: arcs off",
];

/// The element of a track of `net` nearest to `at`: (track, element, distance).
fn nearest(geo: &Geometry, net: u32, at: Point) -> Option<(usize, usize, f64)> {
    let mut best: Option<(usize, usize, f64)> = None;
    for (ti, t) in geo.tracks.iter().enumerate().filter(|(_, t)| t.net == net) {
        for (ei, e) in t.elems.iter().enumerate() {
            let d = flatten(std::slice::from_ref(e))
                .windows(2)
                .map(|w| point_seg_dist(at, w[0], w[1]))
                .fold(f64::INFINITY, f64::min);
            if best.is_none_or(|b| d < b.2) {
                best = Some((ti, ei, d));
            }
        }
    }
    best
}

/// The clearance problems of a constructed geometry as pairs: an element of a track and an
/// element of another track, or an element and the other net of a pad, via or other copper.
/// Per pair the kind of the check, the most that is missing, and the class of the worse track.
fn pairs(geo: &Geometry, v: &[Violation], class: &[&'static str]) -> Value {
    let mut found: BTreeMap<(String, String), (f64, usize)> = BTreeMap::new();
    for x in v {
        let kind = x.what.split(' ').next().unwrap_or("").to_string();
        let a = nearest(geo, x.nets.0, x.at).filter(|n| n.2 < 0.5);
        let b = x
            .nets
            .1
            .filter(|_| kind == "track")
            .and_then(|n| nearest(geo, n, x.at))
            .filter(|n| n.2 < 1.0);
        let name = |n: Option<(usize, usize, f64)>| n.map_or("-".to_string(), |n| format!("t{}e{}", n.0, n.1));
        let (ka, kb) = (
            name(a),
            if kind == "track" {
                name(b)
            } else {
                format!("n{:?}", x.nets.1)
            },
        );
        let key = (
            kind,
            if ka < kb {
                format!("{ka} {kb}")
            } else {
                format!("{kb} {ka}")
            },
        );
        let rank = |n: Option<(usize, usize, f64)>| {
            n.map_or(0, |n| CLASSES.iter().position(|c| *c == class[n.0]).unwrap_or(0))
        };
        let e = found.entry(key).or_insert((0.0, 0));
        e.0 = e.0.max(x.required - x.actual);
        e.1 = e.1.max(rank(a)).max(rank(b));
    }
    let mut by: BTreeMap<String, usize> = BTreeMap::new();
    for ((kind, _), (d, rank)) in &found {
        *by.entry(format!("kind {kind}")).or_default() += 1;
        *by.entry(format!("missing {}", missing_band(*d))).or_default() += 1;
        *by.entry(format!("class {}", CLASSES[*rank])).or_default() += 1;
    }
    json!({"pairs": found.len(), "by": by})
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
        runs: flag("--runs").map(|s| s.parse()).transpose()?.or(d.runs),
        generations: flag("--generations").map(|s| s.parse()).transpose()?.or(d.generations),
        geometry_generations: flag("--geometry-generations")
            .map(|s| s.parse())
            .transpose()?
            .or(d.geometry_generations),
        move_parts: Some(false),
        ..d
    };
    let rounds: Vec<usize> = flag("--rounds")
        .unwrap_or("100,200,400".into())
        .split(',')
        .filter_map(|s| s.parse().ok())
        .collect();
    let board = Path::new(&args[1]);
    let name = board.file_stem().unwrap().to_string_lossy().to_string();
    let scratch = std::env::temp_dir().join(format!("band-{}-{name}.kicad_pcb", std::process::id()));
    gummiband_core::KEEP_WINNERS.store(true, Ordering::Relaxed);
    let (t0, c0) = (Instant::now(), cpu_seconds());
    let outcome = route_file(board, &scratch, &opts, &Hooks::default())?;
    let (wall, cpu) = (t0.elapsed().as_secs_f64(), cpu_seconds() - c0);
    let routed = &outcome.routed;
    let b: &Board = &routed.board;
    let fin = &routed.res.geometry;
    let st = &routed.res.stats;
    let clear: Vec<Violation> = routed.violations.iter().filter(|v| !v.is_width()).cloned().collect();
    let spots = check::spots(&clear);

    // the router of the winner again
    let winners = std::mem::take(&mut *gummiband_core::WINNERS.lock().unwrap());
    let same = |a: &Geometry, b: &Geometry| {
        a.tracks.len() == b.tracks.len() && a.tracks.iter().zip(&b.tracks).all(|(x, y)| x.pts == y.pts)
    };
    let w = winners
        .iter()
        .rev()
        .find(|w| same(&w.finished, fin))
        .or(winners.last())
        .expect("a search ran");
    let matched = same(&w.finished, fin);
    let mut r = gummiband_core::rebuild_router(b, &routed.sub, w);
    let first = r.geometry();
    let rebuilt = first.tracks.len() == w.before_finish.tracks.len()
        && first.vias.len() == w.before_finish.vias.len()
        && first
            .tracks
            .iter()
            .zip(&w.before_finish.tracks)
            .all(|(x, y)| x.seg == y.seg && x.net == y.net && x.layer == y.layer);

    let mut construction = serde_json::Map::new();
    for (mode, on, disc, local, corridor) in [
        ("edge", false, false, false, false),
        ("stack", true, false, false, false),
        ("disc strict", false, true, false, true),
        ("disc local", false, true, true, true),
        ("disc", false, true, true, false),
    ] {
        r.stack_band = on;
        r.disc_band = disc;
        r.disc_local = local;
        r.disc_corridor = corridor;
        let (tc, cc) = (Instant::now(), cpu_seconds());
        let g = r.geometry();
        let (secs, cpu_s) = (tc.elapsed().as_secs_f64(), cpu_seconds() - cc);
        // one PathInfo per segment
        let segs: BTreeSet<u32> = g.tracks.iter().map(|t| t.seg).collect();
        let infos: BTreeMap<u32, PathInfo> = segs.iter().map(|&s| (s, r.seg_path_info(s).1)).collect();
        let class: Vec<&'static str> = g.tracks.iter().map(|t| class_of(&infos[&t.seg])).collect();
        let mut classes: BTreeMap<&str, usize> = BTreeMap::new();
        let mut why = [0usize; 6];
        let mut gave = [0usize; 8];
        let (mut bends, mut wraps, mut stack_wraps, mut corners, mut refused, mut own) = (0, 0, 0, 0, 0, 0);
        let mut idle = 0;
        // wraps lost with a polyline: all bends of the track
        let mut lost_bends = [0usize; 5];
        for i in infos.values() {
            *classes.entry(class_of(i)).or_default() += 1;
            if i.fallback == 1 {
                why[i.why as usize] += 1;
            }
            if i.fallback != 0 {
                lost_bends[i.fallback as usize] += i.bends;
            }
            for k in 0..8 {
                gave[k] += i.gave_way[k];
            }
            bends += i.bends;
            wraps += i.wraps;
            stack_wraps += i.stack_wraps;
            corners += i.corners;
            refused += i.refused;
            own += i.own;
            idle += i.idle;
        }
        let v = clearance(b, &g);
        let mut relaxed = serde_json::Map::new();
        for &n in &rounds {
            let mut gr = g.clone();
            let (tx, cx) = (Instant::now(), cpu_seconds());
            relax(
                b,
                &routed.sub,
                &mut gr,
                &RelaxConfig {
                    iterations: n,
                    ..RelaxConfig::default()
                },
            );
            let (rs, rc) = (tx.elapsed().as_secs_f64(), cpu_seconds() - cx);
            let mut c = counts(&clearance(b, &gr));
            c["relax_s"] = json!(rs);
            c["relax_cpu"] = json!(rc);
            c["shape"] = shape(&gr);
            c["crossings"] = json!(crossings(&gr).len());
            relaxed.insert(n.to_string(), c);
        }
        construction.insert(
            mode.to_string(),
            json!({
                "segments": infos.len(),
                "classes": classes,
                "no_tangent_kinds": {"end in circle": why[1], "nested": why[2], "overlap": why[3], "same centre": why[4], "more than 270": why[5]},
                "bends": bends, "wraps": wraps, "stack_wraps": stack_wraps, "corners": corners,
                "bends_of_polylines": {"no tangent": lost_bends[1], "leaves triangles": lost_bends[2], "misses portal": lost_bends[3], "arcs off": lost_bends[4]},
                "gave_way": {"end in circle": gave[0], "nested": gave[1], "overlap": gave[2], "same centre": gave[3],
                             "more than 270": gave[4], "misses portal": gave[5], "leaves triangles": gave[6], "out of lane": gave[7]},
                "refused": refused, "no_distance_corners": own, "idle": idle,
                "construct_s": secs, "construct_cpu": cpu_s,
                "shape": shape(&g),
                "crossings": crossings(&g).iter().map(|&(i, j, at)| json!({
                    "nets": [b.nets[g.tracks[i].net as usize].name.clone(), b.nets[g.tracks[j].net as usize].name.clone()],
                    "classes": [class[i], class[j]], "at": [at.x, at.y], "layer": g.tracks[i].layer,
                })).collect::<Vec<_>>(),
                "check": counts(&v),
                "pairs": pairs(&g, &v, &class),
                "relaxed": relaxed,
            }),
        );
    }
    let report = json!({
        "board": name,
        "seed": routed.seed,
        "band_on": gummiband_core::knobs::BAND.on(),
        "relax_rounds": RelaxConfig::default().iterations,
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
        "shape": shape(fin),
        "winner_matched": matched,
        "rebuilt": rebuilt,
        "construction": construction,
        "violation_list": clear.iter().map(|v| json!({
            "what": v.what, "at": [v.at.x, v.at.y], "actual": v.actual, "required": v.required,
        })).collect::<Vec<_>>(),
    });
    std::fs::write(&args[2], serde_json::to_string(&report)?)?;
    let _ = std::fs::remove_file(&scratch);
    let _ = std::fs::remove_file(scratch.with_extension("kicad_pro"));
    let c = &report["construction"];
    println!(
        "{name} seed {} band {}: {}/{} routed, {} violations at {} spots, {} vias, {:.0} mm, {:.1} s cpu; construction pieces edge {} stack {} disc {}, matched {matched} rebuilt {rebuilt}",
        routed.seed,
        if gummiband_core::knobs::BAND.on() { "on" } else { "off" },
        st.routed,
        st.conns,
        clear.len(),
        spots.len(),
        st.vias,
        st.length,
        cpu,
        c["edge"]["check"]["pieces"],
        c["stack"]["check"]["pieces"],
        c["disc"]["check"]["pieces"],
    );
    Ok(())
}
