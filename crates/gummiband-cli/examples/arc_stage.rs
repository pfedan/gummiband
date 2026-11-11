//! The finished copper of a board with and without the arcs around pivots (`GB_PIVOT_ARCS=0`
//! turns them off).
//! Routes the board (parts stay where they are), with the knobs as the environment sets them,
//! and writes what the pipeline gave: open connections (as the pipeline counts them, and the
//! nets whose finished copper is not one piece), the own check, vias, length, CPU time,
//! the shape of the copper (lines, arcs, pieces under 50 µm, corners, per track), the tracks
//! that pass a via of another net at its clearance and in how many pieces, and what
//! became of the tracks in the last pass (`gummiband_core::pivot::OUTCOME`, summed over the
//! passes of the run: the winner and its alternatives): how many are built from their pivots
//! in the end, and why the others are not. Then it runs that pass
//! once more on the finished copper and writes the same for it: with the knob off that is the
//! pass at the very end of the pipeline, after the widths are fitted; with the knob on it shows
//! what a second pass still changes.
//! Usage: arc_stage BOARD OUT.json [--seed N] [--runs N] [--generations N]
//!        [--geometry-generations N] [--nets-file FILE]
use std::path::Path;
use std::sync::atomic::Ordering;
use std::time::Instant;

use gummiband_app::{Hooks, RouteOptions, route_file};
use gummiband_core::board::Board;
use gummiband_core::check::{self, Violation};
use gummiband_core::geom::{Point, arc_from_three, pt};
use gummiband_core::math;
use gummiband_core::pivot::{NAMES, OUTCOME, OUTCOMES, TALLY};
use gummiband_core::realize::{Elem, Geometry, flatten};
use gummiband_core::relax::{RelaxConfig, pivot_pass};
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

/// The directions in which an element starts and ends.
fn directions(e: &Elem) -> Option<(Point, Point)> {
    match *e {
        Elem::Line(a, b) => (a.dist(b) > 1e-9).then(|| ((b - a).norm(), (b - a).norm())),
        Elem::Arc(a, m, b) => {
            let (_, _, sa, sweep) = arc_from_three(a, m, b)?;
            let s = sweep.signum();
            let at = |ang: f64| pt(-math::sin(ang), math::cos(ang)) * s;
            Some((at(sa), at(sa + sweep)))
        }
    }
}

/// Lines, arcs, pieces under 50 µm, corners (two elements that meet at more than two degrees)
/// and the length of a geometry, and how many tracks have a short piece or a corner.
fn shape(geo: &Geometry) -> Value {
    let (mut lines, mut arcs, mut short, mut corners, mut length) = (0, 0, 0, 0, 0.0);
    let (mut with_short, mut with_corner, mut most) = (0, 0, 0);
    // corners by their turn: 2 to 5 degrees, 5 to 15, 15 to 45, more
    let mut by_turn = [0usize; 4];
    for t in &geo.tracks {
        let (s0, c0) = (short, corners);
        for e in &t.elems {
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
        for w in t.elems.windows(2) {
            if let (Some((_, d0)), Some((d1, _))) = (directions(&w[0]), directions(&w[1]))
                && d0.dot(d1) < math::cos((2.0f64).to_radians())
            {
                corners += 1;
                let turn = math::atan2(d0.cross(d1), d0.dot(d1)).abs().to_degrees();
                by_turn[[5.0, 15.0, 45.0].iter().filter(|&&x| turn >= x).count()] += 1;
            }
        }
        with_short += (short > s0) as usize;
        with_corner += (corners > c0) as usize;
        most = most.max(t.elems.len());
    }
    json!({"tracks": geo.tracks.len(), "lines": lines, "arcs": arcs, "pieces_under_50um": short,
           "corners": corners, "corners_by_turn": by_turn, "length": length, "tracks_with_short_piece": with_short,
           "tracks_with_corner": with_corner, "most_elements": most})
}

/// The tracks with the most short pieces and corners, at most `n` of them: net, and the
/// elements in order as (l or a, length in µm, radius in µm, turn to the next in degrees).
fn rough(geo: &Geometry, n: usize) -> Value {
    let mut all: Vec<(usize, Value)> = Vec::new();
    for t in &geo.tracks {
        let mut bad = 0;
        let mut list = Vec::new();
        for (i, e) in t.elems.iter().enumerate() {
            let l = elem_len(e);
            let turn = t.elems.get(i + 1).and_then(|n| {
                let (d0, d1) = (directions(e)?.1, directions(n)?.0);
                Some(math::atan2(d0.cross(d1), d0.dot(d1)).to_degrees())
            });
            bad += (l < 0.05) as usize + turn.is_some_and(|x| x.abs() > 2.0) as usize;
            let r = match e {
                Elem::Arc(a, m, b) => arc_from_three(*a, *m, *b).map(|x| (x.1 * 1000.0).round()),
                Elem::Line(..) => None,
            };
            let kind = if r.is_some() { "a" } else { "l" };
            list.push(json!([
                kind,
                (l * 1000.0).round(),
                r,
                turn.map(|x| (x * 10.0).round() / 10.0)
            ]));
        }
        if bad > 0 {
            let at = t.pts.first().map(|p| [p.x, p.y]);
            all.push((bad, json!({"net": t.net, "from": at, "rough": bad, "elems": list})));
        }
    }
    all.sort_by(|a, b| b.0.cmp(&a.0));
    Value::Array(all.into_iter().take(n).map(|x| x.1).collect())
}

/// The element of a track of `net` nearest to p: what it is, its length, the radius of an arc
/// and the distance of its points from p.
fn element_at(geo: &Geometry, net: u32, p: Point) -> Value {
    let mut best: Option<(f64, &Elem)> = None;
    for t in geo.tracks.iter().filter(|t| t.net == net) {
        for e in &t.elems {
            let pts = flatten(std::slice::from_ref(e));
            let d = pts
                .windows(2)
                .map(|w| gummiband_core::geom::point_seg_dist(p, w[0], w[1]))
                .fold(f64::INFINITY, f64::min);
            if best.is_none_or(|b| d < b.0) {
                best = Some((d, e));
            }
        }
    }
    match best {
        Some((d, e @ Elem::Line(..))) => json!({"is": "line", "length": elem_len(e), "distance": d}),
        Some((d, e @ Elem::Arc(a, m, b))) => json!({"is": "arc", "length": elem_len(e), "distance": d,
            "radius": arc_from_three(*a, *m, *b).map(|x| x.1)}),
        None => Value::Null,
    }
}

/// Where tracks pass vias of other nets within 30 µm of their clearance: the passages (a track
/// and a via), their elements, those under 50 µm, the elements more than half a micrometre
/// nearer than the clearance, and the twelve passages with the most pieces under 50 µm (net,
/// place of the via, elements, short ones, what the nearest lacks or has to spare in µm).
fn via_passages(board: &Board, geo: &Geometry) -> Value {
    let (mut passages, mut elems, mut short, mut near, mut short_near, mut most) = (0, 0, 0, 0, 0, 0);
    let mut one_elem = 0;
    let mut list: Vec<(usize, Value)> = Vec::new();
    for t in &geo.tracks {
        let flat: Vec<(Vec<Point>, f64)> = t
            .elems
            .iter()
            .map(|e| (flatten(std::slice::from_ref(e)), elem_len(e)))
            .collect();
        for v in geo.vias.iter().filter(|v| v.drill > 0.0 && v.net != t.net) {
            let c = board.clearance(Some(t.net), Some(v.net));
            if t.pts
                .iter()
                .all(|p| p.dist(v.at) > v.diameter / 2.0 + c + t.width + 0.1)
            {
                continue;
            }
            let (mut n, mut s, mut close, mut s_close, mut spare) = (0, 0, 0, 0, f64::INFINITY);
            for (pts, l) in &flat {
                let d = pts
                    .windows(2)
                    .map(|w| gummiband_core::geom::point_seg_dist(v.at, w[0], w[1]))
                    .fold(f64::INFINITY, f64::min);
                let (Some(&a), Some(&b)) = (pts.first(), pts.last()) else {
                    continue;
                };
                let gap = d - v.diameter / 2.0 - t.piece_width(a, b) / 2.0 - c;
                if gap > 0.03 {
                    continue;
                }
                n += 1;
                s += (*l < 0.05) as usize;
                close += (gap < -0.0005) as usize;
                s_close += (*l < 0.05 && gap < -0.0005) as usize;
                spare = spare.min(gap);
            }
            if n == 0 {
                continue;
            }
            passages += 1;
            one_elem += (n == 1) as usize;
            elems += n;
            short += s;
            near += close;
            short_near += s_close;
            most = most.max(n);
            if s > 0 {
                list.push((
                    s,
                    json!({"net": board.nets[t.net as usize].name, "via": [v.at.x, v.at.y], "elements": n,
                           "under_50um": s, "nearer_than_clearance": close, "spare_um": (spare * 1e4).round() / 10.0}),
                ));
            }
        }
    }
    list.sort_by(|a, b| b.0.cmp(&a.0));
    json!({"passages": passages, "of_one_element": one_elem, "elements": elems, "under_50um": short,
           "nearer_than_clearance": near, "under_50um_and_nearer": short_near, "most_elements": most,
           "worst": list.into_iter().take(12).map(|x| x.1).collect::<Vec<_>>()})
}

/// What became of the tracks in the last passes since the call before (`pivot::OUTCOMES`).
fn outcomes() -> Value {
    let mut m = serde_json::Map::new();
    for (name, n) in OUTCOMES.iter().zip(&OUTCOME) {
        m.insert(name.to_string(), json!(n.swap(0, Ordering::Relaxed)));
    }
    Value::Object(m)
}

fn tally() -> Value {
    let mut m = serde_json::Map::new();
    for (name, n) in NAMES.iter().zip(&TALLY) {
        m.insert(name.to_string(), json!(n.swap(0, Ordering::Relaxed)));
    }
    Value::Object(m)
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
        move_parts: Some(false),
        // one net name per line: only these nets are routed (a net that gets a pour is left out)
        nets: match flag("--nets-file") {
            Some(f) => std::fs::read_to_string(f)?
                .lines()
                .map(str::to_string)
                .filter(|l| !l.is_empty())
                .collect(),
            None => Vec::new(),
        },
        ..d
    };
    let board = Path::new(&args[1]);
    let name = board.file_stem().unwrap().to_string_lossy().to_string();
    let scratch = std::env::temp_dir().join(format!("arc-{}-{name}.kicad_pcb", std::process::id()));
    let (t0, c0) = (Instant::now(), cpu_seconds());
    let outcome = route_file(board, &scratch, &opts, &Hooks::default())?;
    let (wall, cpu) = (t0.elapsed().as_secs_f64(), cpu_seconds() - c0);
    let routed = &outcome.routed;
    let fin = &routed.res.geometry;
    let st = &routed.res.stats;
    let clear: Vec<Violation> = routed.violations.iter().filter(|v| !v.is_width()).cloned().collect();
    let spots = check::spots(&clear);
    // the nets whose finished copper is not one piece
    let nets: Vec<u32> = (0..routed.board.nets.len() as u32).collect();
    let apart = gummiband_core::legal::open_in_copper(&routed.board, fin, &nets).len();
    let whole_run = tally();
    let in_pipeline = outcomes();
    let last_instead = gummiband_core::pivot::LAST_INSTEAD.load(Ordering::Relaxed);

    // the last pass once more, on the finished copper
    let mut again = fin.clone();
    let t1 = Instant::now();
    let again_built = pivot_pass(&routed.board, &routed.sub, &mut again);
    let again_s = t1.elapsed().as_secs_f64();
    let on_finished = tally();
    let again_outcomes = outcomes();
    let after = check::check(&routed.board, &again);

    let on = gummiband_core::knobs::PIVOT_ARCS.on();
    let report = json!({
        "board": name,
        "seed": routed.seed,
        "pivot_arcs": on,
        "relax_rounds": RelaxConfig::default().iterations,
        "wall_s": wall,
        "cpu_s": cpu,
        "conns": st.conns,
        "routed": st.routed,
        "nets_apart_in_copper": apart,
        "vias": st.vias,
        "length": st.length,
        "violations": clear.len(),
        "spots": spots.len(),
        "shorts": spots.iter().filter(|s| s.short).count(),
        "width_findings": routed.violations.len() - clear.len(),
        "shape": shape(fin),
        "via_passages": via_passages(&routed.board, fin),
        "rough_tracks": rough(fin, 12),
        "tally_whole_run": whole_run,
        "outcomes": in_pipeline,
        "pass_last_instead": last_instead,
        "again": {"tally": on_finished, "outcomes": again_outcomes, "shape": shape(&again), "tracks_with_a_new_form": again_built, "wall_s": again_s,
                  "violations": after.iter().filter(|v| !v.is_width()).count()},
        "violation_list": clear.iter().map(|v| json!({
            "what": v.what, "at": [v.at.x, v.at.y], "actual": v.actual, "required": v.required,
            "element": element_at(fin, v.nets.0, v.at),
        })).collect::<Vec<_>>(),
    });
    std::fs::write(&args[2], serde_json::to_string(&report)?)?;
    let _ = std::fs::remove_file(&scratch);
    let _ = std::fs::remove_file(scratch.with_extension("kicad_pro"));
    let s = &report["shape"];
    println!(
        "{name} seed {} arcs {}: {}/{} routed, {} violations at {} spots, {} vias, {:.0} mm, {:.1} s cpu; {} lines {} arcs, {} under 50 um, {} corners",
        routed.seed,
        if on { "on" } else { "off" },
        st.routed,
        st.conns,
        clear.len(),
        spots.len(),
        st.vias,
        st.length,
        cpu,
        s["lines"],
        s["arcs"],
        s["pieces_under_50um"],
        s["corners"],
    );
    Ok(())
}
