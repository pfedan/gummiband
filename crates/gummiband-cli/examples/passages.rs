//! The findings of the own check on a routed board, each with its passage
//! (`gummiband_core::passage`): from the two items outwards to the first thing that cannot
//! move, what lies between needs, and what room there is from wall to wall. Routes the board
//! as `gummiband route` does with the same options, and writes one entry per spot: the kind
//! of the finding, the walls, the tracks and vias between them, and the room that is left
//! with a margin of 0, 2, 4 and 7 µm on top of every clearance (2 µm: what a constructed
//! piece is to keep; 4 µm: what the shortcuts ask of a line today; 7 µm: what the arcs around
//! pivots keep). Negative room: so much is missing, and no geometry can lay it. Also the room
//! with the vias where they are, and the fewest things that have to move to give the two
//! items their clearance. For a via among the two items, as one body through all layers
//! (`via_room`): the least room where it is, what leaves it that, and how far the nearest
//! place is where nothing is missing.
//!
//! `--dump STATE.json` keeps the routed state (board, copper, findings), and `--from
//! STATE.json` reads one instead of routing: the same spots without the route.
//! Usage: passages BOARD OUT.json [--seed N] [--runs N] [--generations N]
//!        [--geometry-generations N] [--no-move-parts] [--dump STATE.json]
//!        passages --from STATE.json OUT.json
use std::path::Path;

use gummiband_app::{Hooks, RouteOptions, route_file};
use gummiband_core::board::Board;
use gummiband_core::check;
use gummiband_core::passage::{Kind, passages, via_room};
use gummiband_core::realize::Geometry;
use serde_json::{Value, json};

/// The margins in mm the room is given for.
const MARGINS: [f64; 4] = [0.0, 0.002, 0.004, 0.007];

/// A routed board as the check sees it.
struct State {
    name: String,
    seed: u64,
    conns: usize,
    routed: usize,
    vias: usize,
    board: Board,
    geometry: Geometry,
    violations: Vec<check::Violation>,
}

fn route(args: &[String]) -> anyhow::Result<State> {
    let flag = |name: &str| {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1).cloned())
    };
    let d = RouteOptions::default();
    let opts = RouteOptions {
        seed: flag("--seed").map(|s| s.parse()).transpose()?,
        runs: flag("--runs").map(|s| s.parse()).transpose()?.or(d.runs),
        generations: flag("--generations").map(|s| s.parse()).transpose()?.or(d.generations),
        geometry_generations: flag("--geometry-generations")
            .map(|s| s.parse())
            .transpose()?
            .or(d.geometry_generations),
        move_parts: if args.iter().any(|a| a == "--no-move-parts") {
            Some(false)
        } else {
            d.move_parts
        },
        ..d
    };
    let board = Path::new(&args[1]);
    let name = board.file_stem().unwrap().to_string_lossy().to_string();
    let scratch = std::env::temp_dir().join(format!("passages-{}-{name}.kicad_pcb", std::process::id()));
    let outcome = route_file(board, &scratch, &opts, &Hooks::default())?;
    let routed = outcome.routed;
    let st = &routed.res.stats;
    let state = State {
        name,
        seed: routed.seed,
        conns: st.conns,
        routed: st.routed,
        vias: st.vias,
        board: routed.board,
        geometry: routed.res.geometry,
        violations: routed.violations,
    };
    if let Some(path) = flag("--dump") {
        // a board with a rule whose key JSON cannot write (a pair of nets) has no dump
        let dump = || -> serde_json::Result<String> {
            serde_json::to_string(&json!({
                "name": state.name,
                "seed": state.seed,
                "conns": state.conns,
                "routed": state.routed,
                "vias": state.vias,
                "board": serde_json::to_value(&state.board)?,
                "geometry": serde_json::to_value(&state.geometry)?,
                "violations": serde_json::to_value(&state.violations)?,
            }))
        };
        match dump() {
            Ok(text) => std::fs::write(path, text)?,
            Err(e) => eprintln!("no dump of this board: {e}"),
        }
    }
    Ok(state)
}

fn read(path: &str) -> anyhow::Result<State> {
    let mut all: Value = serde_json::from_str(&std::fs::read_to_string(path)?)?;
    let count = |v: &Value| v.as_u64().unwrap_or(0) as usize;
    Ok(State {
        name: all["name"].as_str().unwrap_or("").to_string(),
        seed: all["seed"].as_u64().unwrap_or(0),
        conns: count(&all["conns"]),
        routed: count(&all["routed"]),
        vias: count(&all["vias"]),
        board: serde_json::from_value(all["board"].take())?,
        geometry: serde_json::from_value(all["geometry"].take())?,
        violations: serde_json::from_value(all["violations"].take())?,
    })
}

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let state: State = if args[1] == "--from" {
        read(&args[2])?
    } else {
        route(&args)?
    };
    let out = if args[1] == "--from" { &args[3] } else { &args[2] };
    let name = &state.name;
    let b = &state.board;
    let fin = &state.geometry;
    let clear: Vec<check::Violation> = state.violations.iter().filter(|v| !v.is_width()).cloned().collect();
    let spots = check::spots(&clear);
    let found = passages(b, fin, &clear);
    let net = |n: Option<u32>| n.map(|n| b.nets[n as usize].name.clone());
    let um = |x: f64| (x * 1e4).round() / 10.0;
    // the part a spot lies at: the nearest pin within 1.5 mm
    let part = |p: gummiband_core::geom::Point| {
        b.pins
            .iter()
            .map(|pin| (pin.pos.dist(p), &pin.footprint))
            .filter(|x| x.0 < 1.5)
            .min_by(|x, y| x.0.total_cmp(&y.0))
            .map(|x| x.1.clone())
    };
    let list: Vec<Value> = found
        .iter()
        .map(|p| {
            let walls = p.walls();
            json!({
                "what": p.what,
                "kind": p.kind(),
                "at": [p.at.x, p.at.y],
                "part": part(p.at),
                "layer": p.layer,
                "missing_um": um(p.missing),
                "pieces": p.pieces,
                "crossing": p.crossing,
                "walls": [walls[0].kind.word(), walls[1].kind.word()],
                "wall_nets": [net(walls[0].net), net(walls[1].net)],
                "own": p.own,
                "tracks": p.tracks(),
                "vias": p.vias(),
                "items": p.items,
                "chain": p.chain.iter().map(|t| json!({
                    "is": t.kind.word(), "net": net(t.net), "lo_um": um(t.lo), "hi_um": um(t.hi),
                    "grown_um": um(t.grown)})).collect::<Vec<_>>(),
                "clearances_um": p.clearances.iter().map(|&c| um(c)).collect::<Vec<_>>(),
                "available_um": um(p.available()),
                "own_slack_um": um(p.own_slack()),
                "needed_um": um(p.needed()),
                "gaps": p.gaps(),
                "grown_um": um(p.grown()),
                "grown_vias_stay_um": um(p.grown_vias_stay()),
                "open": walls.iter().any(|w| w.kind == Kind::Open),
                "room_um": MARGINS.iter().map(|&m| um(p.room(m))).collect::<Vec<_>>(),
                "room_vias_stay_um": MARGINS.iter().map(|&m| um(p.room_vias_stay(m))).collect::<Vec<_>>(),
                "movers": MARGINS.iter().map(|&m| p.movers(m, true)).collect::<Vec<_>>(),
                "movers_vias_stay": MARGINS.iter().map(|&m| p.movers(m, false)).collect::<Vec<_>>(),
                // each via among the two items, as one body through all layers
                "via_room": p.via_items().iter().map(|&vi| {
                    let rooms: Vec<_> = MARGINS.iter().map(|&m| via_room(b, fin, vi, m, &[p.dir])).collect();
                    json!({
                        "net": net(Some(fin.vias[vi].net)),
                        "at": [fin.vias[vi].at.x, fin.vias[vi].at.y],
                        "stays_um": rooms.iter().map(|r| um(r.stays)).collect::<Vec<_>>(),
                        "tight": rooms.iter().map(|r| r.tight.map(|t| json!({
                            "layer": t.layer, "is": t.kind.word(), "net": net(t.net), "between": t.between,
                            "available_um": um(t.available), "dir": [t.dir.x, t.dir.y]}))).collect::<Vec<_>>(),
                        "home_um": rooms.iter().map(|r| r.home.map(|h| um(h.1))).collect::<Vec<_>>(),
                        "home": rooms.iter().map(|r| r.home.map(|h| [h.0.x, h.0.y])).collect::<Vec<_>>(),
                    })
                }).collect::<Vec<_>>(),
                "from": [p.from.x, p.from.y],
                "dir": [p.dir.x, p.dir.y],
            })
        })
        .collect();
    let with_room = |m: f64| found.iter().filter(|p| !p.crossing && p.room(m) >= 0.0).count();
    let report = json!({
        "board": name,
        "seed": state.seed,
        "conns": state.conns,
        "routed": state.routed,
        "vias": state.vias,
        "violations": clear.len(),
        "spots": spots.len(),
        "width_findings": state.violations.len() - clear.len(),
        "margins_um": MARGINS.iter().map(|&m| um(m)).collect::<Vec<_>>(),
        "passages": list,
    });
    std::fs::write(out, serde_json::to_string(&report)?)?;
    println!(
        "{name} seed {}: {}/{} routed, {} violations at {} spots; {} passages, with room at 0/2/4/7 um: {}/{}/{}/{}, crossing {}",
        state.seed,
        state.routed,
        state.conns,
        clear.len(),
        spots.len(),
        found.len(),
        with_room(0.0),
        with_room(0.002),
        with_room(0.004),
        with_room(0.007),
        found.iter().filter(|p| p.crossing).count(),
    );
    Ok(())
}
