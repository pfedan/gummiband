//! A watched route: what the watch is shown, and that looking changes nothing.

use std::collections::BTreeSet;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use gummiband_app::{FINISHED_PHASE, Hooks, RouteOptions, route_file};
use gummiband_core::board::Board;
use gummiband_core::watch::{Stage, UNTAGGED, View, Watch};

fn bench(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../bench/boards")
        .join(name)
}

/// What a test keeps of a view.
#[derive(Debug, Clone)]
struct Seen {
    stage: Stage,
    phase: String,
    /// The connections of its lines, the sites of its dots, the connections of its open lines.
    lines: BTreeSet<u32>,
    dots: BTreeSet<u32>,
    open: BTreeSet<u32>,
    gone: Vec<u32>,
    gone_sites: Vec<u32>,
    explored: usize,
    step: (usize, usize),
    routed: Option<(usize, usize)>,
    violations: Option<usize>,
}

fn seen(v: &View) -> Seen {
    Seen {
        stage: v.stage,
        phase: v.phase.to_string(),
        lines: v.lines.iter().map(|l| l.conn).collect(),
        dots: v.dots.iter().map(|d| d.site).collect(),
        open: v.open.iter().map(|o| o.2).collect(),
        gone: v.gone.to_vec(),
        gone_sites: v.gone_sites.to_vec(),
        explored: v.explored.len(),
        step: v.step,
        routed: v.routed,
        violations: v.violations,
    }
}

#[test]
fn a_watched_route_shows_its_steps_and_gives_the_same_board() {
    let dir = std::env::temp_dir().join(format!("gummiband-watch-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let opts = RouteOptions {
        runs: Some(4),
        generations: Some(1),
        geometry_generations: Some(1),
        ..Default::default()
    };
    let plain = dir.join("plain.kicad_pcb");
    route_file(&bench("medium.kicad_pcb"), &plain, &opts, &Hooks::default())
        .unwrap()
        .write()
        .unwrap();

    let views: Arc<Mutex<Vec<Seen>>> = Default::default();
    let boards = Arc::new(Mutex::new(0));
    let begin = {
        let (views, boards) = (views.clone(), boards.clone());
        move |_: &Board| {
            *boards.lock().unwrap() += 1;
            let views = views.clone();
            // every glimpse that is due at once: the test sees what a slow machine would
            Watch::new(Duration::ZERO, move |v: &View| views.lock().unwrap().push(seen(v)))
        }
    };
    let hooks = Hooks {
        watch: Some(&begin),
        ..Default::default()
    };
    let watched = dir.join("watched.kicad_pcb");
    let out = route_file(&bench("medium.kicad_pcb"), &watched, &opts, &hooks).unwrap();
    out.write().unwrap();
    assert_eq!(
        std::fs::read_to_string(&plain).unwrap(),
        std::fs::read_to_string(&watched).unwrap(),
        "looking changes nothing"
    );
    let _ = std::fs::remove_dir_all(&dir);

    let views = views.lock().unwrap().clone();
    assert_eq!(
        *boards.lock().unwrap(),
        2,
        "the board of the search, and the board as written"
    );
    let conns = out.routed.res.stats.conns;

    // a run begins with nothing laid and everything open
    let first = &views[0];
    assert_eq!((first.stage, first.phase.as_str()), (Stage::Laying, "portfolio"));
    assert_eq!(first.step, (0, conns));
    assert!(first.lines.is_empty() && first.open.len() == conns);

    // the steps change the whole view before them into the whole view after them
    let (mut lines, mut dots, mut open) = (BTreeSet::new(), BTreeSet::new(), BTreeSet::new());
    let (mut steps, mut searched) = (0, 0);
    for v in &views {
        match v.stage {
            Stage::Step => {
                steps += 1;
                searched += v.explored;
                for c in &v.gone {
                    lines.remove(c);
                    open.remove(c);
                }
                for s in &v.gone_sites {
                    dots.remove(s);
                }
                assert!(v.lines.iter().chain(&v.open).all(|c| v.gone.contains(c)), "{v:?}");
                lines.extend(&v.lines);
                dots.extend(&v.dots);
                open.extend(&v.open);
                assert_eq!(v.step, (lines.len(), conns));
            }
            Stage::Laying => {
                let tagged: BTreeSet<u32> = v.lines.iter().copied().filter(|&c| c != UNTAGGED).collect();
                if v.step.0 > 0 {
                    assert_eq!(tagged, lines, "the lines of the steps before {:?}", v.step);
                    assert_eq!(v.dots, dots, "the vias of the steps before {:?}", v.step);
                    assert_eq!(v.open, open, "the open connections of the steps before {:?}", v.step);
                }
                (lines, dots, open) = (tagged, v.dots.clone(), v.open.clone());
                assert_eq!(v.step, (lines.len(), conns));
                assert_eq!(lines.len() + open.len(), conns);
            }
            _ => {}
        }
    }
    assert!(
        steps >= 2 * conns,
        "a step per connection of the lead runs of two generations: {steps}"
    );
    assert!(searched > 0, "the steps show where their searches looked");

    // the phases in their order, each with its best candidate
    let best: Vec<&str> = views
        .iter()
        .filter(|v| v.stage == Stage::Best)
        .map(|v| v.phase.as_str())
        .collect();
    assert_eq!(best[..3], ["portfolio", "generation 1", "geometry"]);
    assert_eq!(*best.last().unwrap(), FINISHED_PHASE);
    let relaxing: Vec<&Seen> = views.iter().filter(|v| v.stage == Stage::Relaxing).collect();
    assert!(relaxing.len() > 10, "the rounds of a relaxation: {}", relaxing.len());
    assert!(relaxing.iter().all(|v| v.step.0 < v.step.1));
    let last = views.last().unwrap();
    assert_eq!(last.routed, Some((conns, conns)));
    assert_eq!(last.violations, Some(out.routed.violations.len()));
}
