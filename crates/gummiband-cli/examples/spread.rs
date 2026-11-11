//! Lays the squeezed places of a saved state again (`gummiband_core::spread`) and says what
//! that did: the findings of the own check before and after, what was laid, what kept its
//! copper and why. The state is one written by the example `passages` with `--dump`; with a
//! second path the state with the new copper is written there, to be read by `passages --from`.
//! Usage: spread STATE.json [OUT.json]
use gummiband_core::board::Board;
use gummiband_core::check;
use gummiband_core::realize::{Elem, Geometry};
use gummiband_core::spread;
use serde_json::Value;

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let mut all: Value = serde_json::from_str(&std::fs::read_to_string(&args[1])?)?;
    let board: Board = serde_json::from_value(all["board"].clone())?;
    let mut geo: Geometry = serde_json::from_value(all["geometry"].clone())?;
    let saved: Vec<check::Violation> = serde_json::from_value(all["violations"].clone())?;
    let count = |g: &Geometry| {
        let length: f64 = g
            .tracks
            .iter()
            .filter(|t| t.origin.is_none())
            .flat_map(|t| t.pts.windows(2))
            .map(|p| p[0].dist(p[1]))
            .sum();
        let (mut lines, mut arcs) = (0, 0);
        for e in g.tracks.iter().filter(|t| t.origin.is_none()).flat_map(|t| &t.elems) {
            match e {
                Elem::Line(..) => lines += 1,
                Elem::Arc(..) => arcs += 1,
            }
        }
        // how much of the track is narrower than its class, and at how many places the
        // width changes
        let free = || g.tracks.iter().filter(|t| t.origin.is_none());
        let narrow: f64 = free()
            .flat_map(|t| t.pts.windows(2).map(move |p| (t, p[0], p[1])))
            .filter(|(t, a, b)| t.piece_width(*a, *b) < t.width - 1e-9)
            .map(|(_, a, b)| a.dist(b))
            .sum();
        let steps: usize = free()
            .map(|t| {
                let w: Vec<f64> = t.pts.windows(2).map(|p| t.piece_width(p[0], p[1])).collect();
                w.windows(2).filter(|w| (w[0] - w[1]).abs() > 1e-9).count()
            })
            .sum();
        (length, lines, arcs, g.vias.len(), narrow, steps)
    };
    let clear = |v: &[check::Violation]| v.iter().filter(|v| !v.is_width()).cloned().collect::<Vec<_>>();
    let before = check::check(&board, &geo);
    let b = count(&geo);
    let old = geo.clone();
    let start = std::time::Instant::now();
    let report = spread::spread(&board, &mut geo);
    let secs = start.elapsed().as_secs_f64();
    let after = check::check(&board, &geo);
    let a = count(&geo);
    let moved = old
        .tracks
        .iter()
        .zip(&geo.tracks)
        .filter(|(o, n)| o.pts != n.pts)
        .count();
    println!(
        "{}: saved findings {}, check before {} ({} spots, {} of width), after {} ({} spots, {} of width)",
        args[1],
        saved.len(),
        clear(&before).len(),
        check::spots(&clear(&before)).len(),
        before.len() - clear(&before).len(),
        clear(&after).len(),
        check::spots(&clear(&after)).len(),
        after.len() - clear(&after).len(),
    );
    println!(
        "tried {}, laid {}, tracks laid {} (changed {moved}, {} stretches rough), least margin {} um, {secs:.1} s",
        report.tried,
        report.laid,
        report.tracks,
        report.rough,
        report.margin.map_or("-".to_string(), |m| format!("{:.2}", m * 1e3)),
    );
    println!(
        "length {:.3} -> {:.3} mm, lines {} -> {}, arcs {} -> {}, vias {} -> {}",
        b.0, a.0, b.1, a.1, b.2, a.2, b.3, a.3
    );
    println!(
        "narrower than its class: {:.3} -> {:.3} mm, changes of width {} -> {}",
        b.4, a.4, b.5, a.5
    );
    for k in &report.back {
        println!(
            "back: {} at ({:.3}, {:.3}), {} pieces: {}",
            k.what, k.at.x, k.at.y, k.pieces, k.why
        );
    }
    if let Some(out) = args.get(2) {
        all["geometry"] = serde_json::to_value(&geo)?;
        all["violations"] = serde_json::to_value(&after)?;
        std::fs::write(out, serde_json::to_string(&all)?)?;
    }
    Ok(())
}
