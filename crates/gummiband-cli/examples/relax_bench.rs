//! Compare the relaxation on the CPU and the GPU on one unrelaxed routing.
//! Usage: relax_bench BOARD [--batch N] [--repeat N] [--modes cpu,gpu]
//! The routing comes from a small search with GB_NO_RELAX, the same for all modes.

use std::hash::{Hash, Hasher};
use std::time::Instant;

use gummiband_core::relax::{RelaxConfig, relax};
use gummiband_core::route::RouterConfig;
use gummiband_core::topo::SubstrateConfig;
use gummiband_core::{SearchConfig, check, route_board};
use rayon::prelude::*;

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

/// Core seconds this process used so far: unlike the wall clock this hardly depends on what
/// else runs on the machine.
fn cpu_seconds() -> f64 {
    let mut u = RUsage::default();
    unsafe { getrusage(0, &mut u) };
    (u.utime[0] + u.stime[0]) as f64 + (u.utime[1] + u.stime[1]) as f64 * 1e-6
}

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let flag = |name: &str, d: &str| -> String {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1).cloned())
            .unwrap_or_else(|| d.to_string())
    };
    let batch: usize = flag("--batch", "1").parse()?;
    let repeat: usize = flag("--repeat", "1").parse()?;
    let modes = flag("--modes", "cpu,gpu");
    // the same relaxation with a margin changed by next to nothing shows how chaotic it is
    let ensemble: usize = flag("--ensemble", "0").parse()?;
    let kb = gummiband_kicad::read_board(std::path::Path::new(&args[1]), &Default::default())?;
    unsafe { std::env::set_var("GB_NO_RELAX", "1") };
    let search = SearchConfig {
        population: 4,
        generations: 1,
        geometry_generations: 0,
        frames: false,
        progress: None,
        seed: None,
    };
    let t = Instant::now();
    let (sub, res) = route_board(
        &kb.board,
        &SubstrateConfig::default(),
        &RouterConfig::default(),
        &search,
    );
    println!(
        "routed {}/{} in {:.1} s, {} tracks, {} vias",
        res.stats.routed,
        res.stats.conns,
        t.elapsed().as_secs_f64(),
        res.geometry.tracks.len(),
        res.geometry.vias.len()
    );
    for mode in modes.split(',').filter(|_| ensemble > 0) {
        let t = Instant::now();
        let mut counts: Vec<(usize, usize)> = (0..ensemble)
            .map(|k| {
                let d = RelaxConfig::default();
                let cfg = RelaxConfig {
                    gpu: mode == "gpu",
                    margin: d.margin + k as f64 * 1e-7,
                    ..d
                };
                let mut g = res.geometry.clone();
                relax(&kb.board, &sub, &mut g, &cfg);
                let v = check::check(&kb.board, &g);
                (v.len(), v.iter().filter(|x| x.actual <= 0.0).count())
            })
            .collect();
        counts.sort_unstable();
        println!("{mode} ensemble {:.1} s: {counts:?}", t.elapsed().as_secs_f64());
    }
    for mode in modes.split(',').filter(|_| ensemble == 0) {
        for _ in 0..repeat {
            let cfg = RelaxConfig {
                gpu: mode == "gpu",
                ..Default::default()
            };
            let (t, c0, p0) = (Instant::now(), cpu_seconds(), gummiband_core::prof::snapshot());
            let done: Vec<_> = (0..batch)
                .into_par_iter()
                .map(|_| {
                    let mut g = res.geometry.clone();
                    relax(&kb.board, &sub, &mut g, &cfg);
                    g
                })
                .collect();
            let secs = t.elapsed().as_secs_f64();
            let core_s = cpu_seconds() - c0;
            let p1 = gummiband_core::prof::snapshot();
            let ms = |i: usize| (p1[i] - p0[i]) as f64 * 1e-6;
            let rounds = (p1[8] - p0[8]).max(1) as f64;
            println!(
                "  {core_s:.2} core s, per round: pieces {:.2}, vias {:.2}, tracks {:.2} ms; gather {:.2}, upload {:.2}, wait {:.2} ms",
                ms(2) / rounds,
                ms(3) / rounds,
                ms(4) / rounds,
                ms(10) / rounds,
                ms(11) / rounds,
                ms(12) / rounds,
            );
            let g = &done[0];
            let v = check::check(&kb.board, g);
            let shorts = v.iter().filter(|x| x.actual <= 0.0).count();
            let len: f64 = g
                .tracks
                .iter()
                .map(|t| t.pts.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>())
                .sum();
            let mut h = std::collections::hash_map::DefaultHasher::new();
            for t in &g.tracks {
                for p in &t.pts {
                    (p.x.to_bits(), p.y.to_bits()).hash(&mut h);
                }
            }
            println!(
                "{mode}: {secs:.2} s, {} violations ({shorts} shorts), {len:.1} mm, hash {:016x}",
                v.len(),
                h.finish()
            );
        }
    }
    Ok(())
}
