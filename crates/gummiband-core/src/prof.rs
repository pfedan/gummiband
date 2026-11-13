//! Cumulative phase timers and work counters, printed with `GB_LOG=timing`. Calls that run in
//! parallel add up, so the shares are of the work done, not of the wall clock. Not for loops
//! over points: an atomic that all threads touch makes the relaxation three times slower.

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

#[derive(Clone, Copy)]
pub enum Slot {
    Evaluate,
    RelaxSetup,
    RelaxPieces,
    RelaxVias,
    RelaxTracks,
    RelaxDirty,
    Taut,
    AreaModel,
    Check,
    Rounds,
    Calls,
    GpuGather,
    GpuUpload,
    GpuWait,
    GoalChecks,
    GoalShut,
    AreaModels,
}

const NAMES: [&str; 17] = [
    "evaluate",
    "relax setup",
    "relax pieces",
    "relax vias",
    "relax tracks",
    "relax dirty",
    "taut",
    "area model",
    "check",
    "rounds",
    "relax calls",
    "gpu gather ns",
    "gpu upload ns",
    "gpu wait ns",
    "goal checks",
    "goal shut in",
    "area models",
];

static SLOTS: [AtomicU64; 17] = [const { AtomicU64::new(0) }; 17];

#[inline]
pub fn add(s: Slot, n: u64) {
    SLOTS[s as usize].fetch_add(n, Ordering::Relaxed);
}

#[inline]
pub fn time<R>(s: Slot, f: impl FnOnce() -> R) -> R {
    let t = Instant::now();
    let r = f();
    add(s, t.elapsed().as_nanos() as u64);
    r
}

/// What one slot holds so far.
pub fn value(s: Slot) -> u64 {
    SLOTS[s as usize].load(Ordering::Relaxed)
}

/// The values collected so far, for a benchmark that compares phases.
pub fn snapshot() -> Vec<u64> {
    SLOTS.iter().map(|s| s.load(Ordering::Relaxed)).collect()
}

/// Print everything collected so far, if asked for.
pub fn report() {
    if !crate::knobs::log(crate::knobs::Log::Timing) {
        return;
    }
    let v: Vec<u64> = SLOTS.iter().map(|s| s.load(Ordering::Relaxed)).collect();
    let timers = Slot::Check as usize + 1;
    let sum: u64 = v[..timers].iter().sum();
    for i in 0..timers {
        eprintln!(
            "prof {:<14} {:9.2} s {:5.1} %",
            NAMES[i],
            v[i] as f64 * 1e-9,
            v[i] as f64 * 100.0 / sum.max(1) as f64
        );
    }
    for i in timers..NAMES.len() {
        eprintln!("prof {:<14} {}", NAMES[i], v[i]);
    }
}
