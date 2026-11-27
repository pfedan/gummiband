//! Looking over the shoulder of a running search.
//!
//! A front end hands the search a [`Watch`] ([`crate::SearchConfig::watch`]). The search shows
//! it [`View`]s of three kinds ([`Stage`]): the copper of one run while it lays its connections,
//! the tracks of one candidate while its relaxation pulls them taut, and the best candidate
//! after every phase. The first two are glimpses: they are built only when the last view is
//! long enough ago, so that looking costs a small share of the run that is watched and nothing
//! of the others. The runs of a generation go side by side; one of them, the lead, is the one
//! that is watched.
//!
//! Watching reads and never writes: a watched search gives the same board as an unwatched one.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use crate::board::{LayerId, NetId};
use crate::geom::Point;
use crate::realize::Geometry;

/// What a [`View`] shows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stage {
    /// The lead run of a generation lays its connections: all its copper as the topology of the
    /// moment gives it, and what is still open.
    Laying,
    /// One step of that run, a connection laid or given up, as a change of the views before it:
    /// the copper of the connections in [`View::gone`] and the vias in [`View::gone_sites`]
    /// went, the lines, dots and open connections of this view came. The copper of the other
    /// connections is not laid out again for a step; the next [`Stage::Laying`] has it anew.
    Step,
    /// The relaxation of the lead candidate at work.
    Relaxing,
    /// The best candidate after a phase, as the progress reports name it.
    Best,
}

/// The tag of copper that belongs to no connection or via site a viewer could follow.
pub const UNTAGGED: u32 = u32::MAX;

/// A track as a viewer draws it.
#[derive(Debug, Clone, Copy)]
pub struct Line<'a> {
    pub pts: &'a [Point],
    pub width: f64,
    pub layer: LayerId,
    pub net: NetId,
    /// The connection of the run it belongs to ([`Stage::Laying`] and [`Stage::Step`]).
    pub conn: u32,
}

/// A via as a viewer draws it.
#[derive(Debug, Clone, Copy)]
pub struct Dot {
    pub at: Point,
    pub diameter: f64,
    pub drill: f64,
    pub net: NetId,
    /// A via pair (see [`crate::realize::Via::half`]); zero for a round via.
    pub half: Point,
    /// The via site of the run it sits on ([`Stage::Laying`] and [`Stage::Step`]).
    pub site: u32,
}

/// One look at a search.
#[derive(Debug, Clone)]
pub struct View<'a> {
    pub stage: Stage,
    /// The phase: the one at work for a glimpse, the one that ended for [`Stage::Best`]. The
    /// names are those of [`crate::Progress::phase`].
    pub phase: &'a str,
    pub lines: Vec<Line<'a>>,
    pub dots: Vec<Dot>,
    /// Connections not laid: from a pin to the nearest pin it may end on (twice the pin where
    /// it ends on a plane or an area), and the connection.
    pub open: Vec<(Point, Point, u32)>,
    /// [`Stage::Step`]: the connections whose copper and open line went.
    pub gone: &'a [u32],
    /// [`Stage::Step`]: the via sites whose via went.
    pub gone_sites: &'a [u32],
    /// Where the search for the way of this step looked, with the layer ([`Stage::Step`]).
    pub explored: &'a [(Point, u8)],
    /// Clearance violations of the candidate ([`Stage::Best`] after the geometry step).
    pub spots: &'a [Point],
    /// How far the stage is: connections laid of all, or rounds of the relaxation.
    pub step: (usize, usize),
    /// Connections routed of all, where the stage knows ([`Stage::Best`]).
    pub routed: Option<(usize, usize)>,
    pub violations: Option<usize>,
}

impl<'a> View<'a> {
    /// A view with nothing in it.
    pub fn empty(stage: Stage, phase: &'a str) -> View<'a> {
        View {
            stage,
            phase,
            lines: Vec::new(),
            dots: Vec::new(),
            open: Vec::new(),
            gone: &[],
            gone_sites: &[],
            explored: &[],
            spots: &[],
            step: (0, 0),
            routed: None,
            violations: None,
        }
    }

    /// A view of `geo`, with nothing else in it.
    pub fn of(stage: Stage, phase: &'a str, geo: &'a Geometry) -> View<'a> {
        let mut v = View::empty(stage, phase);
        v.add(geo, |_| UNTAGGED, |_| UNTAGGED);
        v
    }

    /// Puts the copper of `geo` into the view; `conn` gives the connection of track i, `site`
    /// the via site of via i.
    pub fn add(&mut self, geo: &'a Geometry, conn: impl Fn(usize) -> u32, site: impl Fn(usize) -> u32) {
        self.lines.extend(geo.tracks.iter().enumerate().map(|(i, t)| Line {
            pts: &t.pts,
            width: t.width,
            layer: t.layer,
            net: t.net,
            conn: conn(i),
        }));
        // a soft point without a drill is no via
        self.dots.extend(
            geo.vias
                .iter()
                .enumerate()
                .filter(|(_, v)| v.drill > 0.0)
                .map(|(i, v)| Dot {
                    at: v.at,
                    diameter: v.diameter,
                    drill: v.drill,
                    net: v.net,
                    half: v.half,
                    site: site(i),
                }),
        );
    }
}

struct Inner {
    sink: Box<dyn Fn(&View) + Send + Sync>,
    t0: Instant,
    /// Microseconds after `t0` from which the next glimpse is due; `u64::MAX` while one is
    /// built.
    next: AtomicU64,
    every: Duration,
}

/// Receives the views of a search. Cheap to clone; the clones share the sink and the clock of
/// the glimpses.
#[derive(Clone)]
pub struct Watch {
    inner: Arc<Inner>,
    /// The phase at work, for the glimpses of a run or a relaxation, which do not know it.
    phase: Arc<str>,
}

impl std::fmt::Debug for Watch {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("Watch")
    }
}

/// A glimpse may take this share of the time of the run it looks at: after one that took a
/// millisecond to build, the next comes ten milliseconds later at the earliest.
const GLIMPSE_SHARE: u32 = 10;

impl Watch {
    /// A watch whose glimpses come at most once in `every`. `sink` is called from the threads
    /// of the search, one call at a time for the glimpses; it should return quickly.
    pub fn new(every: Duration, sink: impl Fn(&View) + Send + Sync + 'static) -> Watch {
        Watch {
            inner: Arc::new(Inner {
                sink: Box::new(sink),
                t0: Instant::now(),
                next: AtomicU64::new(0),
                every,
            }),
            phase: Arc::from(""),
        }
    }

    /// The same watch for the glimpses of `phase`.
    pub fn at(&self, phase: &str) -> Watch {
        Watch {
            inner: self.inner.clone(),
            phase: Arc::from(phase),
        }
    }

    pub fn phase(&self) -> &str {
        &self.phase
    }

    fn now(&self) -> u64 {
        self.inner.t0.elapsed().as_micros() as u64
    }

    /// Is a glimpse due? For work that only a glimpse needs and that has to start before it.
    pub fn due(&self) -> bool {
        self.now() >= self.inner.next.load(Ordering::Relaxed)
    }

    /// A view that may be left out: `build` runs only when a glimpse is due, and shows its view
    /// through the function it is handed. A `build` that panics shows nothing and costs the
    /// caller nothing.
    pub fn glimpse(&self, build: impl FnOnce(&dyn Fn(&View))) {
        let inner = &*self.inner;
        let now = self.now();
        let next = inner.next.load(Ordering::Relaxed);
        if now < next
            || inner
                .next
                .compare_exchange(next, u64::MAX, Ordering::Relaxed, Ordering::Relaxed)
                .is_err()
        {
            return;
        }
        let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| build(&|v| (inner.sink)(v))));
        let after = self.now();
        let pause = (inner.every.as_micros() as u64).max((after - now) * GLIMPSE_SHARE as u64);
        inner.next.store(after + pause, Ordering::Relaxed);
    }

    /// A view that is always shown.
    pub fn show(&self, v: &View) {
        (self.inner.sink)(v);
    }

    /// A view that is always shown, from a state that may be half way: a `build` that panics
    /// shows nothing and costs the caller nothing.
    pub fn attempt(&self, build: impl FnOnce(&dyn Fn(&View))) {
        let inner = &*self.inner;
        let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| build(&|v| (inner.sink)(v))));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex;

    #[test]
    fn glimpses_keep_their_distance_and_a_show_always_comes() {
        let seen = Arc::new(Mutex::new(Vec::new()));
        let s = seen.clone();
        let w = Watch::new(Duration::from_secs(3600), move |v: &View| {
            s.lock().unwrap().push((v.stage, v.phase.to_string()))
        });
        let geo = Geometry::default();
        let lead = w.at("portfolio");
        assert!(lead.due());
        for _ in 0..3 {
            lead.glimpse(|show| show(&View::of(Stage::Laying, lead.phase(), &geo)));
        }
        assert!(!lead.due());
        w.show(&View::of(Stage::Best, "portfolio", &geo));
        assert_eq!(
            *seen.lock().unwrap(),
            vec![
                (Stage::Laying, "portfolio".to_string()),
                (Stage::Best, "portfolio".to_string())
            ]
        );
    }

    #[test]
    fn a_glimpse_that_panics_shows_nothing_and_the_next_one_comes() {
        let seen = Arc::new(Mutex::new(0));
        let s = seen.clone();
        let w = Watch::new(Duration::ZERO, move |_: &View| *s.lock().unwrap() += 1);
        let geo = Geometry::default();
        let hook = std::panic::take_hook();
        std::panic::set_hook(Box::new(|_| {}));
        w.glimpse(|_| panic!("a topology half way"));
        std::panic::set_hook(hook);
        w.glimpse(|show| show(&View::of(Stage::Laying, "", &geo)));
        assert_eq!(*seen.lock().unwrap(), 1);
    }
}
