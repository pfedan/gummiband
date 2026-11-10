//! The 45 degree step on a routed file, without routing again: see
//! [`gummiband_core::octo::octilinear_copper`].

use anyhow::{Result, bail};
use gummiband_core::octo::{OctoStats, octilinear_copper};
use gummiband_core::realize::Elem;

use crate::read::KicadBoard;
use crate::write::{self, Item, StripOptions};

/// What [`angles_45`] did.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Angles45 {
    #[serde(flatten)]
    pub stats: OctoStats,
    /// Runs of track between vias, branches and pads that changed (and were written again).
    pub tracks_changed: usize,
    pub items_removed: usize,
    pub items_written: usize,
    /// Gummiband's own clearance check before and after.
    pub violations_before: usize,
    pub violations_after: usize,
}

/// The file text of `kb` (read with [`crate::KeepCopper::All`]) with the unlocked tracks of
/// `nets` (all nets if empty) on the 45 degree grid. Vias, track ends and locked copper stay;
/// tracks that do not change stay in the file as they are.
pub fn angles_45(kb: &KicadBoard, nets: &[String]) -> Result<(String, Angles45)> {
    for n in nets {
        if kb.board.net_id(n).is_none() {
            bail!("unknown net {n}");
        }
    }
    let mut b = kb.board.clone();
    let (geo, changes, stats) = octilinear_copper(&mut b, nets);
    let mut old = geo.clone();
    for (t, ch) in old.tracks.iter_mut().zip(&b.soft.chains) {
        t.pts = ch.pts.clone();
        t.elems = ch.elems.clone();
        t.fitted.clear();
    }
    // this step changes copper it did not lay: it may not take a track out, so a result with
    // a short that the file does not have is not written
    let shorts = gummiband_core::legal::new_shorts(&b, &old, &b, &geo);
    if !shorts.is_empty() {
        bail!(
            "the 45 degree step would short {}: nothing written, please report this board",
            shorts.join(", ")
        );
    }
    let violations_before = gummiband_core::check::check(&b, &old).len();
    let violations_after = gummiband_core::check::check(&b, &geo).len();
    let mut items = Vec::new();
    for t in geo
        .tracks
        .iter()
        .filter(|t| t.origin.is_some_and(|o| changes.chains[o as usize]))
    {
        for e in &t.elems {
            items.push(match *e {
                Elem::Line(a, b) => Item::Track {
                    width: t.piece_width(a, b),
                    a,
                    b,
                    layer: t.layer,
                    net: t.net,
                },
                Elem::Arc(a, mid, b) => Item::Arc {
                    width: t.piece_width(a, b),
                    a,
                    mid,
                    b,
                    layer: t.layer,
                    net: t.net,
                },
            });
        }
    }
    let ranges = write::strip_ranges(
        kb,
        &StripOptions {
            items: changes.strip.clone(),
            ..Default::default()
        },
    );
    let text = write::patch(kb, &ranges, &items);
    let out = Angles45 {
        stats,
        tracks_changed: changes.chains.iter().filter(|c| **c).count(),
        items_removed: ranges.len(),
        items_written: items.len(),
        violations_before,
        violations_after,
    };
    Ok((text, out))
}
