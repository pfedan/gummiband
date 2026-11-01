//! Moving a part on a routed file with the copper following it: see
//! [`gummiband_core::follow::move_with_copper`].

use anyhow::{Result, bail};
use gummiband_core::follow::{FollowStats, move_with_copper, moving_refs};
use gummiband_core::geom::{Point, pt};
use gummiband_core::realize::Elem;

use crate::read::KicadBoard;
use crate::write::{self, Item, StripOptions};

/// What [`move_part_following`] did.
#[derive(Debug, Clone, serde::Serialize)]
pub struct MovedPart {
    pub reference: String,
    /// Where the part was and where it is now, in mm.
    pub from: (f64, f64),
    pub to: (f64, f64),
    /// Other footprints that moved along (the rest of its block).
    pub block: Vec<String>,
    #[serde(flatten)]
    pub stats: FollowStats,
    pub items_removed: usize,
    pub items_written: usize,
}

/// The position of footprint `reference` in the file.
pub fn footprint_at(kb: &KicadBoard, reference: &str) -> Option<Point> {
    kb.root
        .args()
        .iter()
        .filter(|n| matches!(n.head(), Some("footprint") | Some("module")))
        .find(|fp| {
            let name = fp
                .children("property")
                .find(|p| p.args().first().and_then(|x| x.as_str()) == Some("Reference"))
                .and_then(|p| p.args().get(1)?.as_str())
                .or_else(|| {
                    fp.children("fp_text")
                        .find(|t| t.args().first().and_then(|x| x.as_str()) == Some("reference"))
                        .and_then(|t| t.args().get(1)?.as_str())
                });
            name == Some(reference)
        })
        .and_then(|fp| {
            let a = fp.child("at")?.args();
            Some(pt(a.first()?.as_f64()?, a.get(1)?.as_f64()?))
        })
}

/// The file text of `kb` (read with [`crate::KeepCopper::All`]) with footprint `reference`
/// moved towards (`x`, `y`), as far as the copper around it can give way, and that copper
/// following it. Locked copper and copper nothing touched stay as they are in the file.
pub fn move_part_following(kb: &KicadBoard, reference: &str, x: f64, y: f64) -> Result<(String, MovedPart)> {
    let Some(from) = footprint_at(kb, reference) else {
        bail!("no footprint {reference:?}");
    };
    if !kb.board.courtyards.contains_key(reference) && !kb.board.footprint_obstacles.contains_key(reference) {
        bail!("footprint {reference:?} has no pads or courtyard to move");
    }
    let mut b = kb.board.clone();
    let (geo, changes, reached, stats) = move_with_copper(&mut b, reference, pt(x, y) - from);
    let refs = moving_refs(&b, reference);
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
    for v in geo
        .vias
        .iter()
        .filter(|v| v.drill > 0.0 && v.origin.is_some_and(|o| changes.nodes[o as usize]))
    {
        items.push(Item::Via {
            at: v.at,
            diameter: v.diameter,
            drill: v.drill,
            net: v.net,
        });
    }
    let ranges = write::strip_ranges(
        kb,
        &StripOptions {
            items: changes.strip.clone(),
            ..Default::default()
        },
    );
    let text = write::patch(kb, &ranges, &items);
    let text = if reached.len() > 0.0 {
        let offsets = refs.iter().map(|r| (r.clone(), reached)).collect();
        write::move_footprints(&text, &offsets, &kb.board.blocks)?
    } else {
        text
    };
    let to = from + reached;
    let out = MovedPart {
        reference: reference.to_string(),
        from: (from.x, from.y),
        to: (to.x, to.y),
        block: refs.into_iter().filter(|r| r != reference).collect(),
        stats,
        items_removed: ranges.len(),
        items_written: items.len(),
    };
    Ok((text, out))
}
