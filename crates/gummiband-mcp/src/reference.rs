//! `reference_check`: does every signal track run over unbroken copper of its reference plane?
//!
//! The return current of a fast signal flows in the plane right under the track. Where the
//! plane has no copper under it (a slot, the edge of a zone, a row of antipads that merged) or
//! where the track passes from the zone of one net into the zone of another (a split plane),
//! the return current has to detour, which adds loop inductance, ringing and EMI; under a
//! differential pair it costs common mode rejection. The check rasters the filled zones of the
//! layers next to each routing layer and walks the centre line of every track over them.

use gummiband_core::math;
use std::path::PathBuf;

use anyhow::{Result, bail};
use gummiband_core::board::{Board, LayerId, NetId, ObstacleKind};
use gummiband_core::geom::{Point, Polygon, Shape};
use gummiband_kicad::power::{self, ZoneFill};
use gummiband_kicad::{KeepCopper, KicadBoard, ReadOptions, read_board};
use rmcp::schemars::{self, JsonSchema};
use serde::{Deserialize, Serialize};

use crate::external;

/// Raster step of the reference layers in mm.
const STEP: f64 = 0.05;
/// Default for the shortest gap that is listed; shorter ones (mostly one antipad of a
/// neighbouring via) are only counted.
const MIN_GAP_MM: f64 = 0.5;
/// Zone clearance assumed where a zone gives none (KiCad's default).
const ZONE_CLEARANCE: f64 = 0.5;

fn r3(x: f64) -> f64 {
    (x * 1000.0).round() / 1000.0
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct ReferenceCheckIn {
    /// Path of a routed .kicad_pcb.
    pub path: String,
    /// Nets to check (names or globs, `*` any run, `?` one character). Left out: every net with
    /// tracks except the ground net, nets with a plane and nets of clearly wide (power) classes.
    pub nets: Option<Vec<String>>,
    /// Shortest gap in mm that is listed (default 0.5); shorter gaps are only counted.
    pub min_gap: Option<f64>,
    /// Refill the zones with kicad-cli first (default true when a reference layer has unfilled
    /// zones and kicad-cli is found). Without fill the zone outlines count, which hides the
    /// antipads and is optimistic.
    pub refill: Option<bool>,
    /// How many findings to list (default 30, the longest gaps and the splits first).
    pub max_items: Option<usize>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ReferenceLayerOut {
    /// Routing layer.
    pub layer: String,
    /// The copper layers next to it in the stackup that carry zones (its reference planes).
    pub references: Vec<String>,
    /// Nets of the zones on those layers.
    pub zone_nets: Vec<String>,
    /// Track length checked on this layer in mm.
    pub track_mm: f64,
}

#[derive(Debug, Clone, Serialize, JsonSchema)]
pub struct ReferenceIssueOut {
    pub net: String,
    /// "gap": no copper of any zone under the track; "split": the track passes from a zone of
    /// one net into a zone of another net on its reference layer.
    pub kind: String,
    /// Layer of the track.
    pub layer: String,
    /// The reference layer the finding is on.
    pub reference_layer: String,
    /// Middle of the gap, or the point of the split.
    pub x: f64,
    pub y: f64,
    /// Length of the gap along the track (0 for a split).
    pub length_mm: f64,
    /// For a split: the zone nets before and after it along the track.
    pub from_net: Option<String>,
    pub to_net: Option<String>,
    /// The net belongs to a recognised differential pair.
    pub pair: bool,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ReferenceNetOut {
    pub net: String,
    pub track_mm: f64,
    /// Track length over no copper at all, short gaps included.
    pub gap_mm: f64,
    pub gaps: usize,
    pub splits: usize,
    pub pair: bool,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct ReferenceCheckOut {
    pub path: String,
    /// The refilled copy that was measured, if any.
    pub measured: String,
    pub layers: Vec<ReferenceLayerOut>,
    pub nets_checked: usize,
    pub track_mm: f64,
    /// Gaps of at least min_gap and splits, over all nets.
    pub findings: usize,
    /// Gaps shorter than min_gap (counted, not listed).
    pub short_gaps: usize,
    /// The first max_items findings: splits, then gaps longest first.
    pub items: Vec<ReferenceIssueOut>,
    /// Nets with findings, worst first.
    pub nets: Vec<ReferenceNetOut>,
    pub notes: Vec<String>,
}

/// One reference layer on a raster: per cell the index of the zone that covers it (u32::MAX: no
/// copper).
struct Raster {
    x0: f64,
    y0: f64,
    nx: usize,
    ny: usize,
    cell: Vec<u32>,
}

const NONE: u32 = u32::MAX;

impl Raster {
    fn at(&self, p: Point) -> u32 {
        let ix = ((p.x - self.x0) / STEP).floor();
        let iy = ((p.y - self.y0) / STEP).floor();
        if ix < 0.0 || iy < 0.0 || ix as usize >= self.nx || iy as usize >= self.ny {
            return NONE;
        }
        self.cell[iy as usize * self.nx + ix as usize]
    }

    /// Fills `poly` (even-odd, so KiCad's filled polygons with their bridged holes come out
    /// right) with `id`, row by row at the cell centres.
    fn fill(&mut self, poly: &Polygon, id: u32) {
        let n = poly.pts.len();
        if n < 3 {
            return;
        }
        let bb = poly.bbox();
        let r0 = (((bb.min.y - self.y0) / STEP).floor().max(0.0)) as usize;
        let r1 = ((((bb.max.y - self.y0) / STEP).ceil()) as usize).min(self.ny);
        let mut xs: Vec<f64> = Vec::new();
        for row in r0..r1 {
            let y = self.y0 + (row as f64 + 0.5) * STEP;
            xs.clear();
            let mut j = n - 1;
            for i in 0..n {
                let (a, b) = (poly.pts[j], poly.pts[i]);
                if (a.y > y) != (b.y > y) {
                    xs.push(a.x + (y - a.y) / (b.y - a.y) * (b.x - a.x));
                }
                j = i;
            }
            xs.sort_by(f64::total_cmp);
            for span in xs.chunks_exact(2) {
                let c0 = ((span[0] - self.x0) / STEP - 0.5).ceil().max(0.0) as usize;
                let c1 = ((span[1] - self.x0) / STEP - 0.5).floor();
                if c1 < 0.0 {
                    continue;
                }
                let c1 = (c1 as usize).min(self.nx - 1);
                for c in c0..=c1 {
                    self.cell[row * self.nx + c] = id;
                }
            }
        }
    }
}

/// The copper layers next to `l` that carry zones.
fn references(zf: &[ZoneFill], nl: usize, l: LayerId) -> Vec<LayerId> {
    let mut out = Vec::new();
    for m in [l as i64 - 1, l as i64 + 1] {
        if m < 0 || m >= nl as i64 {
            continue;
        }
        let m = m as LayerId;
        if zf.iter().any(|z| z.layer == m) {
            out.push(m);
        }
    }
    out
}

/// Largest pad clearance of the zones in the file (the antipad around a foreign hole).
fn zone_clearance(kb: &KicadBoard) -> f64 {
    kb.root
        .children("zone")
        .filter_map(|z| z.child("connect_pads")?.child_f64("clearance"))
        .fold(None, |m: Option<f64>, c| Some(m.map_or(c, |m| m.max(c))))
        .unwrap_or(ZONE_CLEARANCE)
}

fn default_nets(b: &Board) -> Vec<NetId> {
    let dw = b
        .classes
        .iter()
        .find(|c| c.name == "Default")
        .map(|c| c.track_width)
        .unwrap_or_else(|| b.classes.iter().map(|c| c.track_width).fold(f64::INFINITY, f64::min));
    let gnd = crate::practice::ground_net(b);
    b.routable_nets()
        .filter(|&n| Some(n) != gnd && b.nets[n as usize].planes.is_empty())
        .filter(|&n| b.class_of(n).track_width < (2.0 * dw).max(0.5) - 1e-6)
        .collect()
}

/// A stretch of one track over no copper, before the stretches that meet are joined.
struct Run {
    net: NetId,
    layer: LayerId,
    reference: LayerId,
    a: Point,
    b: Point,
    len: f64,
}

pub fn reference_check(p: &ReferenceCheckIn) -> Result<ReferenceCheckOut> {
    let path = PathBuf::from(&p.path);
    if !path.is_file() {
        bail!("{} does not exist", path.display());
    }
    let opts = ReadOptions {
        keep: KeepCopper::All,
        ..Default::default()
    };
    let min_gap = p.min_gap.unwrap_or(MIN_GAP_MM).max(0.0);
    let max_items = p.max_items.unwrap_or(30);
    let mut notes = Vec::new();
    let first = read_board(&path, &opts)?;
    let nl = first.board.layers.len();
    let unfilled = {
        let zf = power::zones(&first);
        (0..nl as LayerId)
            .filter(|&l| !first.board.layers[l as usize].is_plane)
            .flat_map(|l| references(&zf, nl, l))
            .any(|m| zf.iter().any(|z| z.layer == m && !z.filled))
    };
    let mut measured = path.clone();
    let kb = if p.refill.unwrap_or(unfilled) {
        match external::refilled(&path) {
            Ok(f) => {
                measured = f;
                read_board(&measured, &opts)?
            }
            Err(e) => {
                notes.push(format!(
                    "zones not refilled ({e:#}); unfilled zones count with their outline, which hides the antipads and is optimistic"
                ));
                first
            }
        }
    } else {
        first
    };
    let b = &kb.board;
    let zf = power::zones(&kb);
    let zclr = zone_clearance(&kb);
    // nets
    let nets: Vec<NetId> = match &p.nets {
        Some(v) if !v.is_empty() => {
            let mut out = Vec::new();
            for q in v {
                let hits: Vec<NetId> = match b.net_id(q) {
                    Some(n) => vec![n],
                    None => b
                        .routable_nets()
                        .filter(|&n| power::glob(q, &b.nets[n as usize].name))
                        .collect(),
                };
                if hits.is_empty() {
                    bail!("no net matches {q:?}");
                }
                out.extend(hits);
            }
            out.sort();
            out.dedup();
            out
        }
        _ => default_nets(b),
    };
    let mut pair_net = vec![false; b.nets.len()];
    for (a, c) in b.diff_pairs() {
        pair_net[a as usize] = true;
        pair_net[c as usize] = true;
    }
    // rasters of the reference layers
    let mut bb = gummiband_core::geom::Rect::empty();
    for o in b.obstacles.iter() {
        let r = o.shape.bbox();
        bb.add(r.min);
        bb.add(r.max);
    }
    for z in zf.iter() {
        for q in z.polys.iter() {
            let r = q.bbox();
            bb.add(r.min);
            bb.add(r.max);
        }
    }
    if bb.min.x > bb.max.x {
        bail!("the board has no copper");
    }
    let (x0, y0) = (bb.min.x - 1.0, bb.min.y - 1.0);
    let nx = ((bb.max.x + 1.0 - x0) / STEP).ceil() as usize + 1;
    let ny = ((bb.max.y + 1.0 - y0) / STEP).ceil() as usize + 1;
    if nx.saturating_mul(ny) > 40_000_000 {
        bail!("board too large for the raster ({nx} x {ny} cells of {STEP} mm)");
    }
    let mut rasters: Vec<Option<Raster>> = (0..nl).map(|_| None).collect();
    let mut layers_out = Vec::new();
    let mut refs_of: Vec<Vec<LayerId>> = vec![Vec::new(); nl];
    for l in 0..nl as LayerId {
        if b.layers[l as usize].is_plane {
            continue;
        }
        let has_tracks = b
            .obstacles
            .iter()
            .any(|o| o.kind == ObstacleKind::Track && o.layers.has(l) && o.net.is_some_and(|n| nets.contains(&n)));
        if !has_tracks {
            continue;
        }
        let refs = references(&zf, nl, l);
        if refs.is_empty() {
            notes.push(format!(
                "{} has no zone on a layer next to it: its tracks have no reference plane and were not checked",
                b.layers[l as usize].name
            ));
        }
        for &m in refs.iter() {
            if rasters[m as usize].is_none() {
                let mut r = Raster {
                    x0,
                    y0,
                    nx,
                    ny,
                    cell: vec![NONE; nx * ny],
                };
                // lower priority first, so the zone KiCad fills first wins where outlines overlap
                let mut idx: Vec<usize> = (0..zf.len()).filter(|&i| zf[i].layer == m).collect();
                idx.sort_by_key(|&i| zf[i].priority);
                for i in idx {
                    for q in zf[i].polys.iter() {
                        r.fill(q, i as u32);
                    }
                }
                rasters[m as usize] = Some(r);
            }
        }
        let mut zn: Vec<String> = zf
            .iter()
            .filter(|z| refs.contains(&z.layer))
            .map(|z| {
                z.net
                    .map_or_else(|| "(no net)".to_string(), |n| b.nets[n as usize].name.clone())
            })
            .collect();
        zn.sort();
        zn.dedup();
        layers_out.push(ReferenceLayerOut {
            layer: b.layers[l as usize].name.clone(),
            references: refs.iter().map(|&m| b.layers[m as usize].name.clone()).collect(),
            zone_nets: zn,
            track_mm: 0.0,
        });
        refs_of[l as usize] = refs;
    }
    // where a track meets a hole of its own net (its via, a through hole pin), the plane has the
    // hole's antipad: that is the layer change, not a gap
    let mut holes: Vec<Vec<(Point, f64)>> = vec![Vec::new(); b.nets.len()];
    for o in b.obstacles.iter() {
        let (Some(n), Some(_)) = (o.net, o.hole) else { continue };
        let r = o.shape.bbox();
        let c = Point {
            x: (r.min.x + r.max.x) * 0.5,
            y: (r.min.y + r.max.y) * 0.5,
        };
        let rad = 0.5 * (r.max.x - r.min.x).max(r.max.y - r.min.y);
        holes[n as usize].push((c, rad + zclr + 2.0 * STEP));
    }
    let net_of_zone = |i: u32| -> Option<NetId> { if i == NONE { None } else { zf[i as usize].net } };
    let mut runs: Vec<Run> = Vec::new();
    let mut splits: Vec<ReferenceIssueOut> = Vec::new();
    let mut track_mm = vec![0.0f64; b.nets.len()];
    let mut layer_mm = vec![0.0f64; nl];
    for o in b.obstacles.iter().filter(|o| o.kind == ObstacleKind::Track) {
        let Some(net) = o.net.filter(|n| nets.contains(n)) else {
            continue;
        };
        let Shape::Capsule { a, b: e, .. } = o.shape else {
            continue;
        };
        let Some(l) = o.layers.iter().next() else { continue };
        let refs = &refs_of[l as usize];
        if refs.is_empty() {
            continue;
        }
        let len = (e - a).len();
        track_mm[net as usize] += len;
        layer_mm[l as usize] += len;
        let steps = ((len / (STEP * 0.5)).ceil() as usize).max(1);
        let pts: Vec<Point> = (0..=steps).map(|i| a + (e - a) * (i as f64 / steps as f64)).collect();
        let excluded: Vec<bool> = pts
            .iter()
            .map(|&q| holes[net as usize].iter().any(|&(c, r)| (q - c).len() <= r))
            .collect();
        let under: Vec<Vec<u32>> = refs
            .iter()
            .map(|&m| {
                let r = rasters[m as usize].as_ref().unwrap();
                pts.iter().map(|&q| r.at(q)).collect()
            })
            .collect();
        let covered = |i: usize| under.iter().any(|u| u[i] != NONE);
        // gaps: no copper on any reference layer
        let mut start: Option<usize> = None;
        for i in 0..=pts.len() {
            let gap = i < pts.len() && !excluded[i] && !covered(i);
            match (gap, start) {
                (true, None) => start = Some(i),
                (false, Some(s)) => {
                    let (pa, pb) = (pts[s], pts[i - 1]);
                    // the nearest reference layer that has no copper there
                    runs.push(Run {
                        net,
                        layer: l,
                        reference: refs[0],
                        a: pa,
                        b: pb,
                        len: (pb - pa).len() + STEP * 0.5,
                    });
                    start = None;
                }
                _ => {}
            }
        }
        // splits: the zone net under the track changes on one reference layer while no other
        // reference layer runs on unbroken underneath
        for (k, &m) in refs.iter().enumerate() {
            let mut last: Option<(usize, NetId)> = None;
            for i in 0..pts.len() {
                if excluded[i] {
                    continue;
                }
                let Some(zn) = net_of_zone(under[k][i]) else { continue };
                if let Some((j, ln)) = last
                    && ln != zn
                {
                    let other_holds = under.iter().enumerate().any(|(kk, u)| {
                        kk != k && (j..=i).all(|t| u[t] != NONE && net_of_zone(u[t]) == net_of_zone(u[j]))
                    });
                    let q = (pts[j] + pts[i]) * 0.5;
                    // one crossing seen from two pieces of track (a joint, an arc) is one split
                    let seen = splits.iter().any(|s| {
                        s.net == b.nets[net as usize].name
                            && s.reference_layer == b.layers[m as usize].name
                            && math::hypot(s.x - q.x, s.y - q.y) < 0.5
                    });
                    if !other_holds && !seen {
                        splits.push(ReferenceIssueOut {
                            net: b.nets[net as usize].name.clone(),
                            kind: "split".into(),
                            layer: b.layers[l as usize].name.clone(),
                            reference_layer: b.layers[m as usize].name.clone(),
                            x: r3(q.x),
                            y: r3(q.y),
                            length_mm: 0.0,
                            from_net: Some(b.nets[ln as usize].name.clone()),
                            to_net: Some(b.nets[zn as usize].name.clone()),
                            pair: pair_net[net as usize],
                        });
                    }
                }
                last = Some((i, zn));
            }
        }
    }
    // join the stretches of one net and layer that meet at a joint of two track pieces
    let mut joined: Vec<Run> = Vec::new();
    runs.sort_by_key(|r| (r.net, r.layer));
    for r in runs {
        let near = |p: Point, q: Point| (p - q).len() <= STEP * 1.5;
        if let Some(j) = joined
            .iter_mut()
            .rev()
            .take_while(|j| j.net == r.net && j.layer == r.layer)
            .find(|j| near(j.a, r.a) || near(j.a, r.b) || near(j.b, r.a) || near(j.b, r.b))
        {
            // keep the two far ends
            let cands = [(j.a, r.b), (j.a, r.a), (j.b, r.a), (j.b, r.b)];
            let (pa, pb) = cands
                .into_iter()
                .max_by(|x, y| (x.0 - x.1).len().total_cmp(&(y.0 - y.1).len()))
                .unwrap();
            j.a = pa;
            j.b = pb;
            j.len += r.len;
        } else {
            joined.push(r);
        }
    }
    let mut per: Vec<ReferenceNetOut> = Vec::new();
    let mut gaps: Vec<ReferenceIssueOut> = Vec::new();
    let mut short_gaps = 0;
    for &n in nets.iter() {
        let mine: Vec<&Run> = joined.iter().filter(|r| r.net == n).collect();
        let long: Vec<&&Run> = mine.iter().filter(|r| r.len + 1e-9 >= min_gap).collect();
        short_gaps += mine.len() - long.len();
        let nsplits = splits.iter().filter(|s| s.net == b.nets[n as usize].name).count();
        for r in long.iter() {
            let q = (r.a + r.b) * 0.5;
            gaps.push(ReferenceIssueOut {
                net: b.nets[n as usize].name.clone(),
                kind: "gap".into(),
                layer: b.layers[r.layer as usize].name.clone(),
                reference_layer: b.layers[r.reference as usize].name.clone(),
                x: r3(q.x),
                y: r3(q.y),
                length_mm: r3(r.len),
                from_net: None,
                to_net: None,
                pair: pair_net[n as usize],
            });
        }
        if !long.is_empty() || nsplits > 0 {
            per.push(ReferenceNetOut {
                net: b.nets[n as usize].name.clone(),
                track_mm: r3(track_mm[n as usize]),
                gap_mm: r3(mine.iter().map(|r| r.len).sum()),
                gaps: long.len(),
                splits: nsplits,
                pair: pair_net[n as usize],
            });
        }
    }
    per.sort_by(|x, y| {
        (y.splits, y.gap_mm)
            .partial_cmp(&(x.splits, x.gap_mm))
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| x.net.cmp(&y.net))
    });
    gaps.sort_by(|x, y| y.length_mm.total_cmp(&x.length_mm));
    let findings = splits.len() + gaps.len();
    let mut items: Vec<ReferenceIssueOut> = splits.into_iter().chain(gaps).collect();
    items.truncate(max_items);
    for lo in layers_out.iter_mut() {
        if let Some(l) = b.layer_id(&lo.layer) {
            lo.track_mm = r3(layer_mm[l as usize]);
        }
    }
    if per.iter().any(|n| n.pair) {
        notes.push(
            "differential pairs over a gap or a split lose common mode rejection: route them over unbroken ground"
                .into(),
        );
    }
    if findings > 0 {
        notes.push(
            "a split is fixed by routing the track over one zone, or by a stitching capacitor between the two nets next to the crossing; a gap by moving the track, closing the slot (move vias apart, smaller zone clearance) or a ground track on the other side"
                .into(),
        );
    }
    Ok(ReferenceCheckOut {
        path: path.display().to_string(),
        measured: measured.display().to_string(),
        layers: layers_out,
        nets_checked: nets.len(),
        track_mm: r3(track_mm.iter().sum()),
        findings,
        short_gaps,
        items,
        nets: per,
        notes,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two 4 layer boards in one: a signal from R1 to R2 on F.Cu over In1.Cu, where In1.Cu has
    /// a GND zone on the left and a +5V zone on the right (`split`), or one GND zone with a slot
    /// across the track (`slot`).
    fn write_board(dir: &std::path::Path, kind: &str) -> PathBuf {
        let smd = |num: &str, x: f64, net: &str| {
            format!("(pad \"{num}\" smd rect (at {x} 0) (size 0.6 0.6) (layers \"F.Cu\" \"F.Mask\") (net \"{net}\"))")
        };
        let part = |r: &str, x: f64, pads: [String; 2]| {
            format!(
                "(footprint \"Test:{r}\" (layer \"F.Cu\") (at {x} 110) (property \"Reference\" \"{r}\" (at 0 0) (layer \"F.Fab\")) (attr smd) {} {})",
                pads[0], pads[1]
            )
        };
        let zone = |net: &str, pts: &[(f64, f64)], filled: &[(f64, f64)]| {
            let p = |v: &[(f64, f64)]| {
                v.iter()
                    .map(|(x, y)| format!("(xy {x} {y})"))
                    .collect::<Vec<_>>()
                    .join(" ")
            };
            let fill = if filled.is_empty() {
                String::new()
            } else {
                format!("(filled_polygon (layer \"In1.Cu\") (pts {}))", p(filled))
            };
            format!(
                "(zone (net \"{net}\") (layer \"In1.Cu\") (uuid \"00000000-0000-0000-0000-00000000000{}\") (connect_pads (clearance 0.2)) (polygon (pts {})) {fill})",
                net.len(),
                p(pts)
            )
        };
        let zones = match kind {
            "split" => vec![
                zone(
                    "GND",
                    &[(100.0, 100.0), (110.0, 100.0), (110.0, 120.0), (100.0, 120.0)],
                    &[(100.5, 100.5), (109.8, 100.5), (109.8, 119.5), (100.5, 119.5)],
                ),
                zone(
                    "+5V",
                    &[(110.0, 100.0), (120.0, 100.0), (120.0, 120.0), (110.0, 120.0)],
                    &[(110.2, 100.5), (119.5, 100.5), (119.5, 119.5), (110.2, 119.5)],
                ),
            ],
            // a 1 mm slot from the top edge down past the track, at x 109.5..110.5
            _ => vec![zone(
                "GND",
                &[(100.0, 100.0), (120.0, 100.0), (120.0, 120.0), (100.0, 120.0)],
                &[
                    (100.5, 100.5),
                    (109.5, 100.5),
                    (109.5, 112.0),
                    (110.5, 112.0),
                    (110.5, 100.5),
                    (119.5, 100.5),
                    (119.5, 119.5),
                    (100.5, 119.5),
                ],
            )],
        };
        let text = format!(
            "(kicad_pcb (version 20260206) (generator \"gummiband_test\") (generator_version \"10.0\")\n\
             (general (thickness 1.6)) (paper \"A4\")\n\
             (layers (0 \"F.Cu\" signal) (4 \"In1.Cu\" power) (6 \"In2.Cu\" power) (2 \"B.Cu\" signal) \
             (1 \"F.Mask\" user) (3 \"B.Mask\" user) (25 \"Edge.Cuts\" user) (35 \"F.Fab\" user))\n\
             (setup (pad_to_mask_clearance 0))\n{}\n{}\n\
             (segment (start 103 110) (end 117 110) (width 0.2) (layer \"F.Cu\") (net \"SIG\") (uuid \"10000000-0000-0000-0000-000000000001\"))\n\
             {}\n\
             (gr_rect (start 100 100) (end 120 120) (stroke (width 0.1) (type default)) (fill no) (layer \"Edge.Cuts\"))\n)\n",
            part("R1", 102.0, [smd("1", 1.0, "SIG"), smd("2", -1.0, "GND")]),
            part("R2", 118.0, [smd("1", -1.0, "SIG"), smd("2", 1.0, "+5V")]),
            zones.join("\n")
        );
        let pcb = dir.join(format!("{kind}.kicad_pcb"));
        std::fs::write(&pcb, text).unwrap();
        let pro = serde_json::json!({"net_settings": {"classes": [{"name": "Default", "clearance": 0.15,
            "track_width": 0.2, "via_diameter": 0.6, "via_drill": 0.3}]}});
        std::fs::write(pcb.with_extension("kicad_pro"), pro.to_string()).unwrap();
        pcb
    }

    fn check(kind: &str) -> ReferenceCheckOut {
        let dir = std::env::temp_dir().join(format!("gb-refcheck-{}-{kind}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let pcb = write_board(&dir, kind);
        reference_check(&ReferenceCheckIn {
            path: pcb.display().to_string(),
            nets: None,
            min_gap: None,
            refill: Some(false),
            max_items: None,
        })
        .unwrap()
    }

    #[test]
    fn a_track_across_two_zones_is_a_split_and_a_gap() {
        let o = check("split");
        assert_eq!(o.layers.len(), 1, "{:?}", o.layers);
        assert_eq!(o.layers[0].references, vec!["In1.Cu".to_string()]);
        let s: Vec<_> = o.items.iter().filter(|i| i.kind == "split").collect();
        assert_eq!(s.len(), 1, "{:?}", o.items);
        assert_eq!(s[0].from_net.as_deref(), Some("GND"));
        assert_eq!(s[0].to_net.as_deref(), Some("+5V"));
        assert!((s[0].x - 110.0).abs() < 0.3, "{:?}", s[0]);
        // the 0.4 mm between the fills is shorter than the default min_gap: counted only
        assert!(o.items.iter().all(|i| i.kind != "gap"), "{:?}", o.items);
        assert_eq!(o.short_gaps, 1);
        assert_eq!(o.nets.len(), 1);
        assert_eq!(o.nets[0].net, "SIG");
    }

    #[test]
    fn a_slot_under_a_track_is_a_gap() {
        let o = check("slot");
        let g: Vec<_> = o.items.iter().filter(|i| i.kind == "gap").collect();
        assert_eq!(g.len(), 1, "{:?}", o.items);
        assert!((g[0].length_mm - 1.0).abs() < 0.11, "{:?}", g[0]);
        assert!((g[0].x - 110.0).abs() < 0.1, "{:?}", g[0]);
        assert_eq!(g[0].reference_layer, "In1.Cu");
        assert!(o.items.iter().all(|i| i.kind != "split"));
    }
}
