//! Good practice around routing: power planning and power copper before (`power_plan`,
//! `power_copper`), current carrying checks (`current_check`) and ground pours with stitching
//! vias (`ground_pour`) after, and the `good_practice` prompt that walks an agent through a
//! whole board.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use gummiband_core::board::{Board, LayerId, LayerSet, NetId, Obstacle, ObstacleKind};
use gummiband_core::geom::{Point, Polygon, Shape};
use gummiband_kicad::power::{self, ZoneFill};
use gummiband_kicad::{KeepCopper, KicadBoard, ReadOptions, read_board, write};
use rmcp::schemars::{self, JsonSchema};
use serde::{Deserialize, Serialize};

use crate::external;

/// Widths above this go into a pour or plane rather than a track.
pub const POUR_ABOVE_MM: f64 = 2.5;
pub const DEFAULT_RISE: f64 = 20.0;

fn r3(x: f64) -> f64 {
    (x * 1000.0).round() / 1000.0
}

fn up05(x: f64) -> f64 {
    (x / 0.05 - 1e-6).ceil() * 0.05
}

fn layer_name(b: &Board, l: LayerId) -> String {
    b.layers[l as usize].name.clone()
}

fn read_all(path: &Path) -> Result<KicadBoard> {
    read_board(
        path,
        &ReadOptions {
            keep: KeepCopper::All,
            ..Default::default()
        },
    )
}

// ------------------------------------------------------------------------------- inputs

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct CurrentIn {
    /// Net name or glob pattern (`*` any run, `?` one character), e.g. "*VBUS_P" or "/Power/SW_?".
    pub net: String,
    /// Continuous current in A.
    pub amps: f64,
    /// Short term current in A (a short or inrush the copper must survive), optional.
    pub short_amps: Option<f64>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct PowerPlanIn {
    /// Board id from load_board.
    pub board: String,
    /// Currents of the power nets. Left out: estimated from the net class widths (a class wider
    /// than the default one is taken to carry what its width carries on an outer layer).
    pub currents: Option<Vec<CurrentIn>>,
    /// Allowed temperature rise in K (default 20).
    pub temp_rise: Option<f64>,
    /// true: store the recommended track width (and the same minimum width, no necking) as a net
    /// option for the following routes, for every net whose class is too narrow.
    pub apply: Option<bool>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
pub struct PathIn {
    /// Exact net name, or a glob pattern when neither `from` nor `to` is given.
    pub net: String,
    /// Continuous current in A.
    pub amps: f64,
    pub short_amps: Option<f64>,
    /// Source terminals: pads as "REF.PAD" ("R110.*" = every pad of R110 on the net); pins in
    /// parallel join with "+" ("J1.A4+J1.B9"). Several terminals share the current evenly.
    pub from: Option<Vec<String>>,
    /// Sink pads, same syntax ("+" allowed); any of them may take the current.
    pub to: Option<Vec<String>>,
    /// A name for the table, optional.
    pub name: Option<String>,
    /// power_copper only: the layer of the pour (default the layer all terminal pads share,
    /// outer first).
    pub layer: Option<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct CurrentCheckIn {
    /// Path of a routed .kicad_pcb.
    pub path: String,
    /// Current paths. With `from` and `to` the narrowest cross-section between them is measured
    /// (tracks, pours and vias together); a net with two pins gets that automatically. Left out:
    /// the nets whose class is wider than the default class, with the current their class width
    /// carries.
    pub paths: Option<Vec<PathIn>>,
    /// Allowed temperature rise in K (default 20).
    pub temp_rise: Option<f64>,
    /// Refill the zones with kicad-cli first (default true when the file has unfilled zones of
    /// a checked net and kicad-cli is found). Without fill the zone outlines are used, which is
    /// optimistic.
    pub refill: Option<bool>,
    /// How many narrow segments to list per path (default 5).
    pub max_segments: Option<usize>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct GroundPourIn {
    /// Path of a routed .kicad_pcb.
    pub path: String,
    /// Net of the pours and vias (default "GND").
    pub net: Option<String>,
    /// Layers for the pours (default: the outer copper layers). A layer that already has a zone
    /// of the net is left as it is; the small "gb:" pours of power_copper do not count.
    pub layers: Option<Vec<String>>,
    /// How the pads of the net join the pours: "thermal" (default, thermal reliefs), "solid"
    /// (no reliefs: clears KiCad's starved_thermal where spokes find no room, harder to solder
    /// for small passives) or "thermal_tht" (reliefs on through hole pads, SMD pads solid).
    pub pad_connection: Option<String>,
    /// Width of the thermal spokes in mm (default 0.3). Four spokes carry about what a track of
    /// four times the width carries; a connector pin for several amperes wants wider ones.
    pub thermal_spoke_width: Option<f64>,
    /// Grid pitch of the stitching vias in mm (default 5, 0 = no grid).
    pub pitch: Option<f64>,
    /// Spacing of the via row along the board edge in mm (default 3, 0 = no edge row).
    pub edge_pitch: Option<f64>,
    /// Distance of the edge row from the outline in mm (default 1).
    pub edge_inset: Option<f64>,
    /// Keep the vias out of part courtyards (default true).
    pub avoid_courtyards: Option<bool>,
    /// Via size, default the net class.
    pub via_diameter: Option<f64>,
    pub via_drill: Option<f64>,
    /// Zone priority of the pours (default 0, below the power pours).
    pub priority: Option<u32>,
    /// Output file (default <input>.gnd.kicad_pcb).
    pub output: Option<String>,
}

#[derive(Debug, Deserialize, JsonSchema)]
pub struct PowerCopperIn {
    /// Board id from load_board.
    pub board: String,
    /// Current paths with `from` pads (and `to` pads for a net without plane). A net without
    /// plane gets one pour over all its terminal pads; a net with a plane gets, at every `from`
    /// and `to` terminal, vias into the plane for the current (the `from` terminals share it, every
    /// `to` terminal takes all of it) and a small pour that joins pad and vias.
    pub paths: Vec<PathIn>,
    /// Allowed temperature rise in K (default 20).
    pub temp_rise: Option<f64>,
    /// How far the pour reaches beyond the hull of its pads, mm (default 0.5).
    pub margin: Option<f64>,
    /// Terminals further apart than this (mm, default 12) get no pour: route them wide.
    pub max_span: Option<f64>,
    /// Via size of the via fields, default the net class.
    pub via_diameter: Option<f64>,
    pub via_drill: Option<f64>,
    /// Zone priority of the pours (default 20, above ground pours).
    pub priority: Option<u32>,
}

// ------------------------------------------------------------------------------ outputs

#[derive(Debug, Serialize, JsonSchema)]
pub struct CopperLayerOut {
    pub layer: String,
    pub copper_um: f64,
    pub outer: bool,
    pub plane: bool,
    /// Nets with a zone on this layer.
    pub zone_nets: Vec<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct WidthOut {
    pub layer: String,
    pub width_mm: f64,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct NetPlanOut {
    pub net: String,
    pub class: String,
    pub amps: f64,
    pub short_amps: Option<f64>,
    /// "given" or "class width" (estimated).
    pub source: String,
    pub class_width_mm: f64,
    /// Width the custom rules let the net neck down to.
    pub class_min_width_mm: Option<f64>,
    /// IPC-2221 width per copper layer for `amps`.
    pub required: Vec<WidthOut>,
    /// Track width to route with on the outer routing layers (rounded up to 0.05 mm).
    pub track_width_mm: f64,
    /// Inner planes of the net.
    pub planes: Vec<String>,
    /// Zones of the net in the file: "layer w x h".
    pub zones: Vec<String>,
    pub via_drill_mm: f64,
    pub via_amps: f64,
    /// Vias wherever the current changes layer (and per pin that takes the full current from a plane).
    pub vias_per_transition: usize,
    /// The plan asks for a change: a wider track, a pour or a plane.
    pub action: bool,
    pub advice: Vec<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct AppliedOut {
    pub net: String,
    pub width_mm: f64,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PowerPlanOut {
    pub board: String,
    pub temp_rise: f64,
    pub layers: Vec<CopperLayerOut>,
    pub nets: Vec<NetPlanOut>,
    /// Board level findings: ground plane, plane layers, reference planes.
    pub board_notes: Vec<String>,
    /// Width options stored with apply.
    pub applied: Vec<AppliedOut>,
    /// The minimum widths as KiCad custom rules, to append to the project's .kicad_dru: KiCad's
    /// DRC then enforces them, and Gummiband reads them on the next load_board.
    pub dru_rules: String,
    pub next_steps: Vec<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct SegmentOut {
    pub layer: String,
    pub width_mm: f64,
    pub required_mm: f64,
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct NeckOut {
    pub connected: bool,
    /// Narrowest cross-section in mm, as a width on the reference layer.
    pub width_mm: f64,
    /// Required width on the reference layer.
    pub required_mm: f64,
    pub reference_layer: String,
    /// Where the band breaks first.
    pub x: Option<f64>,
    pub y: Option<f64>,
    pub layer: Option<String>,
    /// Smallest total cross-section (parallel copper and vias added up, a min cut on the
    /// raster), up to twice the required width; a cut at an angle counts a little short.
    pub cross_section_mm: Option<f64>,
    /// Where that cut lies, when it is below the limit.
    pub cut_x: Option<f64>,
    pub cut_y: Option<f64>,
    pub cut_layer: Option<String>,
    pub step_mm: f64,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PathOut {
    pub name: String,
    pub net: String,
    pub amps: f64,
    pub short_amps: Option<f64>,
    /// Required width per layer the net has copper on.
    pub required: Vec<WidthOut>,
    /// Track segments narrower than required (outside pours of the net), narrowest first.
    pub narrow_segments: usize,
    pub segments: Vec<SegmentOut>,
    pub narrowest_track_mm: Option<f64>,
    pub neck: Option<NeckOut>,
    pub vias: usize,
    pub via_amps: f64,
    pub vias_per_transition: usize,
    /// "ok" or "FLAG: ..." (with a neck measurement the neck decides; narrow stubs to sense pins
    /// then do not count).
    pub verdict: String,
    pub notes: Vec<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct CurrentCheckOut {
    pub path: String,
    /// The refilled copy that was measured, if any.
    pub measured: String,
    pub temp_rise: f64,
    pub paths: Vec<PathOut>,
    pub flagged: usize,
    pub notes: Vec<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct GroundPourOut {
    pub board: String,
    pub path: String,
    pub net: String,
    pub zones_added: Vec<String>,
    pub vias_added: usize,
    pub edge_vias: usize,
    pub warnings: Vec<String>,
}

// ----------------------------------------------------------------------------- power plan

/// The ground net: "GND" if there is one, else the net with the most pins whose name holds GND.
pub fn ground_net(b: &Board) -> Option<NetId> {
    b.net_id("GND").or_else(|| {
        b.nets
            .iter()
            .enumerate()
            .filter(|(_, n)| n.name.to_uppercase().contains("GND"))
            .max_by_key(|(_, n)| n.pins.len())
            .map(|(i, _)| i as NetId)
    })
}

/// The default class: the one named "Default", else the narrowest.
fn default_width(b: &Board) -> f64 {
    b.classes
        .iter()
        .find(|c| c.name == "Default")
        .map(|c| c.track_width)
        .unwrap_or_else(|| b.classes.iter().map(|c| c.track_width).fold(f64::INFINITY, f64::min))
}

fn outer_ref(b: &Board) -> LayerId {
    (0..b.layers.len() as LayerId)
        .find(|&l| power::is_outer(b, l) && !b.layers[l as usize].is_plane)
        .unwrap_or(0)
}

/// Currents for nets of clearly wide classes (twice the default width and at least 0.5 mm):
/// what their class width carries on an outer layer.
fn estimated(b: &Board, dt: f64) -> Vec<(NetId, f64)> {
    let dw = default_width(b);
    let r = outer_ref(b);
    let cu = power::copper_mm(b, r);
    b.routable_nets()
        .filter(|&n| b.class_of(n).track_width >= (2.0 * dw).max(0.5) - 1e-6)
        .map(|n| {
            // a minimum width from the custom rules is the designer's own sizing
            let c = b.class_of(n);
            (n, power::ipc_amps(c.min_width.unwrap_or(c.track_width), cu, true, dt))
        })
        .collect()
}

fn zone_text(z: &ZoneFill, b: &Board) -> String {
    let mut r = gummiband_core::geom::Rect::empty();
    for p in &z.polys {
        let q = p.bbox();
        r.add(q.min);
        r.add(q.max);
    }
    format!("{} {:.1} x {:.1}", layer_name(b, z.layer), r.width(), r.height())
}

pub fn power_plan(kb: &KicadBoard, id: &str, p: &PowerPlanIn) -> Result<(PowerPlanOut, Vec<(String, f64)>)> {
    let b = &kb.board;
    let dt = p.temp_rise.unwrap_or(DEFAULT_RISE);
    let zf = power::zones(kb);
    let nl = b.layers.len() as LayerId;
    let mut wanted: Vec<(NetId, f64, Option<f64>, &str)> = Vec::new();
    match &p.currents {
        Some(cs) => {
            for c in cs {
                let hits: Vec<NetId> = (0..b.nets.len() as NetId)
                    .filter(|&n| power::glob(&c.net, &b.nets[n as usize].name))
                    .collect();
                if hits.is_empty() {
                    bail!("no net matches {:?} (see list_nets)", c.net);
                }
                for n in hits {
                    if !wanted.iter().any(|w| w.0 == n) {
                        wanted.push((n, c.amps, c.short_amps, "given"));
                    }
                }
            }
        }
        None => {
            for (n, a) in estimated(b, dt) {
                wanted.push((n, (a * 10.0).floor() / 10.0, None, "class width"));
            }
        }
    }
    let mut notes = Vec::new();
    if p.currents.is_none() {
        let hint = |name: &str| {
            let u = name.rsplit('/').next().unwrap_or(name).to_uppercase();
            ["VBUS", "VIN", "VCC", "VDD", "VBAT", "VSYS", "PWR", "VMOT", "VLED"]
                .iter()
                .any(|k| u.contains(k))
                || (u.starts_with('+') && u.contains('V'))
        };
        let narrow: Vec<String> = b
            .routable_nets()
            .filter(|n| !wanted.iter().any(|w| w.0 == *n))
            .filter(|&n| hint(&b.nets[n as usize].name) && b.nets[n as usize].planes.is_empty())
            .map(|n| b.nets[n as usize].name.clone())
            .collect();
        if !narrow.is_empty() {
            notes.push(format!(
                "no currents given; these look like supplies but have a narrow class, give their currents: {}",
                narrow.join(", ")
            ));
        }
    }
    let mut nets = Vec::new();
    let mut applied = Vec::new();
    let mut rules = String::new();
    for &(n, amps, short, source) in &wanted {
        let c = b.class_of(n);
        let net = &b.nets[n as usize];
        let required: Vec<(LayerId, f64)> = (0..nl)
            .map(|l| {
                // a short until the fuse or PTC trips heats the copper like a continuous current
                let a = amps.max(short.unwrap_or(0.0));
                let w = power::ipc_width(a, power::copper_mm(b, l), power::is_outer(b, l), dt);
                (l, w)
            })
            .collect();
        let outer_routing: Vec<f64> = required
            .iter()
            .filter(|(l, _)| power::is_outer(b, *l) && !b.layers[*l as usize].is_plane)
            .map(|x| x.1)
            .collect();
        let need = outer_routing.iter().copied().fold(0.0, f64::max);
        let track = up05(need).max(c.track_width);
        let planes: Vec<String> = net.planes.iter().map(|l| layer_name(b, l)).collect();
        let zones: Vec<String> = zf
            .iter()
            .filter(|z| z.net == Some(n))
            .map(|z| zone_text(z, b))
            .collect();
        let via_amps = power::via_amps(c.via_drill.max(0.1), dt);
        let vias = (amps.max(short.unwrap_or(0.0)) / via_amps).ceil().max(1.0) as usize;
        let mut advice = Vec::new();
        let pours: Vec<&ZoneFill> = zf
            .iter()
            .filter(|z| z.net == Some(n) && !b.layers[z.layer as usize].is_plane)
            .collect();
        let carried = !planes.is_empty() || !pours.is_empty();
        if !pours.is_empty() {
            advice.push(format!(
                "carried by {} pour(s) on {}: route only the short pin connections into them, then current_check from/to across the pours",
                pours.len(),
                pours.iter().map(|z| layer_name(b, z.layer)).collect::<std::collections::BTreeSet<_>>().into_iter().collect::<Vec<_>>().join(", ")
            ));
        }
        if !planes.is_empty() {
            advice.push(format!(
                "carried by the plane on {}: give every pin that takes the full current {} vias into it",
                planes.join(", "),
                vias
            ));
        }
        // a wide track into fine pins blocks the router around them: a pour does it better
        let narrow_pad = net
            .pins
            .iter()
            .map(|&i| b.pins[i].size.0.min(b.pins[i].size.1))
            .fold(f64::INFINITY, f64::min);
        let pour = need > POUR_ABOVE_MM || (need > 1.0 && narrow_pad < need / 2.0);
        if carried {
            // fan-out tracks into the plane or pour: their width matters only where they are long
        } else if pour {
            advice.push(format!(
                "{need:.2} mm is too wide for a track{}: draw a pour (add_zone on an outer layer, priority above the GND pour) or a plane, pre-route the pins into it with add_locked_copper, then skip the net or route it with keep_locked",
                if need > POUR_ABOVE_MM { String::new() } else { format!(" into pins of {narrow_pad:.2} mm") }
            ));
        } else if need > c.track_width + 1e-6 {
            advice.push(format!(
                "class {} routes {:.2} mm, {:.2} mm needed outside: route it with {track:.2} mm (apply)",
                c.name, c.track_width, need
            ));
        }
        if let Some(m) = c.min_width
            && m + 1e-6 < need
            && !carried
        {
            advice.push(format!(
                "custom rules let it neck to {m:.2} mm; apply sets the minimum width to {track:.2} mm (narrower only inside the courtyards of parts whose pads are narrower)"
            ));
        }
        let inner: Vec<String> = required
            .iter()
            .filter(|(l, _)| !power::is_outer(b, *l) && !b.layers[*l as usize].is_plane)
            .filter(|(_, w)| *w > track + 1e-6)
            .map(|(l, w)| format!("{} {:.2} mm", layer_name(b, *l), w))
            .collect();
        if !inner.is_empty() && !pour {
            advice.push(format!(
                "keep it off the inner routing layers or widen it there: {}",
                inner.join(", ")
            ));
        }
        if let Some(s) = short {
            let ws = power::ipc_width(s, power::copper_mm(b, outer_ref(b)), true, dt);
            advice.push(format!(
                "sized for the short term {s} A ({ws:.2} mm outside): the copper has to carry it until the fuse or PTC trips"
            ));
        }
        if vias > 1 {
            advice.push(format!(
                "{vias} vias of {:.2} mm drill ({via_amps:.2} A each) wherever it changes layer",
                c.via_drill
            ));
        }
        if advice.is_empty() {
            advice.push("class width is enough".into());
        }
        let wants_width =
            !carried && !pour && (need > c.track_width + 1e-6 || c.min_width.is_some_and(|m| m + 1e-6 < need));
        let action = wants_width || (!carried && pour);
        let tautology = source == "class width" && c.min_width.is_none_or(|m| m + 1e-6 >= need);
        if !carried && !tautology && !pour && need > 0.0 && net.pins.len() >= 2 {
            // pads narrower than the width: the part's courtyard is exempt
            let mut refs: Vec<&str> = net
                .pins
                .iter()
                .map(|&i| &b.pins[i])
                .filter(|q| q.size.0.min(q.size.1) < need)
                .map(|q| q.footprint.as_str())
                .collect();
            refs.sort();
            refs.dedup();
            let ex: String = refs
                .iter()
                .map(|r| format!(" && !A.intersectsCourtyard('{r}')"))
                .collect();
            rules.push_str(&format!(
                "(rule \"{} {amps} A: min {:.2} mm\"\n  (constraint track_width (min {:.2}mm) (opt {:.2}mm))\n  (condition \"A.Type == 'Track' && A.NetName == '{}'{ex}\"))\n",
                net.name.replace('"', ""),
                (need * 100.0).ceil() / 100.0,
                (need * 100.0).ceil() / 100.0,
                track,
                net.name.replace('\'', "")
            ));
        }
        if p.apply == Some(true) && wants_width && net.pins.len() >= 2 && planes.is_empty() {
            applied.push((net.name.clone(), track));
        }
        nets.push(NetPlanOut {
            net: net.name.clone(),
            class: c.name.clone(),
            amps,
            short_amps: short,
            source: source.into(),
            class_width_mm: c.track_width,
            class_min_width_mm: c.min_width,
            required: required
                .iter()
                .map(|(l, w)| WidthOut {
                    layer: layer_name(b, *l),
                    width_mm: r3(*w),
                })
                .collect(),
            track_width_mm: r3(track),
            planes,
            zones,
            via_drill_mm: c.via_drill,
            via_amps: r3(via_amps),
            vias_per_transition: vias,
            action,
            advice,
        });
    }
    nets.sort_by(|a, b| b.amps.total_cmp(&a.amps));
    // board level
    let layers: Vec<CopperLayerOut> = (0..nl)
        .map(|l| {
            let mut zn: Vec<String> = zf
                .iter()
                .filter(|z| z.layer == l)
                .filter_map(|z| z.net.map(|n| b.nets[n as usize].name.clone()))
                .collect();
            zn.sort();
            zn.dedup();
            CopperLayerOut {
                layer: layer_name(b, l),
                copper_um: power::copper_mm(b, l) * 1000.0,
                outer: power::is_outer(b, l),
                plane: b.layers[l as usize].is_plane,
                zone_nets: zn,
            }
        })
        .collect();
    let gnd = ground_net(b);
    match gnd {
        None => notes.push("no ground net found (GND)".into()),
        Some(g) => {
            let gname = &b.nets[g as usize].name;
            let gplanes: Vec<String> = b.nets[g as usize].planes.iter().map(|l| layer_name(b, l)).collect();
            if gplanes.is_empty() {
                if nl >= 4 {
                    notes.push(format!(
                        "{gname} has no plane: before routing, make an inner layer next to the outer signal layer a {gname} plane (add_zone {gname} on In1.Cu over the whole board; KiCad layer type power)"
                    ));
                } else {
                    notes.push(format!(
                        "two layer board without a {gname} plane: keep B.Cu mostly free for a {gname} pour (ground_pour after routing) and route the long signals on F.Cu"
                    ));
                }
            } else {
                notes.push(format!("{gname} plane on {}", gplanes.join(", ")));
            }
            for l in 0..nl {
                if b.layers[l as usize].is_plane {
                    continue;
                }
                // nearest plane in the stackup and its net
                let near = (0..nl)
                    .filter(|&m| b.layers[m as usize].is_plane)
                    .min_by_key(|&m| (m as i32 - l as i32).abs());
                if let Some(m) = near {
                    let pn: Vec<String> = b
                        .planes
                        .iter()
                        .filter(|p| p.layer == m)
                        .map(|p| b.nets[p.net as usize].name.clone())
                        .collect();
                    if !pn.iter().any(|n| n == gname) && !b.diff_pairs().is_empty() {
                        notes.push(format!(
                            "{} has {} ({}) as its nearest plane, not {gname}: fast signals and differential pairs there have no ground reference; route them on a layer next to the {gname} plane",
                            layer_name(b, l),
                            layer_name(b, m),
                            pn.join(", ")
                        ));
                    }
                }
            }
        }
    }
    for l in 0..nl {
        if b.layers[l as usize].is_plane && !b.planes.iter().any(|p| p.layer == l) {
            notes.push(format!(
                "{} is a plane layer without a zone: add one with add_zone (GND or the main supply)",
                layer_name(b, l)
            ));
        }
    }
    let next_steps = vec![
        "add the zones and pre-routed copper the advice asks for (add_zone, part_info + add_locked_copper), then route the returned board id".into(),
        "route; then current_check on the routed file with from/to for every power path".into(),
        "ground_pour on the routed file, then drc and render the power areas".into(),
    ];
    Ok((
        PowerPlanOut {
            board: id.to_string(),
            temp_rise: dt,
            layers,
            nets,
            board_notes: notes,
            applied: applied
                .iter()
                .map(|(n, w)| AppliedOut {
                    net: n.clone(),
                    width_mm: r3(*w),
                })
                .collect(),
            dru_rules: rules,
            next_steps,
        },
        applied,
    ))
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PourOut {
    pub name: String,
    pub net: String,
    /// "pour" (pads joined on one layer) or "via field" (pad to plane).
    pub kind: String,
    pub layer: String,
    pub terminals: Vec<String>,
    pub amps: f64,
    /// Track width the current needs on the pour's layer.
    pub required_width_mm: f64,
    pub area_mm2: f64,
    pub vias_needed: usize,
    pub vias_added: usize,
    /// What the hull gave way to.
    pub cut_by: Vec<String>,
    pub outline: Vec<[f64; 2]>,
    /// More zones of the pour, where one convex outline cannot reach a long narrow pad (the
    /// strips of a DFN eFuse or power FET between their neighbours): the narrow stubs that leave
    /// the pad along a long side or off an end, and the pours from there.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub exits: Vec<Vec<[f64; 2]>>,
}

fn exits_of(x: &[(usize, power::Pour)]) -> Vec<Vec<[f64; 2]>> {
    x.iter()
        .map(|(_, p)| p.outline.iter().map(|q| [r3(q.x), r3(q.y)]).collect())
        .collect()
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct PowerCopperOut {
    /// Id of the working copy with the pours; route this one.
    pub board: String,
    pub path: String,
    pub pours: Vec<PourOut>,
    pub warnings: Vec<String>,
    /// current_check of the pours before routing (pour paths only, zone outlines unless
    /// kicad-cli refilled them).
    pub check: Option<CurrentCheckOut>,
}

// ---------------------------------------------------------------------------- current check

/// Pins of `net` named by "REF.PAD" or "REF.*".
fn pins_of(b: &Board, net: NetId, spec: &str) -> Result<Vec<usize>> {
    let (r, pad) = spec
        .rsplit_once('.')
        .with_context(|| format!("pad {spec:?} is not REF.PAD"))?;
    let hits: Vec<usize> = b
        .pins
        .iter()
        .enumerate()
        .filter(|(_, p)| p.net == net && p.footprint == r)
        .filter(|(_, p)| power::glob(pad, p.name.rsplit_once('.').map_or("", |x| x.1)))
        .map(|(i, _)| i)
        .collect();
    if hits.is_empty() {
        bail!("no pad {spec:?} on net {}", b.nets[net as usize].name);
    }
    Ok(hits)
}

fn layer_of_shape_net(b: &Board, net: NetId) -> Vec<LayerId> {
    let mut ls: Vec<LayerId> = Vec::new();
    for o in b.obstacles.iter().filter(|o| o.net == Some(net)) {
        for l in o.layers.iter() {
            if !ls.contains(&l) {
                ls.push(l);
            }
        }
    }
    ls.sort();
    ls
}

pub fn current_check(p: &CurrentCheckIn) -> Result<CurrentCheckOut> {
    let path = PathBuf::from(&p.path);
    if !path.is_file() {
        bail!("{} does not exist", path.display());
    }
    let dt = p.temp_rise.unwrap_or(DEFAULT_RISE);
    let max_seg = p.max_segments.unwrap_or(5);
    let mut notes = Vec::new();
    let first = read_all(&path)?;
    // which paths
    let mut paths: Vec<PathIn> = match &p.paths {
        Some(v) if !v.is_empty() => v.clone(),
        _ => {
            notes.push(
                "no paths given: nets wider than the default class, with the current their class width carries".into(),
            );
            estimated(&first.board, dt)
                .into_iter()
                .map(|(n, a)| PathIn {
                    net: first.board.nets[n as usize].name.clone(),
                    amps: (a * 10.0).floor() / 10.0,
                    short_amps: None,
                    from: None,
                    to: None,
                    name: None,
                    layer: None,
                })
                .collect()
        }
    };
    // glob patterns without terminals expand to their nets
    let mut expanded = Vec::new();
    for q in paths.drain(..) {
        if q.from.is_none() && q.to.is_none() && first.board.net_id(&q.net).is_none() {
            let hits: Vec<String> = first
                .board
                .nets
                .iter()
                .filter(|n| power::glob(&q.net, &n.name) && n.pins.len() >= 2)
                .map(|n| n.name.clone())
                .collect();
            if hits.is_empty() {
                bail!("no net matches {:?}", q.net);
            }
            for h in hits {
                expanded.push(PathIn { net: h, ..q.clone() });
            }
        } else {
            expanded.push(q);
        }
    }
    let paths = expanded;
    // refill zones if needed
    let zf0 = power::zones(&first);
    let checked: Vec<Option<NetId>> = paths.iter().map(|q| first.board.net_id(&q.net)).collect();
    let unfilled = zf0.iter().any(|z| !z.filled && checked.contains(&z.net));
    let mut measured = path.clone();
    let kb = if p.refill.unwrap_or(unfilled) {
        match external::refilled(&path) {
            Ok(f) => {
                measured = f;
                read_all(&measured)?
            }
            Err(e) => {
                notes.push(format!(
                    "zones not refilled ({e:#}); unfilled zones count with their outline, which is optimistic"
                ));
                first
            }
        }
    } else {
        first
    };
    let b = &kb.board;
    let zf = power::zones(&kb);
    let ref_layer = outer_ref(b);
    let mut out = Vec::new();
    let mut flagged = 0;
    for q in &paths {
        let net = b.net_id(&q.net).with_context(|| format!("no net named {:?}", q.net))?;
        let c = b.class_of(net);
        // a short until the fuse or PTC trips heats the copper like a continuous current
        let amps = q.amps.max(q.short_amps.unwrap_or(0.0));
        let req = |l: LayerId| power::ipc_width(amps, power::copper_mm(b, l), power::is_outer(b, l), dt);
        let layers = layer_of_shape_net(b, net);
        let mut pnotes = Vec::new();
        // narrow segments outside the net's own fill on that layer
        let mut segs: Vec<SegmentOut> = Vec::new();
        let mut narrowest: Option<f64> = None;
        for o in b
            .obstacles
            .iter()
            .filter(|o| o.net == Some(net) && o.kind == ObstacleKind::Track)
        {
            let Shape::Capsule { a, b: e, r } = o.shape else {
                continue;
            };
            let Some(l) = o.layers.iter().next() else { continue };
            let w = 2.0 * r;
            narrowest = Some(narrowest.map_or(w, |m: f64| m.min(w)));
            let need = req(l);
            if w + 1e-6 >= need {
                continue;
            }
            let in_fill = |p: Point| {
                zf.iter()
                    .filter(|z| z.net == Some(net) && z.layer == l && z.filled)
                    .any(|z| z.polys.iter().any(|poly| poly.contains(p)))
            };
            if in_fill(a) && in_fill(e) {
                continue;
            }
            let m = (a + e) * 0.5;
            segs.push(SegmentOut {
                layer: layer_name(b, l),
                width_mm: r3(w),
                required_mm: r3(need),
                x: r3(m.x),
                y: r3(m.y),
            });
        }
        segs.sort_by(|x, y| x.width_mm.total_cmp(&y.width_mm));
        let narrow = segs.len();
        segs.truncate(max_seg);
        // terminals
        let net_pins: Vec<usize> = b.nets[net as usize].pins.clone();
        let terms: Option<(Vec<Vec<usize>>, Vec<usize>)> = match (&q.from, &q.to) {
            (Some(f), Some(t)) => {
                let mut src = Vec::new();
                for s in f {
                    let mut g = Vec::new();
                    for part in s.split('+') {
                        g.extend(pins_of(b, net, part.trim())?);
                    }
                    src.push(g);
                }
                let mut dst = Vec::new();
                // a sink is any of its pads: "+" joins them like separate entries
                for s in t {
                    for part in s.split('+') {
                        dst.extend(pins_of(b, net, part.trim())?);
                    }
                }
                Some((src, dst))
            }
            (None, None) if net_pins.len() == 2 => Some((vec![vec![net_pins[0]]], vec![net_pins[1]])),
            (None, None) => None,
            _ => bail!("{}: give both from and to", q.net),
        };
        let ref_req = req(ref_layer);
        let mut changes_layer = true;
        let neck = terms.map(|(src, dst)| {
            let share = amps / src.len().max(1) as f64;
            let factor: Vec<f64> = (0..b.layers.len() as LayerId)
                .map(|l| if layers.contains(&l) { req(l) / ref_req } else { 0.0 })
                .collect();
            let nk = power::neck(
                &kb,
                &zf,
                net,
                &src,
                &dst,
                &power::NeckOptions {
                    layer_factor: factor,
                    step: 0.05,
                    max_width: (ref_req * 3.0).max(2.0),
                    end_reach: 0.3,
                    temp_rise: dt,
                    ref_copper_mm: power::copper_mm(b, ref_layer),
                    ref_outer: power::is_outer(b, ref_layer),
                    required: power::ipc_width(share, power::copper_mm(b, ref_layer), true, dt),
                },
            );
            if nk.unfilled_zones > 0 {
                pnotes.push(format!(
                    "{} zones of the net unfilled: their outline was used",
                    nk.unfilled_zones
                ));
            }
            if src.len() > 1 {
                pnotes.push(format!("{} source terminals, {share:.2} A each", src.len()));
            }
            if let Some(w) = nk.cut_window {
                pnotes.push(format!(
                    "too much copper for the cross-section on the whole board: measured in the window x {:.1}..{:.1}, y {:.1}..{:.1} around the terminals, copper outside left out (on the safe side)",
                    w.min.x, w.max.x, w.min.y, w.max.y
                ));
            } else if nk.cut_mm.is_none() {
                pnotes.push("cross-section not measured (too much copper even near the terminals): the band width is all there is".into());
            }
            let need = power::ipc_width(share, power::copper_mm(b, ref_layer), true, dt);
            changes_layer = nk.changes_layer;
            NeckOut {
                connected: nk.connected,
                width_mm: r3(nk.width_mm),
                required_mm: r3(need),
                reference_layer: layer_name(b, ref_layer),
                x: nk.at.map(|a| r3(a.0.x)),
                y: nk.at.map(|a| r3(a.0.y)),
                layer: nk.at.map(|a| layer_name(b, a.1)),
                cross_section_mm: nk.cut_mm.map(r3),
                cut_x: nk.cut_at.map(|a| r3(a.0.x)),
                cut_y: nk.cut_at.map(|a| r3(a.0.y)),
                cut_layer: nk.cut_at.map(|a| layer_name(b, a.1)),
                step_mm: r3(nk.step),
            }
        });
        let vias = b
            .obstacles
            .iter()
            .filter(|o| o.net == Some(net) && o.kind == ObstacleKind::Via)
            .count();
        let via_amps = power::via_amps(c.via_drill.max(0.1), dt);
        let per = (amps / via_amps).ceil().max(1.0) as usize;
        let verdict = match &neck {
            Some(n) if !n.connected => "FLAG: terminals not connected".to_string(),
            // the cross-section decides: a band of the full width, or narrower copper in
            // parallel, and through every via barrel on the way; without it the single band
            Some(n) if n.cross_section_mm.map_or(n.width_mm, |c| c) + n.step_mm / 2.0 < n.required_mm => {
                let (x, y, l, w) = match n.cross_section_mm {
                    Some(c) => (
                        n.cut_x.or(n.x),
                        n.cut_y.or(n.y),
                        n.cut_layer.clone().or(n.layer.clone()),
                        c,
                    ),
                    None => (n.x, n.y, n.layer.clone(), n.width_mm),
                };
                let at_via = match (x, y) {
                    (Some(x), Some(y)) => b.obstacles.iter().any(|o| {
                        o.net == Some(net)
                            && o.kind == ObstacleKind::Via
                            && o.shape.bbox().center().dist(Point { x, y }) < o.shape.bbox().width() / 2.0 + 0.1
                    }),
                    _ => false,
                };
                format!(
                    "FLAG: {} {:.2} mm < {:.2} mm at {}, {} on {}",
                    match (n.cross_section_mm.is_some(), at_via) {
                        (_, true) => "via, cross-section",
                        (true, false) => "cross-section",
                        _ => "neck",
                    },
                    w,
                    n.required_mm,
                    x.unwrap_or(0.0),
                    y.unwrap_or(0.0),
                    l.unwrap_or_default()
                )
            }
            Some(_) => "ok".into(),
            None if narrow > 0 => format!(
                "FLAG: {narrow} segments below the required width (give from/to to tell stubs from the current path)"
            ),
            None => "ok".into(),
        };
        if verdict.starts_with("FLAG") {
            flagged += 1;
        }
        if ((neck.is_some() && changes_layer) || (neck.is_none() && vias > 0)) && vias < per {
            pnotes.push(format!(
                "{vias} vias on the net, {per} needed wherever the current changes layer{}",
                if neck.is_some() {
                    " (the path changes layer)"
                } else {
                    ""
                }
            ));
        }
        out.push(PathOut {
            name: q.name.clone().unwrap_or_else(|| q.net.clone()),
            net: q.net.clone(),
            amps: q.amps,
            short_amps: q.short_amps,
            required: layers
                .iter()
                .map(|&l| WidthOut {
                    layer: layer_name(b, l),
                    width_mm: r3(req(l)),
                })
                .collect(),
            narrow_segments: narrow,
            segments: segs,
            narrowest_track_mm: narrowest.map(r3),
            neck,
            vias,
            via_amps: r3(via_amps),
            vias_per_transition: per,
            verdict,
            notes: pnotes,
        });
    }
    if flagged > 0 {
        notes.push("fix a neck: reroute the net wider (set_net_options width, route with keep_all, nets and reroute), add a pour over the neck (add_zone) or locked copper, then check again".into());
    }
    Ok(CurrentCheckOut {
        path: path.display().to_string(),
        measured: measured.display().to_string(),
        temp_rise: dt,
        paths: out,
        flagged,
        notes,
    })
}

// ---------------------------------------------------------------------------- power copper

/// Pad copper of pins on a layer, as polygons.
fn pad_polys(b: &Board, pins: &[usize], layer: LayerId) -> Vec<gummiband_core::geom::Polygon> {
    pins.iter()
        .flat_map(|&p| b.pins[p].obstacles.iter())
        .map(|&o| &b.obstacles[o])
        .filter(|o| o.layers.has(layer))
        .map(|o| o.shape.to_polygon(0.01))
        .collect()
}

fn groups_of(b: &Board, net: NetId, specs: &[String]) -> Result<Vec<Vec<usize>>> {
    specs
        .iter()
        .map(|s| {
            let mut g = Vec::new();
            for part in s.split('+') {
                g.extend(pins_of(b, net, part.trim())?);
            }
            Ok(g)
        })
        .collect()
}

fn area(pts: &[Point]) -> f64 {
    gummiband_core::geom::Polygon::new(pts.to_vec()).signed_area().abs()
}

/// Lay out power copper before routing: per current path a pour over its pads or via fields
/// into its plane, as "gb:" zones (the router keeps other nets out and counts the pads under a
/// pour as joined) and locked vias. Returns the new board text and the report (board and path
/// filled in by the caller).
pub fn power_copper(kb: &KicadBoard, p: &PowerCopperIn) -> Result<(String, PowerCopperOut)> {
    let b = &kb.board;
    let dt = p.temp_rise.unwrap_or(DEFAULT_RISE);
    let margin = p.margin.unwrap_or(0.5);
    let max_span = p.max_span.unwrap_or(12.0);
    let prio = p.priority.unwrap_or(20);
    let zf = power::zones(kb);
    let mut warnings = Vec::new();
    let mut pours: Vec<PourOut> = Vec::new();
    let mut zones: Vec<(NetId, LayerId, String, Vec<Point>)> = Vec::new();
    let mut vias: Vec<write::Item> = Vec::new();
    // the copper of the paths done so far is in the way of the later ones (of other nets)
    let mut work = b.clone();
    let mut done = (0, 0);
    for q in &p.paths {
        for (net, layer, _, outline) in &zones[done.0..] {
            work.obstacles.push(Obstacle {
                shape: Shape::Polygon(Polygon::new(outline.clone())),
                layers: LayerSet::single(*layer),
                net: Some(*net),
                kind: ObstacleKind::Pour,
                hole: None,
                clearance: work.clearance(Some(*net), None),
                pin: None,
                locked: true,
            });
        }
        for v in &vias[done.1..] {
            if let write::Item::Via {
                at,
                diameter,
                drill,
                net,
            } = v
            {
                work.obstacles.push(Obstacle {
                    shape: Shape::Circle {
                        c: *at,
                        r: diameter / 2.0,
                    },
                    layers: work.all_layers(),
                    net: Some(*net),
                    kind: ObstacleKind::Via,
                    hole: Some(drill / 2.0),
                    clearance: work.clearance(Some(*net), None),
                    pin: None,
                    locked: true,
                });
            }
        }
        done = (zones.len(), vias.len());
        let b = &work;
        let net = b
            .net_id(&q.net)
            .with_context(|| format!("no net named {:?} (power_copper takes exact names)", q.net))?;
        let nname = b.nets[net as usize].name.clone();
        let label = q.name.clone().unwrap_or_else(|| nname.clone());
        let from = groups_of(b, net, q.from.as_deref().unwrap_or(&[]))?;
        let to = groups_of(b, net, q.to.as_deref().unwrap_or(&[]))?;
        if from.is_empty() {
            bail!("{label}: give the from pads");
        }
        let amps = q.amps.max(q.short_amps.unwrap_or(0.0));
        let all: Vec<usize> = from.iter().chain(to.iter()).flatten().copied().collect();
        let layer = match &q.layer {
            Some(l) => b.layer_id(l).with_context(|| format!("no layer {l:?}"))?,
            None => {
                let mut ls: Vec<LayerId> = (0..b.layers.len() as LayerId)
                    .filter(|&l| !b.layers[l as usize].is_plane && all.iter().all(|&x| b.pins[x].layers.has(l)))
                    .collect();
                ls.sort_by_key(|&l| !power::is_outer(b, l));
                match ls.first() {
                    Some(&l) => l,
                    None => {
                        warnings.push(format!("{label}: the pads share no routing layer, give layer"));
                        continue;
                    }
                }
            }
        };
        if b.layers[layer as usize].is_plane {
            bail!("{label}: {} is a plane layer", layer_name(b, layer));
        }
        let cu = power::copper_mm(b, layer);
        let need = power::ipc_width(amps, cu, power::is_outer(b, layer), dt);
        let names = |g: &[usize]| g.iter().map(|&x| b.pins[x].name.clone()).collect::<Vec<_>>().join("+");
        let has_plane = !b.nets[net as usize].planes.is_empty();
        if !has_plane {
            if to.is_empty() {
                warnings.push(format!("{label}: {nname} has no plane, give the to pads for a pour"));
                continue;
            }
            let center = |g: &[usize]| {
                g.iter().fold(Point { x: 0.0, y: 0.0 }, |s, &x| s + b.pins[x].pos) * (1.0 / g.len().max(1) as f64)
            };
            let span = from
                .iter()
                .flat_map(|f| to.iter().map(move |t| center(f).dist(center(t))))
                .fold(0.0, f64::max);
            if span > max_span {
                warnings.push(format!(
                    "{label}: terminals {span:.1} mm apart (max_span {max_span}): no pour, route it {} mm wide",
                    r3(need)
                ));
                continue;
            }
            // terminal groups (from first) with the spine pairs between them
            let join = |gs: &[&Vec<usize>], nf: usize| {
                let groups: Vec<(Vec<Polygon>, Point)> =
                    gs.iter().map(|g| (pad_polys(b, g, layer), center(g))).collect();
                let pairs: Vec<(usize, usize)> = (0..nf).flat_map(|i| (nf..gs.len()).map(move |j| (i, j))).collect();
                power::pour_joining(b, net, layer, &groups, &pairs, margin)
            };
            let whole = join(&from.iter().chain(to.iter()).collect::<Vec<_>>(), from.len());
            // terminals in parallel that one pour cannot join (the VBUS pin groups of a USB-C
            // receptacle with the signal pins between them): one pour from each to the sinks
            let split = from.len() > 1 && whole.as_ref().is_none_or(|w| !w.lost.is_empty());
            let pieces: Vec<(Vec<Vec<usize>>, Option<power::Joined>)> = if split {
                from.iter()
                    .map(|f| {
                        (
                            vec![f.clone()],
                            join(&std::iter::once(f).chain(to.iter()).collect::<Vec<_>>(), 1),
                        )
                    })
                    .collect()
            } else {
                vec![(from.clone(), whole)]
            };
            if split
                && pieces
                    .iter()
                    .filter(|(_, x)| x.as_ref().is_some_and(|x| x.lost.is_empty()))
                    .count()
                    > 0
            {
                warnings.push(format!(
                    "{label}: the from terminals do not fit under one pour on {}: one pour from each of them; join them elsewhere if they must share the current (locked copper on another layer)",
                    layer_name(b, layer)
                ));
            }
            for (fg, pour) in pieces {
                let tname = fg.iter().map(|g| names(g)).collect::<Vec<_>>().join(", ");
                let Some(pour) = pour else {
                    warnings.push(format!(
                        "{label}: no room for a pour on {}{}",
                        layer_name(b, layer),
                        if split { format!(" from {tname}") } else { String::new() }
                    ));
                    continue;
                };
                if !pour.lost.is_empty() {
                    warnings.push(format!(
                        "{label}: the pour{} does not reach every pad (other copper in between): route the rest",
                        if split { format!(" from {tname}") } else { String::new() }
                    ));
                }
                let name = if split {
                    format!("{}power {nname} {tname}", gummiband_kicad::read::GB_ZONE_PREFIX)
                } else {
                    format!("{}power {nname}", gummiband_kicad::read::GB_ZONE_PREFIX)
                };
                let mut cut_by: Vec<String> = Vec::new();
                for x in pour.pieces().flat_map(|x| x.cut_by.iter()) {
                    if !cut_by.contains(x) {
                        cut_by.push(x.clone());
                    }
                }
                pours.push(PourOut {
                    name: name.clone(),
                    net: nname.clone(),
                    kind: "pour".into(),
                    layer: layer_name(b, layer),
                    terminals: fg.iter().chain(to.iter()).map(|g| names(g)).collect(),
                    amps: if split { amps / from.len() as f64 } else { amps },
                    required_width_mm: r3(need),
                    area_mm2: r3(pour.pieces().map(|x| area(&x.outline)).sum()),
                    vias_needed: 0,
                    vias_added: 0,
                    cut_by,
                    outline: pour.main.outline.iter().map(|x| [r3(x.x), r3(x.y)]).collect(),
                    exits: exits_of(&pour.stubs),
                });
                for (_, st) in &pour.stubs {
                    zones.push((net, layer, format!("{name} exit"), st.outline.clone()));
                }
                zones.push((net, layer, name, pour.main.outline));
            }
            continue;
        }
        // a plane net: vias from each terminal into the plane
        let c = b.class_of(net);
        let (dia, drill) = (
            p.via_diameter.unwrap_or(c.via_diameter),
            p.via_drill.unwrap_or(c.via_drill),
        );
        let per_via = power::via_amps(drill, dt);
        let placed_before: Vec<Point> = vias
            .iter()
            .filter_map(|v| match v {
                write::Item::Via { at, .. } => Some(*at),
                _ => None,
            })
            .collect();
        let mut placed_all = placed_before;
        for (gi, g) in from.iter().chain(to.iter()).enumerate() {
            let share = if gi < from.len() {
                amps / from.len() as f64
            } else {
                amps
            };
            let count = (share / per_via - 1e-9).ceil().max(1.0) as usize;
            let pad = pad_polys(b, g, layer);
            if pad.is_empty() {
                warnings.push(format!(
                    "{label}: {} has no copper on {}",
                    names(g),
                    layer_name(b, layer)
                ));
                continue;
            }
            let parts: Vec<String> = g.iter().map(|&x| b.pins[x].footprint.clone()).collect();
            let room = power::ViaRoom::new(b, &zf, net, dia, drill, 0.15, true)
                .allow_in(parts)
                .into_plane();
            // vias of the net that already touch the pad count
            let have: Vec<Point> = b
                .obstacles
                .iter()
                .filter(|o| o.net == Some(net) && o.kind == ObstacleKind::Via)
                .filter_map(|o| match o.shape {
                    Shape::Circle { c, r } if pad.iter().any(|x| x.dist_to_point(c) <= r) => Some(c),
                    _ => None,
                })
                .collect();
            let missing = count.saturating_sub(have.len());
            let mut spots = power::via_field(&room, &pad, missing, &placed_all);
            let mut terms = pad.clone();
            terms.extend(
                spots
                    .iter()
                    .map(|&s| gummiband_core::geom::circle_polygon(s, dia / 2.0, 0.01)),
            );
            let pour = power::pour_outline(b, net, layer, &terms, &[], 0.2);
            let mut exits: Vec<(usize, power::Pour)> = Vec::new();
            // the vias the pour misses (all off the pad without a pour)
            let on_pad = |s: &Point| pad.iter().any(|x| x.dist_to_point(*s) <= dia / 2.0);
            let mut lost: Vec<Point> = match &pour {
                Some(pour) => pour
                    .lost
                    .iter()
                    .filter(|&&i| i >= pad.len())
                    .map(|&i| spots[i - pad.len()])
                    .collect(),
                None => spots.iter().copied().filter(|s| !on_pad(s)).collect(),
            };
            // beside a strip pad the one pour is cut down to the strip: a via off its end or side
            // gets a pour of its own that leaves the strip through an exit, or does not count
            let strip = match pad.as_slice() {
                [x] => power::Strip::of(x),
                _ => None,
            };
            if let Some(st) = strip {
                lost.retain(|&s| {
                    let groups = [
                        (pad.clone(), st.c),
                        (vec![gummiband_core::geom::circle_polygon(s, dia / 2.0, 0.01)], s),
                    ];
                    let Some(j) =
                        power::pour_joining(b, net, layer, &groups, &[(0, 1)], 0.2).filter(|j| j.lost.is_empty())
                    else {
                        return true;
                    };
                    exits.push((1, j.main));
                    exits.extend(j.stubs);
                    false
                });
                exits.dedup_by(|a, b| a.1.outline == b.1.outline);
            }
            if pour.is_some() || strip.is_some() {
                // a via the cut left outside the pour and off the pad does not count
                spots.retain(|s| !lost.contains(s) || on_pad(s));
            }
            if have.len() + spots.len() < count {
                // would the vias fit if the plane reached there? then the zone is the problem
                let loose = power::ViaRoom::new(b, &zf, net, dia, drill, 0.15, true)
                    .allow_in(g.iter().map(|&x| b.pins[x].footprint.clone()).collect());
                let beside = power::via_field(&loose, &pad, missing, &placed_all)
                    .into_iter()
                    .filter(|s| !room.reaches_plane(*s))
                    .count();
                let hint = if beside > 0 {
                    let planes: Vec<String> = b.nets[net as usize].planes.iter().map(|l| layer_name(b, l)).collect();
                    format!(
                        ": {beside} more fit beside the pad where the {nname} plane ({}) has no copper; extend its zone there or lower the priority of the zone that covers it",
                        planes.join(", ")
                    )
                } else {
                    ": add more by hand or use a larger drill".to_string()
                };
                warnings.push(format!(
                    "{label}: room for {} of {count} vias at {}{hint}",
                    have.len() + spots.len(),
                    names(g)
                ));
            }
            for &s in &spots {
                vias.push(write::Item::Via {
                    at: s,
                    diameter: dia,
                    drill,
                    net,
                });
                placed_all.push(s);
            }
            let name = format!("{}vias {nname} {}", gummiband_kicad::read::GB_ZONE_PREFIX, names(g));
            let outline = pour.as_ref().map(|x| x.outline.clone()).unwrap_or_default();
            for (_, x) in &exits {
                zones.push((net, layer, format!("{name} exit"), x.outline.clone()));
            }
            pours.push(PourOut {
                name: name.clone(),
                net: nname.clone(),
                kind: "via field".into(),
                layer: layer_name(b, layer),
                terminals: vec![names(g)],
                amps: r3(share),
                required_width_mm: r3(power::ipc_width(share, cu, power::is_outer(b, layer), dt)),
                area_mm2: r3(area(&outline) + exits.iter().map(|x| area(&x.1.outline)).sum::<f64>()),
                vias_needed: count,
                vias_added: spots.len(),
                cut_by: pour.as_ref().map(|x| x.cut_by.clone()).unwrap_or_default(),
                outline: outline.iter().map(|x| [r3(x.x), r3(x.y)]).collect(),
                exits: exits_of(&exits),
            });
            if !spots.is_empty() && outline.len() >= 3 {
                zones.push((net, layer, name, outline));
            }
        }
    }
    let mut text = if vias.is_empty() {
        kb.text.clone()
    } else {
        write::add_locked(kb, &vias)
    };
    // overlapping zones on one layer must not share a priority (KiCad: zones_intersect), e.g.
    // the via field pours of two FETs side by side: the later one goes one priority up
    let mut taken: Vec<(LayerId, u32, Polygon)> = zf
        .iter()
        .flat_map(|z| {
            z.polys
                .iter()
                .map(move |q| (z.layer, z.priority.max(0) as u32, q.clone()))
        })
        .collect();
    for (i, (net, layer, name, outline)) in zones.iter().enumerate() {
        let poly = Polygon::new(outline.clone());
        let mut priority = prio;
        while taken
            .iter()
            .any(|(l, p, q)| *l == *layer && *p == priority && q.dist_to_poly(&poly) <= 0.0)
        {
            priority += 1;
        }
        taken.push((*layer, priority, poly));
        let spec = write::ZoneSpec {
            net: *net,
            layer: *layer,
            priority,
            name,
            connect: write::PadConnect::Solid,
            spoke: write::DEFAULT_SPOKE,
        };
        text = write::add_zone_named(kb, &text, &spec, outline, 0x7c0 + i as u64);
    }
    Ok((
        text,
        PowerCopperOut {
            board: String::new(),
            path: String::new(),
            pours,
            warnings,
            check: None,
        },
    ))
}

// ----------------------------------------------------------------------------- ground pour

pub fn ground_pour(p: &GroundPourIn) -> Result<(PathBuf, GroundPourOut)> {
    let path = PathBuf::from(&p.path);
    if !path.is_file() {
        bail!("{} does not exist", path.display());
    }
    let kb = read_all(&path)?;
    let b = &kb.board;
    let net = match &p.net {
        Some(n) => b.net_id(n).with_context(|| format!("no net named {n:?}"))?,
        None => ground_net(b).context("no ground net found; name it with net")?,
    };
    let nname = b.nets[net as usize].name.clone();
    let outline = b
        .outline
        .iter()
        .max_by(|x, y| x.signed_area().abs().total_cmp(&y.signed_area().abs()))
        .context("the board has no closed outline (Edge.Cuts)")?;
    let zf = power::zones(&kb);
    let layers: Vec<LayerId> = match &p.layers {
        Some(ls) => ls
            .iter()
            .map(|n| b.layer_id(n).with_context(|| format!("no layer {n:?}")))
            .collect::<Result<_>>()?,
        None => (0..b.layers.len() as LayerId)
            .filter(|&l| power::is_outer(b, l))
            .collect(),
    };
    let prio = p.priority.unwrap_or(0);
    let mut warnings = Vec::new();
    let mut zone_layers = Vec::new();
    let connect = match p.pad_connection.as_deref() {
        None => write::PadConnect::Thermal,
        Some(s) => write::PadConnect::parse(s)
            .with_context(|| format!("pad_connection {s:?}: use thermal, solid or thermal_tht"))?,
    };
    let spoke = p.thermal_spoke_width.unwrap_or(write::DEFAULT_SPOKE);
    if spoke <= 0.0 {
        bail!("thermal_spoke_width must be positive");
    }
    for &l in &layers {
        // the small "gb:" pours of power_copper are no ground pour
        let full = |z: &&power::ZoneFill| {
            z.net == Some(net) && z.layer == l && !z.name.starts_with(gummiband_kicad::read::GB_ZONE_PREFIX)
        };
        if zf.iter().any(|z| full(&z)) {
            warnings.push(format!(
                "{} already has a {nname} zone, left as it is",
                layer_name(b, l)
            ));
            continue;
        }
        let low: Vec<String> = zf
            .iter()
            .filter(|z| z.layer == l && z.net != Some(net) && z.priority <= prio as i64)
            .filter_map(|z| z.net.map(|n| b.nets[n as usize].name.clone()))
            .collect();
        if !low.is_empty() {
            warnings.push(format!(
                "zones of {} on {} have a priority no higher than the {nname} pour ({prio}): raise theirs so they win",
                low.join(", "),
                layer_name(b, l)
            ));
        }
        zone_layers.push(l);
    }
    let c = b.class_of(net);
    let opts = power::StitchOptions {
        net,
        pitch: p.pitch.unwrap_or(5.0),
        edge_pitch: p.edge_pitch.unwrap_or(3.0),
        edge_inset: p.edge_inset.unwrap_or(1.0),
        diameter: p.via_diameter.unwrap_or(c.via_diameter),
        drill: p.via_drill.unwrap_or(c.via_drill),
        margin: 0.3,
        avoid_courtyards: p.avoid_courtyards.unwrap_or(true),
    };
    let pour_layers = zf
        .iter()
        .filter(|z| z.net == Some(net))
        .map(|z| z.layer)
        .chain(zone_layers.iter().copied())
        .chain(b.nets[net as usize].planes.iter())
        .collect::<std::collections::BTreeSet<_>>();
    let spots = if pour_layers.len() >= 2 {
        power::stitch_spots(b, &zf, &opts)
    } else {
        warnings.push(format!("{nname} has copper area on one layer only: no stitching vias"));
        Vec::new()
    };
    let edge = if opts.edge_pitch > 0.0 {
        spots
            .iter()
            .filter(|s| {
                b.outline
                    .iter()
                    .map(|q| q.boundary_dist(**s))
                    .fold(f64::INFINITY, f64::min)
                    < opts.edge_inset + 0.5
            })
            .count()
    } else {
        0
    };
    let items: Vec<write::Item> = spots
        .iter()
        .map(|&at| write::Item::Via {
            at,
            diameter: opts.diameter,
            drill: opts.drill,
            net,
        })
        .collect();
    let mut text = if items.is_empty() {
        kb.text.clone()
    } else {
        write::add_locked(&kb, &items)
    };
    for (i, &l) in zone_layers.iter().enumerate() {
        let spec = write::ZoneSpec {
            net,
            layer: l,
            priority: prio,
            name: "",
            connect,
            spoke,
        };
        text = write::add_zone_named(&kb, &text, &spec, &outline.pts, 0x9d0 + i as u64);
    }
    let out_path = match &p.output {
        Some(o) => PathBuf::from(o),
        None => {
            let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("board");
            path.with_file_name(format!("{stem}.gnd.kicad_pcb"))
        }
    };
    std::fs::write(&out_path, text).with_context(|| format!("writing {}", out_path.display()))?;
    external::copy_project_files(&path, &out_path);
    // hand back a filled board, so a render or KiCad shows the pours right away
    match external::refilled(&out_path).and_then(|f| std::fs::copy(&f, &out_path).map_err(anyhow::Error::from)) {
        Ok(_) => {}
        Err(e) => warnings.push(format!(
            "zones not filled ({e:#}): KiCad fills them (B), drc refills them"
        )),
    }
    Ok((
        out_path.clone(),
        GroundPourOut {
            board: String::new(),
            path: out_path.display().to_string(),
            net: nname,
            zones_added: zone_layers.iter().map(|&l| layer_name(b, l)).collect(),
            vias_added: spots.len(),
            edge_vias: edge,
            warnings,
        },
    ))
}

// --------------------------------------------------------------------------------- prompt

pub const PROMPT_NAME: &str = "good_practice";

pub const PROMPT_DESCRIPTION: &str = "Route a KiCad board end to end the way a careful layouter would: \
plan power planes and track widths from the currents first, route with Gummiband, then check current \
carrying capacity, pour ground with stitching vias and finish with KiCad's DRC.";

pub fn prompt_text(path: &str, currents: Option<&str>, notes: Option<&str>) -> String {
    let currents = currents.map(|c| format!("Known currents: {c}\n")).unwrap_or_default();
    let notes = notes
        .map(|n| format!("Notes from the designer: {n}\n"))
        .unwrap_or_default();
    format!(
        "Route the board {path} in one go with the gummiband tools and apply the usual rules of good \
PCB practice around the autorouter. Work through every step without asking back, unless a decision \
would change the design intent (moving a connector, dropping a part); then stop and ask.
{currents}{notes}
1. Look: load_board, list_nets. Note the layers and which are planes, the net classes, the \
differential pairs, the power and ground nets. part_info on connectors, regulators, fuses and \
eFuses tells you where the current enters and where it goes.

2. Currents: give every power net a continuous current (connector and fuse ratings, regulator \
outputs, LED counts, motor or load currents from the schematic and datasheets) and a short term \
current where a fuse or PTC limits a short. Without facts take the estimate power_plan makes from \
the class widths, and say so in the report. Name the current paths: source pads -> sink pads.

3. Plan before routing: power_plan with the currents and apply true. Follow its advice and board \
notes: a solid GND plane on an inner layer next to the main signal layer (add_zone over the whole \
board; on two layers keep B.Cu for a GND pour), planes or pours for nets too wide for a track \
(add_zone on an outer layer, priority above the GND pour), the datasheet layout of converters, \
eFuses and their input and output loops as locked copper (part_info, add_locked_copper), enough \
vias per pin into the planes (power_plan gives the count). Paths a track carries badly get \
power_copper with their from and to pads: the switch node of a converter from its pins to the \
inductor, a large pad of a plane net (inductor, output capacitor, connector) into its plane. \
Route the board id it returns. If you can write files, append \
power_plan's dru_rules to the project's .kicad_dru and load_board again, so KiCad's DRC enforces \
the widths too. Lock what has its place: connectors, mounting holes, power stages (pass them as \
fixed to place_parts and route). Keep the switch node of a converter \
small and the input capacitor loop short. Place parts first if they are not placed (move_part for \
connectors and power stages, place_parts with semantic true for the rest).

4. Route: route the latest board id (keep_locked when you added locked copper). Use pairs for \
differential pairs that need coupling. Read unrouted and blocked; fix with set_net_options \
(priority, width) and route again, or finish single nets with keep_all, nets and reroute.

5. Check the copper: current_check on the routed file with from and to for every current path \
of step 2. A FLAG names the neck with its position: widen the net (set_net_options width, then \
route keep_all + nets + reroute), add a pour over the neck (add_zone) or locked copper, and check \
again until every path is ok. Too few vias where the current changes layer count as a neck too.

6. Ground: ground_pour on the routed file (GND pours on the outer layers, stitching vias on a grid \
and along the edge). Use the returned file from now on.

7. Judge: drc on the final file (it refills the zones). Fix clearance violations and unconnected \
items, render the regions of power stages, necks and dense areas and look at them. angles_45 only \
if 45 degree routing is wanted, then drc again.

8. Report: a short table of the current paths (current, required width, narrowest cross-section, \
vias, verdict), what you planned and changed, the DRC counts and the path of the final file."
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rounding_up() {
        assert!((up05(0.69) - 0.7).abs() < 1e-9);
        assert!((up05(0.75) - 0.75).abs() < 1e-9);
    }

    fn smd(num: &str, x: f64, y: f64, w: f64, h: f64, net: &str) -> String {
        format!("(pad \"{num}\" smd rect (at {x} {y}) (size {w} {h}) (layers \"F.Cu\" \"F.Mask\") (net \"{net}\"))")
    }

    fn part(refdes: &str, x: f64, y: f64, pads: &[String]) -> String {
        format!(
            "(footprint \"Test:{refdes}\" (layer \"F.Cu\") (at {x} {y}) \
             (property \"Reference\" \"{refdes}\" (at 0 0) (layer \"F.Fab\")) (attr smd) {})",
            pads.join(" ")
        )
    }

    fn board(parts: &[String]) -> KicadBoard {
        let text = format!(
            "(kicad_pcb (version 20260206) (generator \"gummiband_test\") (generator_version \"10.0\")\n\
             (general (thickness 1.6)) (paper \"A4\")\n\
             (layers (0 \"F.Cu\" signal) (2 \"B.Cu\" signal) (1 \"F.Mask\" user) (3 \"B.Mask\" user) \
             (25 \"Edge.Cuts\" user) (35 \"F.Fab\" user))\n\
             (setup (pad_to_mask_clearance 0))\n{}\n\
             (gr_rect (start 100 100) (end 120 115) (stroke (width 0.1) (type default)) (fill no) (layer \"Edge.Cuts\"))\n)\n",
            parts.join("\n")
        );
        let pro = serde_json::json!({"net_settings": {"classes": [{"name": "Default", "clearance": 0.15,
            "track_width": 0.2, "via_diameter": 0.6, "via_drill": 0.3}]}});
        gummiband_kicad::read::parse_board(text, Some(&pro), &ReadOptions::default()).unwrap()
    }

    /// The pour's zones keep their clearance from the pads of other nets and hang together.
    fn assert_clean(kb: &KicadBoard, o: &PowerCopperOut) {
        let b = &kb.board;
        let f = b.layer_id("F.Cu").unwrap();
        for x in &o.pours {
            let net = b.net_id(&x.net).unwrap();
            let pieces: Vec<Polygon> = std::iter::once(&x.outline)
                .chain(x.exits.iter())
                .map(|o| Polygon::new(o.iter().map(|q| gummiband_core::geom::pt(q[0], q[1])).collect()))
                .collect();
            for (i, poly) in pieces.iter().enumerate() {
                for ob in b.obstacles.iter().filter(|ob| ob.layers.has(f) && ob.net != Some(net)) {
                    let d = poly.dist_to_poly(&ob.shape.to_polygon(0.01));
                    assert!(
                        d >= 0.15 - 2e-3,
                        "{} piece {i} {d:.3} from {:?}",
                        x.name,
                        ob.shape.bbox()
                    );
                }
                assert!(
                    pieces
                        .iter()
                        .enumerate()
                        .any(|(j, q)| j != i && q.dist_to_poly(poly) <= 0.0)
                        || pieces.len() == 1,
                    "{} piece {i} hangs alone",
                    x.name
                );
                // nor do the pours of two paths touch
                for y in o.pours.iter().filter(|y| y.net != x.net) {
                    for q in std::iter::once(&y.outline).chain(y.exits.iter()) {
                        let q = Polygon::new(q.iter().map(|q| gummiband_core::geom::pt(q[0], q[1])).collect());
                        let d = poly.dist_to_poly(&q);
                        assert!(d >= 0.15 - 2e-3, "{} piece {i} {d:.3} from {}", x.name, y.name);
                    }
                }
            }
        }
    }

    /// The pads of a 2 x 2 mm eFuse (TPS25947, VQFN-HR): two 0.3 x 2.4 mm strips for IN and OUT,
    /// 0.2 mm from each other and from the pin rows beside them.
    fn efuse(x: f64, y: f64, inp: &str, out: &str) -> String {
        let mut p = vec![smd("5", -0.25, 0.0, 0.3, 2.4, inp), smd("6", 0.25, 0.0, 0.3, 2.4, out)];
        for (s, nums) in [(-1.0, ["1", "2", "3", "4"]), (1.0, ["10", "9", "8", "7"])] {
            let n = |i: usize| format!("S{}", nums[i]);
            p.push(smd(nums[0], s * 0.9, -0.7, 0.6, 0.3, &n(0)));
            p.push(smd(nums[0], s * 0.725, -0.875, 0.25, 0.65, &n(0)));
            p.push(smd(nums[1], s * 0.9, -0.225, 0.6, 0.25, &n(1)));
            p.push(smd(nums[2], s * 0.9, 0.225, 0.6, 0.25, &n(2)));
            p.push(smd(nums[3], s * 0.9, 0.7, 0.6, 0.3, &n(3)));
            p.push(smd(nums[3], s * 0.725, 0.875, 0.25, 0.65, &n(3)));
        }
        part("U1", x, y, &p)
    }

    fn path(net: &str, from: &str, to: Option<&str>) -> PathIn {
        PathIn {
            net: net.into(),
            amps: 5.0,
            short_amps: None,
            from: Some(vec![from.into()]),
            to: to.map(|t| vec![t.into()]),
            name: None,
            layer: None,
        }
    }

    fn copper_in(kb: &KicadBoard, paths: Vec<PathIn>) -> PowerCopperOut {
        let p = PowerCopperIn {
            board: "b1".into(),
            paths,
            temp_rise: None,
            margin: None,
            max_span: None,
            via_diameter: None,
            via_drill: None,
            priority: None,
        };
        power_copper(kb, &p).unwrap().1
    }

    #[test]
    fn power_copper_leaves_an_efuse_strip_off_its_end() {
        // the shunt sits left of the eFuse, a little below its middle (lichtschnitzel R110/U6):
        // the straight way into the IN strip runs through the pin row, so one convex pour has no
        // room; the pour leaves the strip off an end, between the corner pins
        let kb = board(&[
            efuse(110.0, 107.0, "IN", "OUT"),
            part(
                "R1",
                106.0,
                107.56,
                &[smd("1", -1.06, 0.0, 1.3, 1.6, "B"), smd("2", 1.06, 0.0, 1.3, 1.6, "IN")],
            ),
        ]);
        let o = copper_in(&kb, vec![path("IN", "R1.2", Some("U1.5"))]);
        assert_eq!(o.warnings, Vec::<String>::new(), "{o:?}");
        assert_eq!(o.pours.len(), 1, "{o:?}");
        assert_eq!(o.pours[0].exits.len(), 1, "{o:?}");
        // the stub runs along the strip's axis beyond an end
        let stub = &o.pours[0].exits[0];
        assert!(stub.iter().all(|q| (q[0] - 109.75).abs() < 0.25), "{stub:?}");
        assert!(stub.iter().any(|q| (q[1] - 107.0).abs() > 1.5), "{stub:?}");
        assert_clean(&kb, &o);
    }

    #[test]
    fn power_copper_leaves_a_strip_through_a_gap_in_its_long_side() {
        // a strip with other copper close beyond both ends and along its right side, and on its
        // left side pins near the ends only: the pour comes in through the middle of the left side
        let mut u = vec![smd("1", 0.0, 0.0, 0.3, 2.4, "A")];
        for (i, y) in [-1.0, -0.55, 0.55, 1.0].into_iter().enumerate() {
            u.push(smd(&format!("{}", 10 + i), -0.65, y, 0.6, 0.3, &format!("L{i}")));
        }
        for (i, y) in [-1.0, -0.5, 0.0, 0.5, 1.0].into_iter().enumerate() {
            u.push(smd(&format!("{}", 20 + i), 0.65, y, 0.6, 0.3, &format!("R{i}")));
        }
        u.push(smd("30", 0.0, -1.6, 1.6, 0.4, "E0"));
        u.push(smd("31", 0.0, 1.6, 1.6, 0.4, "E1"));
        let kb = board(&[
            part("U1", 110.0, 107.0, &u),
            part(
                "R1",
                106.0,
                109.0,
                &[smd("1", -1.06, 0.0, 1.3, 1.6, "B"), smd("2", 1.06, 0.0, 1.3, 1.6, "A")],
            ),
        ]);
        let o = copper_in(&kb, vec![path("A", "R1.2", Some("U1.1"))]);
        assert_eq!(o.warnings, Vec::<String>::new(), "{o:?}");
        assert_eq!(o.pours[0].exits.len(), 1, "{o:?}");
        // the stub leaves to the left between the pins L1 and L2
        let stub = &o.pours[0].exits[0];
        assert!(stub.iter().any(|q| q[0] < 109.0), "{stub:?}");
        assert!(
            stub.iter().filter(|q| q[0] < 109.7).all(|q| (q[1] - 107.0).abs() < 0.4),
            "{stub:?}"
        );
        assert_clean(&kb, &o);
    }

    #[test]
    fn power_copper_brings_vias_to_an_efuse_strip() {
        // OUT goes into a plane on B.Cu: the vias sit off the ends of the strip, beside the
        // pin rows, each joined by its own pour (one convex pour is the strip only); the pour
        // of IN, laid out first, is in the way
        let mut kb = board(&[
            efuse(110.0, 107.0, "IN", "OUT"),
            part(
                "R1",
                106.0,
                107.56,
                &[smd("1", -1.06, 0.0, 1.3, 1.6, "B"), smd("2", 1.06, 0.0, 1.3, 1.6, "IN")],
            ),
        ]);
        let out = kb.board.net_id("OUT").unwrap();
        let bl = kb.board.layer_id("B.Cu").unwrap();
        kb.board.nets[out as usize].planes.insert(bl);
        let mut plane = gummiband_core::board::Plane::new(
            out,
            bl,
            Polygon::new(vec![
                gummiband_core::geom::pt(100.0, 100.0),
                gummiband_core::geom::pt(120.0, 100.0),
                gummiband_core::geom::pt(120.0, 115.0),
                gummiband_core::geom::pt(100.0, 115.0),
            ]),
        );
        plane.priority = 0;
        kb.board.planes.push(plane);
        let o = copper_in(&kb, vec![path("IN", "R1.2", Some("U1.5")), path("OUT", "U1.6", None)]);
        let x = &o.pours[1];
        assert_eq!(x.kind, "via field", "{o:?}");
        assert!(x.vias_added >= 2, "{o:?}");
        assert!(!x.exits.is_empty(), "{o:?}");
        assert_clean(&kb, &o);
    }

    #[test]
    fn prompt_names_the_board() {
        let t = prompt_text("/x/a.kicad_pcb", Some("VBUS 4 A"), None);
        assert!(t.contains("/x/a.kicad_pcb") && t.contains("VBUS 4 A") && t.contains("current_check"));
    }
}
