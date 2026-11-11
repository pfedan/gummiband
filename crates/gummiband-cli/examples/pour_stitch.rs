//! Bench tool: write a board with a pour of one net over the board outline and one locked via
//! per island the fill model finds, so that the result can be filled by KiCad and held against
//! the model (`bench/areas-kicad.sh`). The MCP tool `ground_pour` does the same for a user; this
//! is the same step for a shell script. The report itself is `gummiband areas`.
//!
//! Usage: pour_stitch BOARD.kicad_pcb --net NAME --pour LAYER[,LAYER] -o OUT.kicad_pcb
//!   --zone-clearance MM, --zone-min-width MM, --thermal-gap MM, --thermal-spoke-width MM,
//!   --pad-connection thermal|solid|thermal_tht
//!                    settings of the pour, as for `gummiband areas` (0.2, 0.2, 0.3, 0.3, thermal)
//!   --no-vias        the pour only
use std::path::PathBuf;

use gummiband_app::AreaOptions;
use gummiband_app::areas::{model, parse_connect};
use gummiband_core::board::{LayerSet, Obstacle, ObstacleKind};
use gummiband_core::fill::{self, Connect};
use gummiband_core::geom::{Point, Shape};
use gummiband_kicad::{KeepCopper, ReadOptions, read_board, write};

fn main() -> anyhow::Result<()> {
    let mut o = AreaOptions {
        doubtful: false,
        ..Default::default()
    };
    let (mut board, mut out, mut vias) = (PathBuf::new(), PathBuf::new(), true);
    let mut it = std::env::args().skip(1);
    let list = |s: String| s.split(',').map(str::to_string).collect::<Vec<_>>();
    while let Some(x) = it.next() {
        let mut val = || it.next().unwrap_or_else(|| panic!("{x} needs a value"));
        let num = |s: String| s.parse::<f64>().unwrap_or_else(|_| panic!("{s}: not a number"));
        match x.as_str() {
            "--net" => o.nets = list(val()),
            "--pour" => o.pour = list(val()),
            "--zone-clearance" => o.zone_clearance = num(val()),
            "--zone-min-width" => o.zone_min_width = num(val()),
            "--thermal-gap" => o.thermal_gap = num(val()),
            "--thermal-spoke-width" => o.thermal_spoke_width = num(val()),
            "--pad-connection" => o.pad_connection = val(),
            "--plane-layers" => o.plane_layers = list(val()),
            "--no-vias" => vias = false,
            "-o" => out = val().into(),
            s if s.starts_with('-') => panic!("unknown option {s}"),
            _ => board = x.into(),
        }
    }
    if board.as_os_str().is_empty() || out.as_os_str().is_empty() || o.pour.is_empty() {
        panic!(
            "usage: pour_stitch BOARD.kicad_pcb --net NAME --pour LAYERS -o OUT.kicad_pcb (see the head of the source)"
        );
    }
    let kb = read_board(
        &board,
        &ReadOptions {
            keep: KeepCopper::All,
            plane_layers: o.plane_layers.clone(),
            ..Default::default()
        },
    )?;
    let m = model(&kb, &o)?;
    let mut b = kb.board.clone();
    let net = m.nets[0];
    let c = b.class_of(net).clone();
    let pitch = c.via_diameter + b.rules.hole_to_hole.max(c.clearance);
    // one via per island with a site, and the second via where it lands on a piece apart; then
    // again on the board with those vias, until no island has a site left
    let mut items: Vec<write::Item> = Vec::new();
    let mut rounds: Vec<usize> = Vec::new();
    while vias && rounds.len() < 8 {
        let f = fill::fill_net(&b, &m.areas, net, &m.options);
        let mut placed: Vec<Point> = Vec::new();
        let sites = f.islands.iter().filter_map(|i| i.via.as_ref());
        for at in sites.flat_map(|v| v.anchor.into_iter().chain([v.at])) {
            if placed.iter().any(|q| q.dist(at) < pitch) {
                continue;
            }
            placed.push(at);
            b.obstacles.push(Obstacle {
                shape: Shape::Circle {
                    c: at,
                    r: c.via_diameter / 2.0,
                },
                layers: LayerSet::all(b.layers.len()),
                net: Some(net),
                kind: ObstacleKind::Via,
                hole: Some(c.via_drill / 2.0),
                clearance: c.clearance,
                pin: None,
                locked: true,
            });
            items.push(write::Item::Via {
                at,
                diameter: c.via_diameter,
                drill: c.via_drill,
                net,
            });
        }
        rounds.push(placed.len());
        if placed.is_empty() {
            break;
        }
    }
    let mut text = if items.is_empty() {
        kb.text.clone()
    } else {
        write::add_locked(&kb, &items)
    };
    let outline = b.main_outline().expect("outline").pts.clone();
    for (i, &(net, layer)) in m.poured.iter().enumerate() {
        let before = text.len();
        let spec = write::ZoneSpec {
            net,
            layer,
            priority: 0,
            name: "",
            connect: match parse_connect(&o.pad_connection)? {
                Connect::Solid => write::PadConnect::Solid,
                Connect::ThermalTht => write::PadConnect::ThermalTht,
                _ => write::PadConnect::Thermal,
            },
            spoke: o.thermal_spoke_width,
        };
        let with = write::add_zone_named(&kb, &text, &spec, &outline, 0xa7ea + i as u64);
        // the writer has fixed settings: put the ones asked for into the zone it added
        let at = with.len() - (with.len() - before) - 2;
        let (head, zone) = with.split_at(at.min(with.len()));
        let zone = zone
            .replace("(clearance 0.2)", &format!("(clearance {})", o.zone_clearance))
            .replace("(min_thickness 0.2)", &format!("(min_thickness {})", o.zone_min_width))
            .replace("(thermal_gap 0.3)", &format!("(thermal_gap {})", o.thermal_gap));
        text = format!("{head}{zone}");
    }
    std::fs::write(&out, text)?;
    gummiband_app::copy_project_files(&board, &out);
    println!(
        "wrote {} with {} areas and {} stitching vias (per round: {:?})",
        out.display(),
        m.poured.len(),
        items.len(),
        rounds
    );
    Ok(())
}
