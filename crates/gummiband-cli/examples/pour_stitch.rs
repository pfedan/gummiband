//! Bench tool: write a board with a pour of the nets given (one net over the board outline,
//! several nets each around its own pads) and one locked via per island the fill model finds, so that the result can be filled by KiCad and held against
//! the model (`bench/areas-kicad.sh`). The MCP tool `ground_pour` does the same for a user; this
//! is the same step for a shell script. The report itself is `gummiband areas`.
//!
//! Usage: pour_stitch BOARD.kicad_pcb --net NAME[,NAME] --pour LAYER[,LAYER] -o OUT.kicad_pcb
//!   --zone-clearance MM, --zone-min-width MM, --thermal-gap MM, --thermal-spoke-width MM,
//!   --pad-connection thermal|solid|thermal_tht
//!                    settings of the pour, as for `gummiband areas` (0.2, 0.2, 0.3, 0.3, thermal)
//!   --no-vias        the pour only
use std::path::PathBuf;

use gummiband_app::AreaOptions;
use gummiband_app::areas::{model, parse_connect, stitch_islands};
use gummiband_core::fill::Connect;
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
    // one via per island with a site, in rounds, as `ground_pour` with stitch islands
    let mut items: Vec<write::Item> = Vec::new();
    if vias {
        for &net in &m.nets {
            let c = b.class_of(net).clone();
            for at in stitch_islands(&mut b, &m.areas, net, &m.options) {
                items.push(write::Item::Via {
                    at,
                    diameter: c.via_diameter,
                    drill: c.via_drill,
                    net,
                });
            }
        }
    }
    let mut text = if items.is_empty() {
        kb.text.clone()
    } else {
        write::add_locked(&kb, &items)
    };
    let connect = match parse_connect(&o.pad_connection)? {
        Connect::Solid => write::PadConnect::Solid,
        Connect::ThermalTht => write::PadConnect::ThermalTht,
        _ => write::PadConnect::Thermal,
    };
    for (i, &k) in m.poured.iter().enumerate() {
        let a = &m.areas[k];
        let spec = write::ZoneSpec {
            connect,
            spoke: a.spoke_width,
            clearance: a.clearance,
            min_width: a.min_width,
            thermal_gap: a.thermal_gap,
            ..write::ZoneSpec::plain(a.net, a.layer)
        };
        text = write::add_zone_named(&kb, &text, &spec, &a.rings[0].pts, 0xa7ea + i as u64);
    }
    std::fs::write(&out, text)?;
    gummiband_app::copy_project_files(&board, &out);
    println!(
        "wrote {} with {} zones and {} stitching vias",
        out.display(),
        m.poured.len(),
        items.len()
    );
    Ok(())
}
