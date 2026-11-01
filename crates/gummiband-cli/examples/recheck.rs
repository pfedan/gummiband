//! The own clearance check on a routed file, to hold it against the KiCad DRC of the same file.
//! Usage: recheck BOARD.kicad_pcb
use std::collections::HashMap;

use gummiband_core::check;
use gummiband_core::geom::pt;
use gummiband_core::realize::{Elem, Geometry, Track, Via, flatten};
use gummiband_kicad::{KeepCopper, ReadOptions, read_board};

fn main() {
    let path = std::env::args().nth(1).expect("board file");
    let kb = read_board(
        path.as_ref(),
        &ReadOptions {
            keep: KeepCopper::None,
            ..Default::default()
        },
    )
    .unwrap();
    let b = &kb.board;
    let names: HashMap<&str, u32> = b
        .nets
        .iter()
        .enumerate()
        .map(|(i, n)| (n.name.as_str(), i as u32))
        .collect();
    let numbered: HashMap<i64, u32> = kb
        .net_numbers
        .iter()
        .flatten()
        .filter_map(|(name, &num)| Some((num, *names.get(name.as_str())?)))
        .collect();
    let net = |n: &gummiband_kicad::sexpr::Node| -> Option<u32> {
        let a = n.child("net")?.args();
        match (a.first(), a.get(1)) {
            (_, Some(s)) => names.get(s.as_str()?).copied(),
            (Some(f), None) => f
                .as_f64()
                .and_then(|x| numbered.get(&(x as i64)).copied())
                .or_else(|| names.get(f.as_str()?).copied()),
            _ => None,
        }
    };
    let xy = |n: &gummiband_kicad::sexpr::Node, k: &str| {
        let v = n.child_f64s(k)?;
        Some(pt(*v.first()?, *v.get(1)?))
    };
    let mut geo = Geometry {
        tracks: Vec::new(),
        vias: Vec::new(),
    };
    for n in kb.root.args() {
        let Some(id) = net(n) else { continue };
        match n.head() {
            Some("via") => {
                let size = n.child_f64("size").unwrap_or(0.6);
                geo.vias.push(Via {
                    at: xy(n, "at").unwrap(),
                    diameter: size,
                    drill: n.child_f64("drill").unwrap_or(size / 2.0),
                    net: id,
                    layer: None,
                    origin: None,
                    resting: false,
                });
            }
            Some(kind @ ("segment" | "arc")) => {
                let Some(layer) = n.child_str("layer").and_then(|l| b.layer_id(l)) else {
                    continue;
                };
                let (s, e) = (xy(n, "start").unwrap(), xy(n, "end").unwrap());
                let elems = if kind == "arc" {
                    vec![Elem::Arc(s, xy(n, "mid").unwrap(), e)]
                } else {
                    vec![Elem::Line(s, e)]
                };
                let width = n.child_f64("width").unwrap_or(0.2);
                geo.tracks.push(Track {
                    pts: flatten(&elems),
                    elems,
                    width,
                    layer,
                    net: id,
                    necks: Vec::new(),
                    neck_w: width,
                    origin: None,
                    resting: false,
                    fitted: Vec::new(),
                    pattern: None,
                    seg: u32::MAX,
                    tap: None,
                });
            }
            _ => {}
        }
    }
    let v = check::check(b, &geo);
    let spots = check::spots(&v);
    println!(
        "{} tracks, {} vias: {} violations at {} spots",
        geo.tracks.len(),
        geo.vias.len(),
        v.len(),
        spots.len()
    );
    let mut kinds: HashMap<&str, usize> = HashMap::new();
    for x in &v {
        *kinds.entry(x.what.split(' ').next().unwrap_or("")).or_default() += 1;
    }
    println!("{kinds:?}");
    for s in spots.iter().take(12) {
        let name = |n: Option<u32>| n.map_or("-", |n| b.nets[n as usize].name.as_str());
        println!(
            "  {:.2} {:.2} {} ~ {}: {:.3} missing",
            s.at.x,
            s.at.y,
            name(Some(s.nets.0)),
            name(s.nets.1),
            s.missing
        );
    }
}
