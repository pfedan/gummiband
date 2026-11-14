//! How whole a pour of one net would be on a routed file: a pour over the whole board on every
//! copper layer, laid by the fill model on the pads, tracks and vias of the file, and its
//! pieces (`gummiband_core::ground`).
//! Usage: ground BOARD.kicad_pcb [NET] (default GND)
use gummiband_core::fill::Options;
use gummiband_core::ground::pour_wholeness;
use gummiband_kicad::{KeepCopper, ReadOptions, read_board};

fn main() {
    let path = std::env::args().nth(1).expect("board file");
    let name = std::env::args().nth(2).unwrap_or_else(|| "GND".into());
    let kb = read_board(
        path.as_ref(),
        &ReadOptions {
            keep: KeepCopper::All,
            ..Default::default()
        },
    )
    .unwrap();
    let b = &kb.board;
    let Some(net) = b.nets.iter().position(|n| n.name == name) else {
        eprintln!("no net {name}");
        std::process::exit(1);
    };
    let layers: Vec<u8> = (0..b.layers.len() as u8).collect();
    for w in pour_wholeness(b, net as u32, &layers, &Options::default()) {
        println!(
            "{} {}: {} pieces, {:.0} mm2, {:.1} % in the largest, {} pads apart",
            name,
            b.layers[w.layer as usize].name,
            w.pieces,
            w.area,
            100.0 * w.share(),
            w.pads_apart
        );
    }
}
