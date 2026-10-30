use gummiband_core::topo::{Substrate, SubstrateConfig};
use gummiband_kicad::{KeepCopper, ReadOptions, read_board};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let kb = read_board(
        args[1].as_ref(),
        &ReadOptions {
            keep: KeepCopper::None,
            ..Default::default()
        },
    )
    .unwrap();
    let (x, y): (f64, f64) = (args[2].parse().unwrap(), args[3].parse().unwrap());
    let p = gummiband_core::geom::pt(x, y);
    let sub = Substrate::build(&kb.board, &SubstrateConfig::default());
    for (li, m) in sub.m.iter().enumerate() {
        for (v, q) in m.verts.iter().enumerate() {
            if q.dist(p) < 0.6 && li == 1 {
                let st: Vec<u32> = m.vert_tris[v].iter().map(|&t| m.status[t as usize]).collect();
                println!("li {li} v{v} at {:.4},{:.4} statuses {:?}", q.x, q.y, st);
                for &s in &st {
                    if s < u32::MAX - 2 {
                        let o = &kb.board.obstacles[s as usize];
                        println!(
                            "   obs {s}: {:?} layers {:?} net {:?} pin {:?}",
                            o.kind,
                            o.layers,
                            o.net.map(|n| kb.board.nets[n as usize].name.clone()),
                            o.pin.map(|q| kb.board.pins[q].name.clone())
                        );
                    }
                }
            }
        }
    }
}
