use gummiband_kicad::{KeepCopper, ReadOptions, read_board};
fn main() {
    let p = std::env::args().nth(1).unwrap();
    let kb = read_board(
        p.as_ref(),
        &ReadOptions {
            keep: KeepCopper::None,
            ..Default::default()
        },
    )
    .unwrap();
    for n in &kb.board.nets {
        if n.name.contains("DATA") {
            println!("{} -> {}", n.name, kb.board.classes[n.class as usize].name);
        }
    }
    let t = std::fs::read_to_string(std::path::Path::new(&p).with_extension("kicad_dru")).unwrap();
    let r = gummiband_kicad::dru::parse_rules(&t);
    println!(
        "{} rules, pairs {}",
        r.len(),
        gummiband_kicad::dru::pair_clearances(&kb.board, &r).len()
    );
}
