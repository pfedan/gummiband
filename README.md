# Gummiband

Topological ("rubber band") autorouter for KiCad: any-angle tracks with arcs, planned as topology
first and pulled taut afterwards. Rust core, uses all CPU cores, KiCad plugin on top.

Status: early development, see [docs/MILESTONES.md](docs/MILESTONES.md) and
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

```bash
cargo build --release
./target/release/gummiband info board.kicad_pcb
```
