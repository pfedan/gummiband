# Gummiband

Topological ("rubber band") autorouter for KiCad: any-angle tracks with arcs, planned as topology
first and pulled taut afterwards. Rust core, uses all CPU cores, KiCad plugin on top.

Status: early development, see [docs/MILESTONES.md](docs/MILESTONES.md) and
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

```bash
cargo build --release
./target/release/gummiband info board.kicad_pcb
```

## KiCad plugin

`plugin/` holds the KiCad 10 action "Route with Gummiband" (IPC API, Python): link it into
KiCad's plugin folder (`ln -s "$PWD/plugin" ~/Documents/KiCad/10.0/plugins/gummiband` on macOS),
enable *Preferences > Plugins > Enable KiCad API*, and point it at the binary with
`GUMMIBAND_BIN` or `plugin/settings.json`. Details in [plugin/README.md](plugin/README.md).
