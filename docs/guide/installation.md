# Installation

Gummiband is one binary (`gummiband`) plus a thin KiCad plugin that calls it. Build the binary
first, then set up whichever front end you use.

## 1. Build the router

You need a Rust toolchain, Rust 1.85 or newer (the crates use edition 2024). Install it from
<https://rustup.rs> if you do not have one.

```bash
git clone https://github.com/pfedan/gummiband
cd gummiband
cargo build --release
```

The binary is `target/release/gummiband`. The first build downloads the dependencies and takes a
few minutes; later builds are quick. Check it:

```bash
target/release/gummiband --help
```

To update, `git pull` and build again. The plugin picks up the new binary on its next run.

Optional, macOS only: `cargo build --release -p gummiband-cli --features gpu` builds a binary that
can run the track relaxation on the GPU when started with `GB_GPU=1`. It is experimental and
behaves exactly like the normal binary without the variable, see the
[command line](cli.md#environment-variables).

The MCP server for AI agents is a separate binary, built with
`cargo build --release -p gummiband-mcp`. Its setup is in [MCP.md](../MCP.md).

## 2. KiCad plugin

The plugin is an IPC API action plugin for KiCad 10 (Python). It adds four buttons to the PCB
editor's plugin toolbar: *Route with Gummiband*, *Finish selected nets with Gummiband*, *45
degree tracks with Gummiband* and *Place with Gummiband*.

1. **Enable the API.** In KiCad, *Preferences > Plugins*, tick *Enable KiCad API*. On macOS and
   Windows keep KiCad's bundled Python interpreter, which has wxPython for the dialog.
2. **Link the plugin folder** into KiCad's IPC plugin directory (not the old `scripting/plugins`
   folder of SWIG plugins):

   | OS | plugin directory |
   | --- | --- |
   | macOS | `~/Documents/KiCad/10.0/plugins/` |
   | Linux | `~/.local/share/kicad/10.0/plugins/` |
   | Windows | `%USERPROFILE%\Documents\KiCad\10.0\plugins\` |

   ```bash
   ln -s "$PWD/plugin" ~/Documents/KiCad/10.0/plugins/gummiband     # macOS
   ln -s "$PWD/plugin" ~/.local/share/kicad/10.0/plugins/gummiband  # Linux
   ```

   On Windows copy the folder or create a directory junction.
3. **Point the plugin at the binary.** Copy `plugin/settings.example.json` to
   `plugin/settings.json` and set the path:

   ```json
   { "gummiband_bin": "~/code/gummiband/target/release/gummiband" }
   ```

   The file is git-ignored. Alternatives: the environment variable `GUMMIBAND_BIN` (KiCad
   started from the Finder or Dock does not see variables from your shell profile), or a
   `gummiband` on `PATH`, in `~/.cargo/bin`, `/opt/homebrew/bin` or `/usr/local/bin`.
4. **Restart KiCad** or press the reload button in *Preferences > Plugins*. KiCad creates a
   Python environment for the plugin and installs `kicad-python` from PyPI into it; this needs
   internet access once. The toolbar buttons appear when the environment is ready.

Routing from the plugin needs nothing else: no `kicad-cli`, no extra Python packages.

## 3. Optional tools

| Tool | Needed for | Install |
| --- | --- | --- |
| `kicad-cli` | DRC from scripts and benchmarks, the MCP `drc` and `render` tools | ships with KiCad (`/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli` on macOS) |
| `rsvg-convert` (librsvg) | PNG images of the generations (`--frames`, plugin animation), MCP `render` | `brew install librsvg`, `apt install librsvg2-bin` |
| ImageMagick (`magick`) | the animated `evolution.gif` of the generations | `brew install imagemagick`, `apt install imagemagick` |
| Python 3 | benchmark scripts, plugin tests | usually present |

Without them the router works the same; only the images and the DRC shortcuts are missing.

## Troubleshooting

**No Gummiband buttons in KiCad.** Check that the API is enabled, that the folder sits in the
IPC plugin directory of the right KiCad version, and that *Preferences > Plugins* lists the
actions with their toolbar buttons switched on. If the plugin environment failed to build (no
internet, a PyPI hiccup), right-click the action there and choose *Recreate Plugin Environment*.

**"gummiband binary not found".** Set `gummiband_bin` in `plugin/settings.json` to the absolute
path of the built binary (a `~` is expanded). Make sure you built in release mode; the plugin
does not look in `target/debug`.

**The dialog does not appear and the router runs with defaults.** The plugin's Python has no
wxPython (typical for a Linux system interpreter). Select KiCad's bundled interpreter in
*Preferences > Plugins*, or route from the [command line](cli.md) where every option is a flag.

**Routing fails and a message shows stdout and stderr.** The temp folder with the exported board
is kept; its path is in the message. Run `gummiband route` on that copy from a shell to see the
full output, and open an issue with it if the router itself crashed.

**Slow or hot machine.** The router uses every core by default. Limit it with the *Parallel runs*
field of the dialog, or on the command line with `--threads N` (worker threads) and `--runs N`
(parallel attempts). The MCP server uses 4 threads unless `RAYON_NUM_THREADS` says otherwise.

**Build errors about the edition.** Your Rust is too old. `rustup update stable` and build again.
