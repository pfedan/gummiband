# Gummiband KiCad plugin

KiCad action "Route with Gummiband" for the PCB editor (KiCad 10, IPC API plugin). It saves a
copy of the open board with its project into a temp folder, runs `gummiband route` on the copy,
and replaces the tracks and vias of the open board with the routed ones in **one undo step**.
Your board file is not written; save it yourself when you like the result.

## Install (KiCad 10)

1. Build the router: `cargo build --release` in the repository root
   (binary: `target/release/gummiband`).
2. Enable the API: KiCad, *Preferences > Plugins*, tick **Enable KiCad API**.
   On macOS and Windows keep the default Python interpreter (KiCad's bundled one, which has
   wxPython for the dialog).
3. Link (or copy) this folder into the IPC plugin directory of KiCad 10:

   | OS      | plugin directory                                  |
   | ------- | ------------------------------------------------- |
   | macOS   | `~/Documents/KiCad/10.0/plugins/`                 |
   | Linux   | `~/.local/share/kicad/10.0/plugins/`              |
   | Windows | `%USERPROFILE%\Documents\KiCad\10.0\plugins\`     |

   ```bash
   ln -s ~/code/gummiband/plugin ~/Documents/KiCad/10.0/plugins/gummiband
   ```

   This is the `plugins` folder of the IPC API, not the legacy `scripting/plugins` folder of
   the old SWIG action plugins.
4. Restart KiCad (or use the reload button in *Preferences > Plugins*). KiCad creates a
   virtual environment for the plugin
   (`~/Library/Caches/kicad/10.0/python-environments/com.github.pfedan.gummiband` on macOS),
   with access to the interpreter's own packages (`--system-site-packages`, so wxPython comes
   from KiCad's Python), and pip-installs `requirements.txt` into it, which pulls
   `kicad-python` (module `kipy`) and its dependencies from PyPI. This needs internet access
   once. The toolbar button appears when the environment is ready. If it breaks, right-click
   the action in *Preferences > Plugins* and choose *Recreate Plugin Environment*.
5. Tell the plugin where the `gummiband` binary is. It looks in this order:
   1. environment variable `GUMMIBAND_BIN` (note: KiCad started from the Finder or Dock does
      not see variables from your shell profile),
   2. `settings.json` in this folder: copy `settings.example.json` to `settings.json` and set
      `gummiband_bin` (`~` is expanded; the file is git-ignored),
   3. `PATH`, then `~/.cargo/bin`, `/opt/homebrew/bin` and `/usr/local/bin`.

## Use

PCB editor: click the Gummiband button in the plugin toolbar (the action is listed in
*Preferences > Plugins*, where its toolbar button can be switched on and off). The dialog has a
*Basic* and an *Advanced* tab; every field explains itself in a tooltip and maps to one option of
`gummiband route`, see [the plugin guide](../docs/guide/kicad-plugin.md#the-dialog).

- **Keep locked tracks and vias** (default on): locked copper stays as fixed copper the router
  connects to; all unlocked tracks and vias are replaced. Off: every track and via is replaced
  (same as `gummiband route` without `--keep-locked`).
- **Save an animation of the generations** (off by default): one image per generation and an
  animated `evolution.gif` in `gummiband-evolution/` next to the project, `--frames`. Needs
  `rsvg-convert` and ImageMagick (`brew install librsvg imagemagick`).

The dialogs remember their last values in `settings.json` (sections `route` and `place`), so keep
that file when you refresh a copy of the plugin. Fields that only fit one board (references, plane
layers, net files) start empty every time. *Defaults* brings back the values of a fresh
installation.

While the router runs, a progress dialog shows the elapsed time; *Cancel* stops the router and
leaves the board alone. At the end a message box reports what was removed and created, the
router's summary and unrouted connections. If the router fails, its stdout and stderr are shown
and the temp folder is kept (its path is in the message). Undo with Ctrl/Cmd+Z. Zones are not
refilled; refill (B) and run DRC afterwards.

Without wxPython (for example a Linux system Python), the action runs with the saved values or
the defaults and prints its report; KiCad 10.0.1 and later shows plugin output in the status bar.

## How the import works

`gb_plugin/sexpr.py` reads the routed `.kicad_pcb` (pure Python). The new items are the top
level `segment`, `arc` and `via` items of the routed file whose UUID is not in the copy that
was routed (so locked copper the CLI copied through is not created twice). Nets are matched by
name (KiCad 10 files) or through the net table (KiCad 9 files). In one commit the plugin then
deletes the replaced items of the open board (unlocked tracks, arcs and vias, or all of them)
and creates the new ones through the IPC API, in batches of 500 items.

## Files

| File | What |
| --- | --- |
| `plugin.json` | IPC API plugin manifest (schema `https://go.kicad.org/api/schemas/v1`) |
| `requirements.txt` | installed by KiCad into the plugin environment |
| `gummiband_action.py` | entrypoint: dialog, temp copy, CLI run, import, report |
| `gummiband_place.py` | entrypoint of *Place with Gummiband*: scope dialog, CLI run, one commit |
| `gb_plugin/sexpr.py` | s-expression reader |
| `gb_plugin/board_file.py` | copper items of a board file, import plan (no KiCad needed) |
| `gb_plugin/runner.py` | binary lookup, options and their saved values, command line, running the router |
| `gb_plugin/fields.py` | what the dialogs ask for: tabs, sections, fields with range and tooltip |
| `gb_plugin/kicad_apply.py` | conversion to `kipy` objects, the commits of routing and placing |
| `dev_run.py` | headless dry run of the import |
| `tests/` | unit tests |

## Development

```bash
cd plugin
python3 -m unittest            # kipy tests are skipped if kicad-python is not installed

# dry run: what would the plugin remove and create?
python3 dev_run.py board.kicad_pcb --routed routed.kicad_pcb [--keep-locked] [-v]
# or let it run the router first (binary lookup as in the plugin)
python3 dev_run.py board.kicad_pcb --route --runs 4 [--keep-locked]
```

To run the tests and the dry run with the plugin's real environment (Python 3.9 with
`kipy`, which also checks the conversion to API objects), use the interpreter of the plugin
venv, e.g. `~/Library/Caches/kicad/10.0/python-environments/com.github.pfedan.gummiband/bin/python`.

Debugging inside KiCad: start KiCad with `KICAD_ALLOC_CONSOLE=1 KICAD_ENABLE_WXTRACE=1
WXTRACE=KICAD_API` to see the API trace and the plugin's stderr.

## Finish selected nets (follow-me)

The second button, **Finish selected nets with Gummiband**, routes what is missing on the nets of
the selected tracks, vias or pads. A started track is continued from its copper. Unlocked tracks
of other nets shift aside where the new connection needs the room (`--shove`); locked ones stay,
and what the new track does not touch stays exactly as it is. If there is no way even so, the
tracks of nearby nets are routed again (`--give`). Everything arrives as one undo step. After
vias moved, refill the zones before you run the DRC.
