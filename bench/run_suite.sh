#!/bin/sh
# Route every synthetic benchmark board in bench/boards and judge the results with the KiCad DRC.
#
# Usage: bench/run_suite.sh [board ...]          (default: all boards in bench/boards)
# Environment:
#   RUNS=4 GENERATIONS=1     router portfolio size and evolutionary generations
#   GEOMETRY_GENERATIONS=0   geometry generations (0 keeps the suite quick and comparable)
#   RAYON_NUM_THREADS=4      worker threads per router run (boards always run one after another)
#   TIMEOUT=900              seconds per board for the router (killed afterwards)
#   ROUTE_ARGS="..."         extra arguments for `gummiband route`
#   REGEN=1                  regenerate the boards with gen_boards.py first
#   SVG=1                    also write a debug SVG per board
# Output: bench/boards/out/<board>.kicad_pcb (+ .kicad_pro copy, .route.log, .drc.json),
# and the results table on stdout and in bench/boards/out/results.txt.
set -u
export PATH=/opt/homebrew/opt/rustup/bin:$PATH
export RAYON_NUM_THREADS=${RAYON_NUM_THREADS:-4}
BENCH=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$BENCH/.." && pwd)
if [ "${REGEN:-0}" = 1 ] || ! ls "$BENCH"/boards/*.kicad_pcb >/dev/null 2>&1; then
    python3 "$BENCH/gen_boards.py" "$BENCH/boards" || exit 1
fi
echo "building gummiband-cli ..."
(cd "$ROOT" && cargo build --release -q -p gummiband-cli) || { echo "build failed"; exit 1; }
exec python3 - "$ROOT" "$@" <<'PY'
import collections, json, os, re, shutil, signal, subprocess, sys, time

root, wanted = sys.argv[1], sys.argv[2:]
boards_dir = os.path.join(root, "bench", "boards")
out = os.path.join(boards_dir, "out")
os.makedirs(out, exist_ok=True)
gb = os.path.join(root, "target", "release", "gummiband")
runs = os.environ.get("RUNS", "4")
gens = os.environ.get("GENERATIONS", "1")
geo = os.environ.get("GEOMETRY_GENERATIONS", "0")
timeout = float(os.environ.get("TIMEOUT", "900"))
extra = os.environ.get("ROUTE_ARGS", "").split()

# Suite order (by difficulty); boards not listed here follow alphabetically.
ORDER = ["easy", "medium", "bus", "power_mix", "crossing", "qfn_fanout", "four_signal_layers", "bga_escape", "bga_dense"]
found = sorted(f[:-10] for f in os.listdir(boards_dir) if f.endswith(".kicad_pcb"))
names = [n for n in ORDER if n in found] + [n for n in found if n not in ORDER]
if wanted:
    names = [n for n in names if n in wanted]
    missing = [w for w in wanted if w not in found]
    if missing:
        print("unknown boards:", " ".join(missing))

rows, notes = [], []
for name in names:
    src = os.path.join(boards_dir, name + ".kicad_pcb")
    dst = os.path.join(out, name + ".kicad_pcb")
    log = os.path.join(out, name + ".route.log")
    drc = os.path.join(out, name + ".drc.json")
    for f in (dst, drc):
        if os.path.exists(f):
            os.remove(f)
    shutil.copy(os.path.join(boards_dir, name + ".kicad_pro"), os.path.join(out, name + ".kicad_pro"))
    dru = os.path.join(boards_dir, name + ".kicad_dru")
    if os.path.exists(dru):
        shutil.copy(dru, os.path.join(out, name + ".kicad_dru"))
    elif os.path.exists(os.path.join(out, name + ".kicad_dru")):
        os.remove(os.path.join(out, name + ".kicad_dru"))
    cmd = [gb, "route", src, "-o", dst, "--runs", runs, "--generations", gens, "--geometry-generations", geo] + extra
    if os.environ.get("SVG") == "1":
        cmd += ["--svg", os.path.join(out, name + ".svg")]
    row = dict(board=name, status="ok", routed="-", vias="-", length="-", time="-", core="-",
               unconn="-", clearance="-", short="-", crossing="-", other="-")
    print(f"[{name}] {' '.join(cmd)}", flush=True)
    t0 = time.time()
    try:
        p = subprocess.run(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           timeout=timeout, text=True, errors="replace")
        text, rc = p.stdout, p.returncode
    except subprocess.TimeoutExpired as e:
        text = (e.stdout or b"").decode(errors="replace") if isinstance(e.stdout, bytes) else (e.stdout or "")
        rc = None
    dt = time.time() - t0
    with open(log, "w") as f:
        f.write(" ".join(cmd) + "\n" + text)
    row["time"] = f"{dt:.1f}"
    m = re.search(r"best: seed \S+ (\d+)/(\d+) routed, (\d+) vias, ([\d.]+) mm", text)
    if m:
        row.update(routed=f"{m.group(1)}/{m.group(2)}", vias=m.group(3), length=m.group(4))
    m = re.search(r"clearance check: (\d+) violations", text)
    if m:
        row["core"] = m.group(1)
    if rc is None:
        row["status"] = f"timeout {timeout:.0f}s"
    elif rc != 0:
        sig = -rc if rc < 0 else None
        row["status"] = f"signal {signal.Signals(sig).name}" if sig else f"exit {rc}"
    if row["status"] != "ok" or not os.path.exists(dst):
        if row["status"] == "ok":
            row["status"] = "no output"
        tail = [l for l in text.splitlines() if l.strip()][-6:]
        panic = [l for l in text.splitlines() if "panicked" in l or l.startswith("Error")]
        notes.append(f"{name}: {row['status']} after {dt:.1f}s ({' '.join(cmd)})\n    "
                     + "\n    ".join(panic[:3] or tail))
        rows.append(row)
        print(f"[{name}] FAILED: {row['status']} (log: {log})", flush=True)
        continue
    unrouted = [l.strip() for l in text.splitlines() if l.strip().startswith("unrouted:")]
    if unrouted:
        notes.append(f"{name}: " + "; ".join(u[len('unrouted: '):] for u in unrouted[:8])
                     + (f" ... ({len(unrouted)} total)" if len(unrouted) > 8 else ""))
    try:
        subprocess.run(["kicad-cli", "pcb", "drc", "--refill-zones", "--format", "json", "-o", drc, dst],
                       stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       timeout=600, cwd=out)
        d = json.load(open(drc))
        c = collections.Counter(v["type"] for v in d["violations"] if not v["type"].startswith(("silk", "lib_")))
        row["unconn"] = str(len(d["unconnected_items"]))
        row["clearance"] = str(c.pop("clearance", 0))
        row["short"] = str(c.pop("shorting_items", 0))
        row["crossing"] = str(c.pop("tracks_crossing", 0))
        row["other"] = str(sum(c.values()))
        if c:
            notes.append(f"{name}: other DRC: " + ", ".join(f"{k} {v}" for k, v in sorted(c.items())))
    except Exception as e:  # DRC failure must not stop the suite
        row["unconn"] = "drc failed"
        notes.append(f"{name}: kicad-cli drc failed: {e}")
    rows.append(row)
    print(f"[{name}] {row['routed']} routed, {row['vias']} vias, {row['time']} s, "
          f"DRC unconnected {row['unconn']}", flush=True)

cols = [("board", "board"), ("status", "status"), ("routed", "routed"), ("vias", "vias"),
        ("length", "length mm"), ("time", "time s"), ("core", "core viol"), ("unconn", "unconn"),
        ("clearance", "clearance"), ("short", "short"), ("crossing", "crossing"), ("other", "other")]
wid = {k: max(len(h), *(len(r[k]) for r in rows)) if rows else len(h) for k, h in cols}
lines = ["| " + " | ".join(h.ljust(wid[k]) for k, h in cols) + " |",
         "|" + "|".join("-" * (wid[k] + 2) for k, _ in cols) + "|"]
for r in rows:
    lines.append("| " + " | ".join(r[k].ljust(wid[k]) for k, _ in cols) + " |")
report = "\n".join(lines)
if notes:
    report += "\n\nNotes:\n" + "\n".join("- " + n for n in notes)
report += (f"\n\nrouter: --runs {runs} --generations {gens} --geometry-generations {geo} {' '.join(extra)}, RAYON_NUM_THREADS={os.environ.get('RAYON_NUM_THREADS')};"
           " time = wall clock of `gummiband route`;"
           "\ncore viol = Gummiband's own clearance check; unconn/clearance/short/crossing/other = kicad-cli pcb drc"
           " (silk*/lib_* ignored)")
print()
print(report)
with open(os.path.join(out, "results.txt"), "w") as f:
    f.write(report + "\n")
PY
