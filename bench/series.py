#!/usr/bin/env python3
"""A queue of open ended router runs, side by side on a fixed core budget, judged by the KiCad DRC.

Usage: bench/series.py QUEUE_DIR [--cores 12] [--repo DIR]

QUEUE_DIR/jobs.jsonl holds one job per line and is read again whenever a slot is free, so jobs
can be added while the series runs. A job:

    {"id": "arty-main-s1", "commit": "519a2a1", "board": "arty", "seed": 1, "threads": 4,
     "limit": 1800, "args": ["--geometry-generations", "3"], "env": {"GB_X": "1"}}

- `commit` is built once into QUEUE_DIR/bin/<commit> from a clone of the repository in
  QUEUE_DIR/src (fetched from origin); `bin` instead names a binary that exists.
- `board`: arty, corne, ulx3s or ledctl (prepared under bench/work of `--repo`), or a
  path to a .kicad_pcb.
- The run is `route --until-stop --progress --seed SEED ARGS` with RAYON_NUM_THREADS=threads and
  GB_OPEN_LOG=1. It is stopped after `limit` seconds of wall time, or when it reports
  convergence if `stop_converged` is true, or ends by itself with GB_OPEN_GENERATIONS in `env`.

Per job QUEUE_DIR/runs/<id>/ gets progress.jsonl (every progress line with its wall time),
log.txt (stderr), stdout.txt, the routed board, drc.json and result.json. A job with a
result.json is done and not run again, so the series can be restarted. QUEUE_DIR/status.json
shows what runs. A file QUEUE_DIR/STOP lets the running jobs finish and starts no new ones.
"""
import argparse
import collections
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time

IGNORE = ("silk", "lib_", "copper_sliver", "isolated_copper", "padstack", "courtyard", "text_")


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


def board_path(repo, board):
    if board.endswith(".kicad_pcb"):
        return board
    if board == "ledctl":
        return os.path.join(repo, "bench", "work", "ledctl", "ledctl.kicad_pcb")
    return os.path.join(repo, "bench", "work", "ext", board, board + ".kicad_pcb")


def copy_project(src, dst_dir):
    os.makedirs(dst_dir, exist_ok=True)
    stem = os.path.splitext(src)[0]
    name = os.path.basename(stem)
    for ext in (".kicad_pro", ".kicad_dru"):
        if os.path.exists(stem + ext):
            shutil.copy(stem + ext, os.path.join(dst_dir, name + ext))
    return os.path.join(dst_dir, name + ".kicad_pcb")


def drc(pcb):
    """(unconnected, Counter of violation types) or (None, {}) without kicad-cli."""
    if not shutil.which("kicad-cli"):
        return None, collections.Counter()
    rep = os.path.join(os.path.dirname(pcb), "drc.json")
    subprocess.run(["kicad-cli", "pcb", "drc", "--refill-zones", "--format", "json", "-o", rep, pcb],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        d = json.load(open(rep))
    except (OSError, ValueError):
        return None, collections.Counter()
    c = collections.Counter(v["type"] for v in d["violations"] if not v["type"].startswith(IGNORE))
    return len(d["unconnected_items"]), c


class Queue:
    def __init__(self, qdir, repo, cores):
        self.q = os.path.abspath(qdir)
        self.repo = os.path.abspath(repo)
        self.cores = cores
        self.running = {}
        self.base = {}
        for d in ("bin", "runs", "base"):
            os.makedirs(os.path.join(self.q, d), exist_ok=True)

    def jobs(self):
        out = []
        try:
            for line in open(os.path.join(self.q, "jobs.jsonl")):
                line = line.strip()
                if line and not line.startswith("#"):
                    try:
                        out.append(json.loads(line))
                    except ValueError:
                        log(f"bad job line: {line[:80]}")
        except OSError:
            pass
        return out

    def done(self, job):
        return os.path.exists(os.path.join(self.q, "runs", job["id"], "result.json"))

    def binary(self, job):
        if "bin" in job:
            return job["bin"]
        commit = job["commit"]
        out = os.path.join(self.q, "bin", commit)
        if os.path.exists(out):
            return out
        src = os.path.join(self.q, "src")
        if not os.path.exists(src):
            url = subprocess.run(["git", "-C", self.repo, "remote", "get-url", "origin"],
                                 capture_output=True, text=True).stdout.strip()
            subprocess.run(["git", "clone", "-q", url, src], check=True)
        subprocess.run(["git", "-C", src, "fetch", "-q", "origin"], check=True)
        subprocess.run(["git", "-C", src, "checkout", "-q", "--detach", commit], check=True)
        log(f"build {commit}")
        p = subprocess.run(["cargo", "build", "--release", "-q", "-j", "2", "-p", "gummiband-cli"], cwd=src,
                           capture_output=True, text=True)
        if p.returncode != 0:
            log(f"build of {commit} failed: {p.stderr[-2000:]}")
            return None
        shutil.copy(os.path.join(src, "target", "release", "gummiband"), out)
        return out

    def base_drc(self, pcb):
        if pcb not in self.base:
            d = os.path.join(self.q, "base", os.path.basename(os.path.dirname(pcb)))
            cached = os.path.join(d, "base.json")
            if os.path.exists(cached):
                self.base[pcb] = collections.Counter(json.load(open(cached)))
            else:
                b = copy_project(pcb, d)
                shutil.copy(pcb, b)
                _, c = drc(b)
                json.dump(c, open(cached, "w"))
                self.base[pcb] = c
        return self.base[pcb]

    def start(self, job):
        gb = self.binary(job)
        if gb is None:
            self.finish_failed(job, "build failed")
            return
        src = board_path(self.repo, job["board"])
        d = os.path.join(self.q, "runs", job["id"])
        if os.path.exists(d):
            shutil.rmtree(d)
        pcb = copy_project(src, d)
        cmd = [gb, "route", src, "-o", pcb, "--until-stop", "--progress", "--report",
               os.path.join(d, "report.json")]
        if job.get("seed") is not None:
            cmd += ["--seed", str(job["seed"])]
        cmd += [str(a) for a in job.get("args", [])]
        env = dict(os.environ)
        env["RAYON_NUM_THREADS"] = str(job.get("threads", 4))
        env["GB_OPEN_LOG"] = "1"
        env.update({k: str(v) for k, v in job.get("env", {}).items()})
        json.dump({"job": job, "cmd": cmd}, open(os.path.join(d, "job.json"), "w"), indent=1)
        err = open(os.path.join(d, "log.txt"), "w")
        p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=err,
                             text=True, bufsize=1, env=env)
        run = {"job": job, "p": p, "dir": d, "pcb": pcb, "src": src, "t0": time.time(),
               "last": None, "converged_at": None, "stopped": None, "reason": None, "err": err,
               "rusage": None}
        t = threading.Thread(target=self.reader, args=(run,), daemon=True)
        t.start()
        run["thread"] = t
        self.running[job["id"]] = run
        log(f"start {job['id']} ({job.get('threads', 4)} threads)")

    def reader(self, run):
        prog = open(os.path.join(run["dir"], "progress.jsonl"), "w")
        out = open(os.path.join(run["dir"], "stdout.txt"), "w")
        for line in run["p"].stdout:
            out.write(line)
            if line.startswith("progress "):
                try:
                    v = json.loads(line[9:])
                except ValueError:
                    continue
                v["t"] = round(time.time() - run["t0"], 2)
                prog.write(json.dumps(v) + "\n")
                prog.flush()
                run["last"] = v
                if v.get("converged") and run["converged_at"] is None:
                    run["converged_at"] = v["t"]
        out.close()
        prog.close()

    def stop(self, run, reason):
        if run["stopped"] is None:
            run["stopped"] = time.time() - run["t0"]
            run["reason"] = reason
            try:
                run["p"].stdin.write("stop\n")
                run["p"].stdin.close()
            except (OSError, ValueError):
                pass
            log(f"stop {run['job']['id']}: {reason}")

    def poll(self):
        for jid, run in list(self.running.items()):
            job, p = run["job"], run["p"]
            el = time.time() - run["t0"]
            if run["stopped"] is None:
                if el > job.get("limit", 1800):
                    self.stop(run, "limit")
                elif job.get("stop_converged") and run["converged_at"] is not None:
                    self.stop(run, "converged")
                elif os.path.exists(os.path.join(self.q, "KILL")):
                    self.stop(run, "kill")
            elif el - run["stopped"] > job.get("finish_limit", 1200):
                p.kill()
            try:
                pid, status, ru = os.wait4(p.pid, os.WNOHANG)
            except ChildProcessError:
                pid, status, ru = p.pid, 0, None
            if pid == 0:
                continue
            p.returncode = os.waitstatus_to_exitcode(status) if ru is not None else 0
            run["rusage"] = ru
            run["thread"].join(timeout=30)
            run["err"].close()
            del self.running[jid]
            self.finish(run, el)

    def finish_failed(self, job, why):
        d = os.path.join(self.q, "runs", job["id"])
        os.makedirs(d, exist_ok=True)
        json.dump({"id": job["id"], "job": job, "failed": why}, open(os.path.join(d, "result.json"), "w"))

    def finish(self, run, wall):
        job = run["job"]
        res = {"id": job["id"], "job": job, "exit": run["p"].returncode, "wall": round(wall, 1),
               "stopped": run["stopped"], "reason": run["reason"] or "ended",
               "converged_at": run["converged_at"], "last": run["last"]}
        ru = run["rusage"]
        if ru is not None:
            res["cpu"] = round(ru.ru_utime + ru.ru_stime, 1)
            res["maxrss_mb"] = round(ru.ru_maxrss / (1 << 20 if sys.platform == "darwin" else 1 << 10))
        out = open(os.path.join(run["dir"], "stdout.txt")).read() if os.path.exists(
            os.path.join(run["dir"], "stdout.txt")) else ""
        m = re.search(r"clearance check: (\d+) violations at (\d+) spots", out)
        res["own_check"] = int(m.group(1)) if m else None
        res["own_spots"] = int(m.group(2)) if m else None
        try:
            rep = json.load(open(os.path.join(run["dir"], "report.json")))
            res["final"] = {"routed": rep.get("routed"), "conns": rep.get("connections"),
                            "vias": rep.get("vias"), "length": rep.get("length_mm"),
                            "violations": len(rep.get("violations", [])), "seconds": rep.get("seconds")}
        except (OSError, ValueError):
            res["final"] = None
        if os.path.exists(run["pcb"]) and res["exit"] == 0:
            pre = self.base_drc(run["src"])
            unconn, c = drc(run["pcb"])
            kinds = {k: v - pre.get(k, 0) for k, v in c.items() if v - pre.get(k, 0) > 0}
            res["drc"] = {"unconnected": unconn, "errors": sum(kinds.values()), "kinds": kinds}
        json.dump(res, open(os.path.join(run["dir"], "result.json"), "w"), indent=1)
        f = res.get("final") or {}
        log(f"done {job['id']}: {f.get('routed')}/{f.get('conns')} {f.get('vias')} vias "
            f"{(f.get('length') or 0):.0f} mm, drc {res.get('drc')}, {wall:.0f} s")

    def status(self):
        st = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "running": {}}
        for jid, run in self.running.items():
            st["running"][jid] = {"elapsed": round(time.time() - run["t0"]), "last": run["last"],
                                  "stopped": run["stopped"]}
        tmp = os.path.join(self.q, "status.json.tmp")
        json.dump(st, open(tmp, "w"), indent=1)
        os.replace(tmp, os.path.join(self.q, "status.json"))

    def serve(self):
        log(f"series in {self.q}, {self.cores} cores")
        while True:
            self.poll()
            stop = os.path.exists(os.path.join(self.q, "STOP"))
            if not stop:
                used = sum(r["job"].get("threads", 4) for r in self.running.values())
                for job in self.jobs():
                    if job["id"] in self.running or self.done(job):
                        continue
                    if used + job.get("threads", 4) > self.cores:
                        break
                    self.start(job)
                    used += job.get("threads", 4)
            self.status()
            if stop and not self.running:
                log("stopped")
                return
            time.sleep(5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("queue")
    ap.add_argument("--cores", type=int, default=12)
    ap.add_argument("--repo", default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    a = ap.parse_args()
    Queue(a.queue, a.repo, a.cores).serve()


if __name__ == "__main__":
    sys.exit(main())
