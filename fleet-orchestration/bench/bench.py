"""Fast benchmark harness for the fleet planner (standard library only).

Runs the hand-in exactly like check_handin.py (fresh copy of handin/, `python3 planner.py`, 60 s limit,
official validator) but in parallel, longest scenarios first, with a cache keyed by the CONTENT of handin/:
re-running a version you already measured is instant, and so is the baseline.

  python3 bench.py run                        # working tree, suite "val" (8 public scenarios)
  python3 bench.py run HEAD~1 --suite unseen40
  python3 bench.py compare HEAD wt --suite unseen40   # per-scenario deltas, regressions first
  python3 bench.py run wt --levels L4 --seeds 1000-1019 --jobs 6

Refs: "wt" = current working tree, anything else = a git ref (commit, branch, tag).
Suites: val (8) | quick (L1-L4 x 0-4, 20) | unseen40 (L1-L4 x 0-9) | wide (300 seeds >= 1000) | l4 (L4 x 1000-1019)
The planner is time-driven: two runs of the same code can differ by a few points on L4 (use --force to re-measure).
"""
from __future__ import annotations

import argparse, hashlib, json, os, shutil, subprocess, sys, tempfile, time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

BENCH = Path(__file__).resolve().parent
KIT = BENCH.parent / "fleet-kit"
HANDIN = KIT / "handin"
CACHE = BENCH / ".cache"
sys.path.insert(0, str(KIT))
from fleetkit.baseline import greedy_plan  # noqa: E402
from fleetkit.generator import generate  # noqa: E402
from fleetkit.model import Scenario, save_json  # noqa: E402
from fleetkit.scorer import evaluate  # noqa: E402
from fleetkit.validation import validation_ids  # noqa: E402

LEVELS = ("L1", "L2", "L3", "L4")
SUITES = {
    "val": validation_ids(),
    "quick": [f"{l}-{s}" for l in LEVELS for s in range(5)],
    "unseen40": [f"{l}-{s}" for l in LEVELS for s in range(10)],
    "wide": [f"L1-{s}" for s in range(1000, 1100)] + [f"L2-{s}" for s in range(1000, 1100)]
            + [f"L3-{s}" for s in range(1000, 1060)] + [f"L4-{s}" for s in range(1000, 1040)],
    "l4": [f"L4-{s}" for s in range(1000, 1020)],
}


def sh(*cmd: str) -> str:
    return subprocess.run(cmd, check=True, stdout=subprocess.PIPE, cwd=BENCH).stdout.decode().strip()


def tree_hash(d: Path) -> str:
    h = hashlib.sha256()
    for p in sorted(d.rglob("*")):
        if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc":
            h.update(str(p.relative_to(d)).encode()); h.update(b"\0"); h.update(p.read_bytes())
    return h.hexdigest()[:12]


def resolve(ref: str) -> tuple[Path, str, str]:
    """Returns (handin source dir, code hash, label) for 'wt' or a git ref."""
    if ref == "wt":
        return HANDIN, tree_hash(HANDIN), "wt"
    sha = sh("git", "rev-parse", "--verify", f"{ref}^{{commit}}")
    src = CACHE / "src" / sha
    if not (src / "planner.py").exists():
        top = Path(sh("git", "rev-parse", "--show-toplevel"))
        rel = HANDIN.relative_to(top).as_posix()
        tmp = Path(tempfile.mkdtemp(prefix="bench-src-"))
        arch = subprocess.run(["git", "archive", sha, rel], check=True, stdout=subprocess.PIPE, cwd=top).stdout
        subprocess.run(["tar", "-x", "-C", str(tmp)], input=arch, check=True)
        src.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(tmp / rel), str(src)); shutil.rmtree(tmp, ignore_errors=True)
    return src, tree_hash(src), (sha[:7] if sha.startswith(ref) else f"{ref}@{sha[:7]}")


def scenario(name: str) -> dict:
    p = CACHE / "scen" / f"{name}.json"
    if p.exists():
        return json.loads(p.read_text())
    level, seed = name.split("-")
    data = generate(level, int(seed))
    p.parent.mkdir(parents=True, exist_ok=True); save_json(data, p)
    return data


FK_HASH = tree_hash(KIT / "fleetkit")


def baseline(name: str) -> float:
    p = CACHE / "base" / FK_HASH / f"{name}.json"
    if p.exists():
        return json.loads(p.read_text())["score"]
    sc = Scenario.from_dict(scenario(name))
    s = evaluate(sc, greedy_plan(sc)).score
    p.parent.mkdir(parents=True, exist_ok=True); p.write_text(json.dumps({"score": s}))
    return s


def result_path(code: str, name: str, timeout: float) -> Path:
    return CACHE / "results" / code / f"{name}__t{timeout:g}.json"


def run_one(src: Path, code: str, name: str, timeout: float) -> dict:
    data = scenario(name)
    base = baseline(name)
    with tempfile.TemporaryDirectory(prefix="bench-") as tmp:
        work = Path(tmp) / "team"
        shutil.copytree(src, work, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        save_json(data, work / "scenario.json")
        env = {"PATH": os.environ.get("PATH", ""), "PYTHONPATH": str(KIT), "PYTHONDONTWRITEBYTECODE": "1", "HOME": tmp}
        t0 = time.monotonic()
        res = dict(name=name, base=base, score=0.0, note="")
        try:
            p = subprocess.run([sys.executable, "planner.py", "scenario.json", "plan.json"], cwd=work, env=env,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
            res["secs"] = time.monotonic() - t0
            plan_p = work / "plan.json"
            if p.returncode != 0:
                res.update(status="crash", note=" | ".join(p.stdout.decode(errors="replace").strip().splitlines()[-3:])[:300])
            elif not plan_p.exists():
                res.update(status="no_plan")
            else:
                try:
                    plan = json.loads(plan_p.read_text())
                except ValueError as e:
                    res.update(status="bad_json", note=str(e)[:100])
                else:
                    r = evaluate(Scenario.from_dict(data), plan)
                    if r.valid:
                        res.update(status="ok", score=r.score)
                    else:
                        res.update(status="invalid", note="; ".join(str(v) for v in r.violations[:3])[:300])
        except subprocess.TimeoutExpired:
            res.update(status="timeout", secs=timeout, note=f"> {timeout:g} s")
    out = result_path(code, name, timeout)
    out.parent.mkdir(parents=True, exist_ok=True); out.write_text(json.dumps(res))
    return res


def names_from(a) -> list[str]:
    if a.seeds:
        lo_hi = a.seeds.split("-")
        seeds = range(int(lo_hi[0]), int(lo_hi[-1]) + 1) if "-" in a.seeds else [int(x) for x in a.seeds.split(",")]
        return [f"{l}-{s}" for l in (a.levels or ",".join(LEVELS)).split(",") for s in seeds]
    names = SUITES[a.suite]
    return [n for n in names if n.split("-")[0] in a.levels.split(",")] if a.levels else names


def measure(refs: list[str], names: list[str], jobs: int, timeout: float, force: bool) -> list[tuple[str, dict]]:
    """Runs every (ref, scenario) not in cache, in ONE pool, L4 first. Returns [(label, {name: result})]."""
    resolved = [resolve(r) for r in refs]
    todo, out, queued = [], [], set()
    for src, code, label in resolved:
        got = {}
        for n in names:
            p = result_path(code, n, timeout)
            if p.exists() and not force:
                got[n] = json.loads(p.read_text())
            elif (code, n) not in queued:          # same code under two refs: measure once
                queued.add((code, n))
                todo.append((src, code, n, label))
        out.append((label, code, got))
    todo.sort(key=lambda t: (-int(t[2][1]), t[2]))
    if todo:
        print(f"{len(todo)} runs to do, {jobs} jobs", file=sys.stderr)
        for n in {t[2] for t in todo}:      # scenarios + baselines up front, single-threaded
            baseline(n)
        t0 = time.monotonic()
        with ThreadPoolExecutor(jobs) as ex:
            futs = {ex.submit(run_one, src, code, n, timeout): (code, n, label) for src, code, n, label in todo}
            for k, f in enumerate(as_completed(futs), 1):
                code, n, label = futs[f]
                r = f.result()
                for lab, c, got in out:
                    if c == code:
                        got[n] = r
                flag = "" if r["status"] == "ok" else f"  <-- {r['status'].upper()} {r['note'][:80]}"
                print(f"[{k:3d}/{len(todo)}] {label:>14s} {n:8s} {r['score']:8.1f} (base {r['base']:7.1f}) {r['secs']:5.1f}s{flag}",
                      file=sys.stderr)
        print(f"done in {time.monotonic() - t0:.0f} s", file=sys.stderr)
    return [(label, got) for label, _, got in out]


def table(label: str, res: dict, names: list[str]) -> int:
    print(f"\n== {label}")
    print(f"{'lvl':4s} {'n':>4s} {'valid':>6s} {'score':>10s} {'baseline':>10s} {'ratio':>7s} {'t_med':>6s} {'t_max':>6s}")
    tot = base = 0.0
    fails = []
    for l in LEVELS:
        rs = [res[n] for n in names if n.startswith(l + "-")]
        if not rs:
            continue
        s, b = sum(r["score"] for r in rs), sum(r["base"] for r in rs)
        tot += s; base += b
        ts = sorted(r["secs"] for r in rs)
        ok = sum(r["status"] == "ok" for r in rs)
        fails += [r for r in rs if r["status"] != "ok"]
        print(f"{l:4s} {len(rs):4d} {ok:4d}/{len(rs):<2d}{s:9.1f} {b:10.1f} {s / b if b else 0:6.3f}x {ts[len(ts) // 2]:6.1f} {ts[-1]:6.1f}")
    print(f"{'ALL':4s} {len(names):4d} {len(names) - len(fails):4d}/{len(names):<2d}{tot:9.1f} {base:10.1f} {tot / base if base else 0:6.3f}x")
    for r in fails:
        print(f"  FAIL {r['name']}: {r['status']} {r['note']}")
    return len(fails)


def compare(la: str, a: dict, lb: str, b: dict, names: list[str], top: int) -> None:
    print(f"\n== compare  A={la}  ->  B={lb}")
    print(f"{'lvl':4s} {'A':>10s} {'B':>10s} {'delta':>8s} {'better':>7s} {'worse':>6s} {'same':>5s}")
    for l in LEVELS + ("ALL",):
        ns = [n for n in names if l == "ALL" or n.startswith(l + "-")]
        if not ns:
            continue
        d = [b[n]["score"] - a[n]["score"] for n in ns]
        print(f"{l:4s} {sum(a[n]['score'] for n in ns):10.1f} {sum(b[n]['score'] for n in ns):10.1f} {sum(d):+8.1f} "
              f"{sum(x > 0.05 for x in d):7d} {sum(x < -0.05 for x in d):6d} {sum(abs(x) <= 0.05 for x in d):5d}")
    deltas = sorted((b[n]["score"] - a[n]["score"], n) for n in names)
    worst = [x for x in deltas if x[0] < -0.05][:top]
    best = [x for x in reversed(deltas) if x[0] > 0.05][:top]
    if worst:
        print("  regressions: " + ", ".join(f"{n} {d:+.1f}" for d, n in worst))
    if best:
        print("  gains:       " + ", ".join(f"{n} {d:+.1f}" for d, n in best))
    for n in names:
        if (a[n]["status"] == "ok") != (b[n]["status"] == "ok"):
            print(f"  VALIDITY CHANGE {n}: {a[n]['status']} -> {b[n]['status']} {b[n]['note']}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["run", "compare"])
    ap.add_argument("refs", nargs="*", help="'wt' or git refs (run: 1, default wt; compare: 2)")
    ap.add_argument("--suite", default="val", choices=sorted(SUITES))
    ap.add_argument("--levels", help="filter, e.g. L3,L4")
    ap.add_argument("--seeds", help="explicit seeds, e.g. 0-9 or 3,7 (overrides --suite)")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2), help="default: physical cores")
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--force", action="store_true", help="ignore cached results, re-measure")
    ap.add_argument("--top", type=int, default=8)
    a = ap.parse_args()
    refs = a.refs or ["wt"]
    if a.cmd == "compare" and len(refs) != 2:
        ap.error("compare needs exactly two refs, e.g. compare HEAD wt")
    if a.cmd == "run" and len(refs) != 1:
        ap.error("run takes one ref")
    names = names_from(a)
    measured = measure(refs, names, a.jobs, a.timeout, a.force)
    fails = 0
    for label, res in measured:
        fails = table(label, res, names)
    if a.cmd == "compare":
        (la, ra), (lb, rb) = measured
        compare(la, ra, lb, rb, names, a.top)
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
