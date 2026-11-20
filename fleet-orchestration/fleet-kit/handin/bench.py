"""Multi-seed benchmark: our planner vs the greedy baseline, in parallel.

    python3 bench.py --levels L1,L2,L3,L4 --seeds 100-109 --budget 20 --jobs 8
    python3 bench.py --levels L4 --seeds 0-4 --set time_price=0.05     # try a parameter

Seeds 0-99 are for tuning, 100+ for held-out checks. Each scenario runs in its own process with
the given time budget (no baseline fallback), so the numbers show the pipeline alone.
"""
import argparse
import multiprocessing as mp
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

from fleetkit.baseline import greedy_plan  # noqa: E402
from fleetkit.generator import generate  # noqa: E402
from fleetkit.model import Scenario  # noqa: E402
from fleetkit.scorer import evaluate  # noqa: E402


def seeds(text):
    out = []
    for part in text.split(","):
        if "-" in part:
            lo, hi = part.split("-", 1)
            out.extend(range(int(lo), int(hi) + 1))
        else:
            out.append(int(part))
    return out


def run_one(args):
    level, seed, budget, overrides = args
    from fleetplan import params
    for k, v in overrides.items():
        params.LEVELS[level][k] = v
    from fleetplan.pipeline import solve
    data = generate(level, seed)
    sc = Scenario.from_dict(data)
    t = time.monotonic()
    log = {}
    try:
        plan, score = solve(sc, t, budget, log)
        valid = plan is not None and evaluate(sc, plan).valid
    except Exception as exc:  # report, do not hide
        plan, score, valid = None, 0.0, False
        log["error"] = repr(exc)
    took = time.monotonic() - t
    base = evaluate(sc, greedy_plan(sc)).score
    return level, seed, score if valid else 0.0, base, took, valid, log


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--levels", default="L1,L2,L3,L4")
    ap.add_argument("--seeds", default="100-104")
    ap.add_argument("--budget", type=float, default=20.0)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--set", action="append", default=[], help="param=value override")
    a = ap.parse_args()
    overrides = {}
    for kv in a.set:
        k, v = kv.split("=", 1)
        overrides[k] = float(v) if "." in v else int(v)
    tasks = [(lv, s, a.budget, overrides) for lv in a.levels.split(",") for s in seeds(a.seeds)]
    with mp.Pool(a.jobs) as pool:
        results = pool.map(run_one, tasks, chunksize=1)
    per = {}
    for level, seed, score, base, took, valid, log in results:
        flag = "" if valid else "  INVALID"
        err = f"  {log['error']}" if "error" in log else ""
        print(f"{level}-{seed:<4} ours {score:8.1f}  base {base:8.1f}  x{score / base if base else 0:5.2f}"
              f"  {took:5.1f}s  it={log.get('iterations', 0)}{flag}{err}")
        p = per.setdefault(level, [0.0, 0.0, 0])
        p[0] += score
        p[1] += base
        p[2] += 0 if valid else 1
    tot_o = sum(v[0] for v in per.values())
    tot_b = sum(v[1] for v in per.values())
    for level, (o, b, bad) in sorted(per.items()):
        print(f"{level}: ours {o:9.1f}  base {b:9.1f}  x{o / b if b else 0:5.2f}  invalid {bad}")
    print(f"TOTAL: ours {tot_o:.1f}  base {tot_b:.1f}  x{tot_o / tot_b if tot_b else 0:.3f}")


if __name__ == "__main__":
    main()
