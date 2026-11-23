"""Bit-exact regression test for refactors.

Runs the planner in deterministic mode (virtual clock, fixed iterations) on a fixed set of cases
and compares each plan's SHA-256 with a recorded snapshot. Any behavioural change shows up.

    python3 tests/regression.py --record   # snapshot the current behaviour
    python3 tests/regression.py            # check: every case must match the snapshot
"""
import argparse
import hashlib
import json
import multiprocessing as mp
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HANDIN = os.path.dirname(HERE)
KIT = os.path.dirname(HANDIN)
SNAPSHOT = os.path.join(HERE, "regression_snapshot.json")

# (name, level, seed, extra environment, verify mode)
CASES = [
    ("L1-0", "L1", 0, {}, False), ("L1-1", "L1", 1, {}, False),
    ("L2-0", "L2", 0, {}, False), ("L2-1", "L2", 1, {}, False),
    ("L3-0", "L3", 0, {}, False), ("L3-1", "L3", 1, {}, False),
    ("L4-0", "L4", 0, {}, False), ("L4-1", "L4", 1, {}, False),
    ("L2-0-verify", "L2", 0, {}, True),
    ("L3-0-knapsack-weak", "L3", 0, {"FLEET_KNAPSACK": "1", "FLEET_WEAK_ROUTE": "1"}, False),
    ("L3-1-ground", "L3", 1, {"FLEET_GROUND_RUN": "1"}, False),
    ("L4-2-seed7", "L4", 2, {"FLEET_SEED": "7"}, False),
]

RUNNER = r"""
import json, sys
sys.path.insert(0, sys.argv[1]); sys.path.insert(0, sys.argv[2])
from fleetkit.generator import generate
from fleetkit.model import Scenario
from fleetkit.scorer import evaluate
from fleetplan import clock
from fleetplan.pipeline import solve
level, seed, verify = sys.argv[3], int(sys.argv[4]), sys.argv[5] == "1"
sc = Scenario.from_dict(generate(level, seed))
clock.CLOCK.reset()
plan, score = solve(sc, clock.now(), 10.0, {}, verify=verify)
valid = plan is not None and evaluate(sc, plan).valid
print(json.dumps({"plan": plan, "score": score, "valid": valid}, sort_keys=True))
"""


def run_case(case):
    name, level, seed, extra, verify = case
    env = dict(os.environ, FLEET_DETERMINISTIC="1", PYTHONHASHSEED="0", **extra)
    out = subprocess.run([sys.executable, "-c", RUNNER, HANDIN, KIT, level, str(seed), "1" if verify else "0"],
                         env=env, capture_output=True, text=True)
    if out.returncode != 0:
        return name, {"error": out.stderr.strip().splitlines()[-1] if out.stderr else "crash"}
    res = json.loads(out.stdout.strip().splitlines()[-1])
    digest = hashlib.sha256(json.dumps(res["plan"], sort_keys=True).encode()).hexdigest()
    return name, {"sha256": digest, "score": round(res["score"], 6), "valid": res["valid"]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--record", action="store_true")
    ap.add_argument("--jobs", type=int, default=6)
    a = ap.parse_args()
    with mp.Pool(a.jobs) as pool:
        results = dict(pool.map(run_case, CASES))
    if a.record:
        with open(SNAPSHOT, "w") as f:
            json.dump(results, f, indent=1, sort_keys=True)
        for name, r in results.items():
            print(f"recorded {name:<20} {r}")
        return 0
    snap = json.load(open(SNAPSHOT))
    bad = 0
    for name, _, _, _, _ in CASES:
        got, want = results[name], snap.get(name)
        ok = got == want
        bad += not ok
        print(f"{'OK  ' if ok else 'DIFF'} {name:<20} score {got.get('score')}  (snapshot {want.get('score') if want else None})")
    print("ALL IDENTICAL" if not bad else f"{bad} case(s) differ")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
