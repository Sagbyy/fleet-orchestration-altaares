# ALNS fleet planner

`python3 planner.py scenario.json plan.json` — standard library only, about 30–40 s per scenario,
always writes a valid plan.

## Pipeline

```
scenario.json
   │
   ▼
0. Preprocessing (world.py, geom.py)
   • drop contacts that can never pay: R7 collateral, windows too short, needs the fleet lacks,
     hostiles whose points do not cover their effectors (10 pts each + energy)
   • obstacles as circles: protected areas (any altitude) and zones (below their ceiling)
   • observation ring per contact: points 125 m away, outside every obstacle
     (valid in 3D up to z = 70 m since √(125² + 70²) < 150 m)
   • per effector × hostile: earliest arrival and energy at best-range speed
   │
   ▼
1. Construction (alns.py)
   • regret-2 insertion of observations into scout tours (TOPTW)
   • exact timing per tour: windows, 20 s dwell, return by the horizon, battery
   • observation point chosen on the ring where the tour passes (cuts 2×125 m per visit)
   │
   ▼
2. Effector matching (alns.match), on every evaluated solution
   • richest hostiles first; per subtype, the slowest effector that still arrives in time
   • engagement time T = max(observation end + 10 s, window start, arrivals), all-or-nothing
   │
   ▼
3. ALNS (alns.run), until ~75 % of the budget or 20k iterations without a new best
   • destroy: random, worst contribution, related (space + time), whole tour, sector
   • repair: regret-2, regret-2 with noise, greedy with noise
   • simulated-annealing acceptance, adaptive operator weights (roulette)
   │
   ▼
4. 3D realisation and deconfliction (realize.py)
   • scouts at 30 or 70 m, effectors cruise at 150 or 110 m (over every zone at 150 m)
   • detours around obstacles along circumscribed polygons; final dive from ~250 m out
   • ground waiting is free: effectors take off just in time, fly at best-range speed
   • scouts fly legs at best-range speed whenever the next windows allow it
   • each route is checked against the committed ones with the official R8 test; on conflict,
     try another altitude, time shift, landing delay or approach angle
   │
   ▼
5. Safety net (pipeline.py, planner.py)
   • official validator; drop whatever it flags (effector group, scout + dependent groups)
   • drop scout observations nobody engages, when that stays valid
   • compare with the greedy baseline and write the better valid plan; never crash
   │
   ▼
plan.json
```

## Results (public generator, held-out seeds 0–19, 10 s budget)

| Level | Ours | Baseline | Ratio |
|---|---:|---:|---:|
| L1 | 2002 | 1920 | ×1.04 |
| L2 | 5077 | 4554 | ×1.11 |
| L3 | 9750 | 7710 | ×1.26 |
| L4 | 22256 | 10578 | ×2.10 |
| **Total** | **39084** | **24762** | **×1.58** |

80/80 plans valid. In L1 and L2 the plan already serves every reachable contact; what remains is
energy. In L4 the limits are scout battery and time to reach far contacts before their window closes.

## Round 2 (after an independent code review)

- baseline computed first (valid fallback in hand), the optimiser gets the rest of the budget
- scouts and effectors climb and land diagonally (R2 only bounds intermediate waypoints);
  observation ring 143 m at 30 m, pulled to 128 m when a scout flies at 70 m; speeds at 99 %
- leg-length cache and bounding-box filters: ~3x more ALNS iterations on L4
- effector groups: cheapest variants first (low layer, later arrival), retry with the fastest
  effectors when a group cannot be placed
- private copy of the official R8 test (no dependency on fleetkit internals)
- fixed an overflow in the annealing acceptance

Paired comparison on seeds 1000-1039 (160 scenarios, 10 s): +5.2 % overall, +7.8 % on L4.
Official-style run (check_handin, seeds 0-1): 4898 vs 4498 before, 0 failures, L4 in 39 s.

## Round 3

- Hungarian algorithm (fleetplan/hungarian.py, checked against brute force) for the final
  effector assignment: same contacts, minimum energy, arrival deadlines as hard constraints
- contact ordering by value per effector tested (FLEET_MATCH_ORDER=density): +1.9 % then -1.5 %
  on two seed sets, within noise, so value ordering is kept
- finer R8 repair not built: the validator-driven repair never triggered on 160 scenarios

Paired on seeds 1000-1039 (160 scenarios, 10 s): +6.3 % over the first version, x1.85 vs baseline
(L4 +9.4 %, x2.63). Validation set: 4937 vs 2665 for the baseline.

## Ground-level effector runs: disabled on purpose

R2 only bounds the altitude of intermediate waypoints, so a two-waypoint effector route
[home at z = 0, engage at z = 0] passes the validator while crawling kilometres at ground level.
That is a loophole, not flight: it is off by default (FLEET_GROUND_RUN=1 re-enables it for
comparison; worth about +0.4 %). We report it to the organisers instead of exploiting it.

## OR-Tools cross-check (offline, not shipped)

Google OR-Tools routing (guided local search, 60-120 s) on a simplified scout-routing model of
L4-401 and L4-402 scored only 2-3 % above our ALNS, and its tours became infeasible once replayed
in our exact timing model (repaired: well below the ALNS). The ALNS tours are close to what an
industrial solver finds on this problem.

## Parameter tuning

Swept on L3/L4 seeds 0–9 (10 s budget): `time_price`, `regret_w`, `q_frac`, `t0_frac`, `k_routes`.
L3 is insensitive (the search converges). In L4 a smaller destroy (`q_frac` 0.10–0.15) gained ~2 %
at 10 s, but at the real budget (33 s) on held-out seeds 100–107 the default 0.25 stayed best
(10015 vs 9867 vs 9697), so the defaults are kept: tuning on short runs did not carry over.

## Verify board (optional)

`FLEET_VERIFY=1 python3 planner.py ...` adds a scout re-observation 10–120 s after each engagement
when the +25 % bonus beats the extra energy and the airspace stays clear. It lowers the main score
slightly, so the hand-in leaves it off. On the 8 validation scenarios: verify score 4861 with it.

## Tools

```sh
python3 bench.py --levels L1,L2,L3,L4 --seeds 0-19 --budget 10 --jobs 8   # parallel benchmark
python3 bench.py --levels L4 --seeds 0-4 --set time_price=0.05             # try a parameter
python3 check_handin.py                                                   # official-style run
```

Per-level parameters live in `fleetplan/params.py`. `FLEET_BUDGET` (seconds) overrides the run time.
