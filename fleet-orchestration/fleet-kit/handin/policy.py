"""Link-loss policy: dead reckoning, event-triggered messages, an R4 gate and a safety shield.

Every aircraft carries the whole plan, so a peer that has said nothing is assumed to fly it exactly
(dead reckoning on its route clock). Messages only carry what the plan cannot tell:

    {"k": [tau, rate]}   I left the plan: my route clock is `tau` now and moves at `rate` (1, 0.5, 0, -1)
    {"o": ["H05"]}       an observation of H05 is complete (scouts send it, everybody relays it once)

Decisions, each second:
    * an effector never engages a reclassified target, nor one whose observation it cannot prove
      finished `confirm_margin_s` earlier (it holds one step short of the target and waits);
    * an aircraft whose route ahead crosses a known pop-up zone turns back (a scout first finishes
      the observations it can still make before the zone);
    * a shield then picks the first action, in order of preference, that keeps separation from the
      predicted peers and clear of the zones over a short horizon.
"""
import math

from fleetkit.geometry import dist3, segment_enters_cylinder
from fleetkit.linkloss.sim import RouteClock
from fleetkit.model import EFFECTOR, SCOUT, parse_plan

HORIZON_DEVIATED_S = 6.0   # look-ahead of an aircraft that is off its plan
IMMINENT_S = 2.0           # what must be safe when nothing is safe over the full horizon
HORIZON_ON_PLAN_S = 4.0    # an aircraft on its plan only reacts to imminent conflicts
SAMPLE_S = 0.25
SEP_MARGIN_M = 10.0        # on top of d_sep (also covers motion between samples)
ZONE_SOFT_M = 25.0         # keep this far from a known zone edge when looking ahead
ZONE_HARD_M = 3.0          # never plan a step that comes closer than this
EXEMPT_MARGIN_M = 25.0     # only count on the R8 exemption well inside its radius
ZONE_ABORT_S = 8.0         # a scout turns back this long before reaching a zone
REPEAT = 2                 # sends of a fresh state or observation (5 % base loss)
REANNOUNCE_S = 6.0         # while off plan, repeat the state this often...
REANNOUNCE_RANGE_M = 700.0  # ...when a peer is this close
REANNOUNCE_CLOSE_S = 6.0   # and this often when a peer is really close
REANNOUNCE_CLOSE_M = 300.0
REFACT_S = 4.0             # resend a confirmation this often...
REFACT_RANGE_M = 700.0     # ...while an effector still waiting for it is this close
APPROACH_S = 30.0          # an effector this close (in route time) to its target needs the confirmation soon
CONFIRM_SLACK_S = 0.5
MAX_BYTES = 64
EPS = 1e-6
INF = float("inf")


class Track:
    """A belief about one route clock: tau(t) = tau_ref + rate * (t - t_ref)."""

    __slots__ = ("clock", "t_ref", "tau_ref", "rate")

    def __init__(self, clock):
        self.clock = clock
        self.t_ref, self.tau_ref, self.rate = 0.0, 0.0, 1.0

    def set(self, t, tau, rate):
        self.t_ref, self.tau_ref, self.rate = float(t), float(tau), float(rate)

    @property
    def deviated(self):
        return self.rate != 1.0 or abs(self.tau_ref - self.t_ref) > 1e-3

    def tau_at(self, t):
        return self.tau_ref + self.rate * (t - self.t_ref)

    def pos_at(self, t):
        """Predicted position at real time t, or None when on the ground, engaged or landed."""
        tau = self.tau_at(t)
        if tau <= self.clock.start + EPS or tau >= self.clock.end - EPS:
            return None
        return self.clock.position(tau)


class Policy:
    def __init__(self, scenario, plan, aircraft_id):
        routes, _ = parse_plan(plan)
        aircraft = scenario.aircraft_by_id()
        rules = scenario.rules
        self.id = aircraft_id
        self.contacts = scenario.contact_by_id()
        self.sep = rules.d_sep_m + SEP_MARGIN_M
        self.exempt_r = rules.exempt_radius_m
        self.confirm = rules.confirm_margin_s
        self.tracks = {}
        self.targets = {}
        planned_obs = {}
        for aid, route in routes.items():
            self.tracks[aid] = Track(RouteClock(route))
            last = route.waypoints[-1]
            is_eff = aircraft[aid].type == EFFECTOR
            self.targets[aid] = last.target if (last.action == "engage" and is_eff) else None
            if aircraft[aid].type == SCOUT:
                for wp in route.waypoints:
                    if wp.action == "observe" and wp.target:
                        planned_obs.setdefault(wp.target, []).append(wp.depart)
        self.planned_obs = planned_obs
        self.engage_targets = {c for c in self.targets.values() if c}
        self.effectors_of = {}
        for aid, c in self.targets.items():
            if c and aid != aircraft_id:
                self.effectors_of.setdefault(c, []).append(aid)
        self.fact_sent = {}      # observation fact -> last time I sent it
        self.me = self.tracks.pop(aircraft_id)
        self.clock = self.me.clock
        self.route = self.clock.route
        self.target = self.targets[aircraft_id]
        self.is_effector = aircraft[aircraft_id].type == EFFECTOR
        self.announced = Track(self.clock)   # what the peers believe about me
        self.ann_left = 0
        self.last_ann_t = -INF
        self.reclassified = set()
        self.zones = []          # (x, y, radius, top)
        self.confirmed = {}      # contact -> latest time by which an observation of it was complete
        self.known_obs = set()   # observation facts already sent or relayed
        self.queue = {}          # observation fact -> sends left
        self.precut = set()      # facts every aircraft already knows from the plan
        self.aborted = False
        self.started = False

    # ------------------------------------------------------------------ entry point
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:   # an exception fails the whole run: hovering is the lesser evil
            return {"kind": "hold"}, None

    def _step(self, obs):
        t, tau = float(obs["t"]), float(obs["tau"])
        if not self.started:
            self._start(float(obs["t_loss"]))
        self._learn(obs, t)
        intent = self._intent(t, tau)
        rate = self._choose(t, tau, intent)
        message = self._message(t, tau, rate)
        return self._action(rate), message

    def _start(self, t_loss):
        """Observations finished before the cut are certain: everybody flew the plan exactly."""
        self.started = True
        for c, departs in self.planned_obs.items():
            done = [d for d in departs if d <= t_loss + EPS]
            if done:
                self.confirmed[c] = min(done)
                self.known_obs.add(c)
                self.precut.add(c)

    # ------------------------------------------------------------------ knowledge
    def _learn(self, obs, t):
        for e in obs["sensed"]:
            if e["kind"] == "reclassify":
                self.reclassified.add(e["contact"])
            elif e["kind"] == "popup_zone":
                z = (float(e["center"][0]), float(e["center"][1]), float(e["radius_m"]), float(e["top_m"]))
                if z not in self.zones:
                    self.zones.append(z)
        for m in obs["inbox"]:
            p = m.get("payload")
            if not isinstance(p, dict):
                continue
            k = p.get("k")
            sender = m.get("from")
            if isinstance(k, list) and len(k) == 2 and sender in self.tracks:
                self.tracks[sender].set(m["sent_t"], k[0], k[1])
            for c in p.get("o", []) or []:
                self._note_observed(c, float(m["sent_t"]), relay=1)
        for c in obs["own_observed"]:
            self._note_observed(c, t, relay=REPEAT)

    def _note_observed(self, c, when, relay):
        if when < self.confirmed.get(c, INF):
            self.confirmed[c] = when
        if c not in self.known_obs and c in self.engage_targets:
            self.known_obs.add(c)
            self.queue[c] = relay

    # ------------------------------------------------------------------ intent
    def _intent(self, t, tau):
        """Returns 'continue', 'wait' (effector one step short of an unconfirmed target) or 'return'."""
        if self.aborted:
            return "return"
        entry = self._zone_entry(tau)
        if self.target is not None:
            if self.target in self.reclassified or entry is not None:
                return self._abort()
            engage_t = t + (self.clock.end - tau)
            window = self.contacts[self.target].window
            if window and engage_t > window[1] + EPS:
                return self._abort()   # can no longer be served: never risk an engagement
            if tau + 1.0 >= self.clock.end - 1e-9 and not self._confirmed(engage_t):
                return "wait"
            return "continue"
        if entry is not None:
            obs_before = any(
                wp.action == "observe" and self.clock.dep[k] < entry and self.clock.dep[k] > tau
                for k, wp in enumerate(self.route.waypoints))
            if not obs_before or entry - tau <= ZONE_ABORT_S:
                return self._abort()
        return "continue"

    def _abort(self):
        self.aborted = True
        return "return"

    def _confirmed(self, engage_t):
        done = self.confirmed.get(self.target)
        return done is not None and done <= engage_t - self.confirm - CONFIRM_SLACK_S

    def _zone_entry(self, tau):
        """Route-clock time of the first segment ahead that enters a known zone, or None."""
        if not self.zones:
            return None
        taus = [tau] + [b for b in self.clock.breaks if b > tau + EPS]
        pts = [self.clock.position(x) for x in taus]
        for i in range(len(pts) - 1):
            for (x, y, r, top) in self.zones:
                if segment_enters_cylinder(pts[i], pts[i + 1], (x, y), r + ZONE_SOFT_M, top):
                    return taus[i]
        return None

    # ------------------------------------------------------------------ shield
    def _choose(self, t, tau, intent):
        if intent == "continue":
            cands = [1.0, 0.5, 0.0, -1.0]
        elif intent == "wait":
            cands = [0.0, -1.0, 0.5]
        else:
            cands = [0.0, -1.0, 0.5, 1.0]
        off_plan = self.aborted or abs(tau - t) > 1e-3 or intent != "continue"
        horizon = HORIZON_DEVIATED_S if off_plan else HORIZON_ON_PLAN_S
        peers = [(aid, tr) for aid, tr in self.tracks.items() if off_plan or tr.deviated]
        scored = []
        for rate in cands:
            if self._step_forbidden(t, tau, rate):
                continue
            slack = self._slack(t, tau, rate, horizon, peers)
            if slack > 0:
                return rate
            scored.append(rate)
        if not scored:
            return 0.0
        # Nothing is safe over the whole horizon. Peers off their plan have said what they do and
        # will not make room; peers on their plan do make room for an announced deviation at short
        # range (and may even be gone: a failed aircraft looks like one on its plan). So keep clear
        # of the deviated ones first, then of everybody over the next seconds, then the least bad.
        deviated = [(aid, tr) for aid, tr in peers if tr.deviated]
        for rate in scored:
            if self._slack(t, tau, rate, horizon, deviated) > 0:
                return rate
        short = [(self._slack(t, tau, rate, IMMINENT_S, peers), rate) for rate in scored]
        for value, rate in short:
            if value > 0:
                return rate
        return max(short)[1]

    def _step_forbidden(self, t, tau, rate):
        """Hard rules for the coming second: no zone entry, no unconfirmed or forbidden engagement."""
        end = self.clock.end
        nxt = min(end, max(self.clock.start, tau + rate))
        if self.target is not None and rate > 0 and nxt >= end - 1e-9:
            engage_t = t + (end - tau) / rate
            if self.aborted or self.target in self.reclassified or not self._confirmed(engage_t):
                return True
        if self.zones and rate != 0.0:
            lo, hi = (tau, nxt) if nxt >= tau else (nxt, tau)
            taus = [lo] + [b for b in self.clock.breaks if lo + EPS < b < hi - EPS] + [hi]
            pts = [self.clock.position(x) for x in taus]
            for i in range(len(pts) - 1):
                for (x, y, r, top) in self.zones:
                    if segment_enters_cylinder(pts[i], pts[i + 1], (x, y), r + ZONE_HARD_M, top):
                        return True
        return False

    def _slack(self, t, tau, rate, horizon, peers):
        """Smallest margin (m) to separation or to a zone over the horizon if `rate` is kept."""
        worst = INF
        start, end = self.clock.start, self.clock.end
        # A peer's latest news is at least a second old and the peer decides at the same time as I
        # do: besides what it announced, it may also stop where it is. Both futures must be safe.
        futures = [(aid, tr.clock, tr.tau_at(t), (tr.rate, 0.0)) for aid, tr in peers]
        for i in range(1, int(round(horizon / SAMPLE_S)) + 1):
            dt = i * SAMPLE_S
            mt = tau + rate * dt
            if mt >= end - EPS or mt <= start + EPS:
                break   # engaged or landed: no longer in the air
            p = self.clock.position(mt)
            for (x, y, r, _top) in self.zones:
                worst = min(worst, math.hypot(p[0] - x, p[1] - y) - r - ZONE_SOFT_M)
            for aid, clock, tau0, rates in futures:
                for r in rates:
                    x = tau0 + r * dt
                    if x <= clock.start + EPS or x >= clock.end - EPS:
                        continue
                    q = clock.position(x)
                    if self._exempt(aid, p, q):
                        continue
                    worst = min(worst, dist3(p, q) - self.sep)
        return worst

    def _exempt(self, aid, p, q):
        """R8 exemption: two effectors of the same target, both close to it."""
        if self.target is None or self.targets.get(aid) != self.target:
            return False
        c = self.contacts[self.target].pos
        r = self.exempt_r - EXEMPT_MARGIN_M
        return dist3(p, c) <= r and dist3(q, c) <= r

    # ------------------------------------------------------------------ messages
    def _message(self, t, tau, rate):
        ann = self.announced
        if abs(ann.tau_at(t) - tau) > 1e-2 or ann.rate != rate:
            ann.set(t, tau, rate)
            self.ann_left = REPEAT
        elif ann.deviated and self.ann_left == 0:
            near = self._nearest_peer(t)
            every = REANNOUNCE_CLOSE_S if near < REANNOUNCE_CLOSE_M else REANNOUNCE_S
            if near < REANNOUNCE_RANGE_M and t - self.last_ann_t >= every:
                self.ann_left = 1
        msg = {}
        if self.ann_left > 0:
            r = int(rate) if rate == int(rate) else rate
            msg["k"] = [round(tau, 2), r]
            self.ann_left -= 1
            self.last_ann_t = t
        for c in self._facts_to_refresh(t):
            self.queue[c] = max(self.queue.get(c, 0), 1)
        facts = sorted((c for c, n in self.queue.items() if n > 0),
                       key=lambda c: self.fact_sent.get(c, -INF))[:3]
        if facts:
            msg["o"] = facts
        while msg.get("o") and _size(msg) > MAX_BYTES:
            msg["o"].pop()
        if "o" in msg and not msg["o"]:
            del msg["o"]
        for c in msg.get("o", []):
            self.fact_sent[c] = t
            self.queue[c] -= 1
            if self.queue[c] <= 0:
                del self.queue[c]
        return msg or None

    def _facts_to_refresh(self, t):
        """Confirmations worth repeating: an effector of that target is near me and about to
        engage (or already waiting). Effectors still far from their target are left alone."""
        out = []
        here = None
        for c in self.known_obs:
            if c in self.precut or c in self.reclassified or t - self.fact_sent.get(c, -INF) < REFACT_S:
                continue
            window = self.contacts[c].window
            if window and t > window[1]:
                continue
            for aid in self.effectors_of.get(c, ()):
                tr = self.tracks[aid]
                if tr.clock.end - tr.tau_at(t) > APPROACH_S:
                    continue
                q = tr.pos_at(t)
                if q is None:
                    continue
                if here is None:
                    here = self.clock.position(self.announced.tau_at(t))
                if dist3(here, q) < REFACT_RANGE_M:
                    out.append(c)
                    break
        return out

    def _nearest_peer(self, t):
        here = self.clock.position(self.announced.tau_at(t))
        best = INF
        for tr in self.tracks.values():
            q = tr.pos_at(t)
            if q is not None:
                best = min(best, dist3(here, q))
        return best

    @staticmethod
    def _action(rate):
        if rate == 1.0:
            return {"kind": "continue"}
        if rate == 0.0:
            return {"kind": "hold"}
        if rate < 0:
            return {"kind": "return"}
        return {"kind": "speed", "scale": rate}


def _size(msg):
    import json
    return len(json.dumps(msg, separators=(",", ":")).encode())
