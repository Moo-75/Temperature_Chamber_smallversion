#!/usr/bin/env python3
"""Band performance task: 5-45 C open-loop sweeps, 10-40 C analysis (control sensor).

Measures four open-loop 5->45->5 C round trips and four closed-loop
5 C step round trips over the 10-40 C range (subject to dew-point floor).
Each change of direction is separated by a 10-minute duty-zero rest and
closed-loop preparation at the next starting temperature. A final reference
step follows the ladder. Temperature abort limits are disabled by operator
request; sensor-failure and timeout handling remain. Duty-zero rests do not
prevent thermal overshoot. Dew point is estimated from operator-entered values
and is not verified. Condensation risk at 5 C remains when dew point is unknown.

Firmware used unchanged (peltier_operating_system.ino). Closed-loop steps use SET_TEMP,
START, STOP and GET_CTRL. Sweeps use FORCE_PWM; rests use STOP and GET_TEMP.
The simulator copies the firmware control law roughly; it checks the logic only.

Usage
  python perf_bands.py --sim --outdir <folder> --config <json with room_c, room_rh_pct>
  python perf_bands.py --plan pilot    # first real run: one heating sweep only
  python perf_bands.py                 # full run on the real chamber (asks first)

Outputs per run: PerfBands_samples.csv (10 Hz), PerfBands_events.csv, PerfBands_open_bands.csv,
PerfBands_steps.csv, meta.json, and PNG figures. Default folder: perf_runs/PerfBands_<timestamp>/
next to this script. The prefix lets migrate_to_server.py find the folder.
"""

import argparse
import csv
import hashlib
import json
import math
import os
import random
import statistics
import sys
import time
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime
from typing import Optional

DT = 0.1                 # sample period, s (the firmware control loop also runs at 100 ms)
FORCE_REFRESH_S = 1.0    # FORCE_PWM expires after 4 s in the firmware
ADC_STEP_C = 0.0874      # plate ADC step near 24 C (10 kOhm, B = 3950); simulator only
HERE = os.path.dirname(os.path.abspath(__file__))
FIRMWARE = "peltier_operating_system.ino"
# Output CSVs carry this prefix so that migrate_to_server.py (EXPERIMENT_PATTERNS) finds the folder.
CSV_PREFIX = "PerfBands_"


class Abort(Exception):
    """Ends the run. The drive is stopped on the way out."""


def dew_point_c(t_c, rh_pct):
    a, b = 17.62, 243.12
    g = math.log(rh_pct / 100.0) + a * t_c / (b + t_c)
    return b * g / (a - g)


@dataclass
class Config:
    port: str = "/dev/arduino"
    baud: int = 115200
    en_mode: str = "BOTH"
    seed: int = 1
    # range
    target_min_c: float = 10.0
    target_max_c: float = 40.0
    band_width_c: float = 5.0
    dew_margin_c: float = 2.0
    open_min_c: float = 5.0
    open_max_c: float = 45.0
    room_c: Optional[float] = None
    room_rh_pct: Optional[float] = None
    # baseline and fixed rests
    baseline_s: float = 300.0
    baseline_max_s: float = 1800.0
    plate_slope_max: float = 0.003      # C/s over 120 s; used by the baseline only
    rest_fixed_s: float = 600.0         # duty 0 between directions, a fixed time
    # closed-loop steps
    prep_timeout_s: float = 900.0
    step_timeout_s: float = 900.0
    settle_band_c: float = 0.5
    settle_hold_s: float = 60.0         # must stay inside the band this long to count as settled
    hold_5c_s: float = 60.0
    hold_big_s: float = 120.0
    # 5 C ladder: round trips 10 -> 40 -> 10
    ladder_round_trips: int = 4
    open_round_trips: int = 4
    # open-loop sweeps
    sweep_timeout_s: float = 300.0
    plateau_window_s: float = 60.0
    plateau_slope_c_s: float = 0.01
    # output
    include_ref_end: bool = True        # one reference 10 -> 15 C step at the end
    make_plots: bool = True             # PNG curves after the run (needs matplotlib)
    stages: tuple = ()                  # empty: all stages; open_heat, open_cool, ladder, ref_end


# ---------- ranges and helpers ----------

def band_list(cfg):
    """All 5 C bands between the target limits, as (lo, hi) pairs."""
    n = int(round((cfg.target_max_c - cfg.target_min_c) / cfg.band_width_c))
    return [(round(cfg.target_min_c + i * cfg.band_width_c, 6),
             round(cfg.target_min_c + (i + 1) * cfg.band_width_c, 6)) for i in range(n)]


def floor_info(cfg):
    dew = dew_point_c(cfg.room_c, cfg.room_rh_pct)
    low = max(cfg.target_min_c, dew + cfg.dew_margin_c)
    return dew, low


def heat_edges(bands, low):
    """Band upper edges above the floor, in travel order; the last one is the top of the range."""
    return [hi for lo, hi in bands if hi > low + 1e-9]


def cool_edges(bands, high, low):
    """Band lower edges below the start, in travel order, ending at the floor."""
    return [lo for lo, hi in reversed(bands) if high - 1e-9 > lo > low + 1e-9] + [round(low, 3)]


def passed(value, level, direction):
    return value >= level if direction > 0 else value <= level


def lin_slope(xs, ys):
    n = len(xs)
    if n < 2:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0.0:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx


def fmt(value, nd=3):
    return "" if value is None else f"{value:.{nd}f}"


def mean_sd(values, nd=2):
    vals = [v for v in values if v is not None]
    if not vals:
        return "-"
    if len(vals) == 1:
        return f"{vals[0]:.{nd}f}"
    return f"{statistics.mean(vals):.{nd}f} ± {statistics.stdev(vals):.{nd}f}"


def firmware_sha1():
    path = os.path.join(HERE, FIRMWARE)
    if not os.path.exists(path):
        return None
    with open(path, "rb") as f:
        return hashlib.sha1(f.read()).hexdigest()


def step_metrics(samples, t_cmd, start, target, settle_rel, hold_s, sat_threshold=0.98):
    """Rise, overshoot and steady-state numbers of one closed-loop step."""
    rel = [(s.t - t_cmd, s.plate, s.duty) for s in samples if s.plate is not None]
    delta = target - start
    span = abs(delta)
    sgn = 1.0 if delta >= 0 else -1.0
    end = settle_rel + hold_s if settle_rel is not None else (rel[-1][0] if rel else 0.0)
    window = [(t, p, u) for t, p, u in rel if t <= end]
    t10 = t90 = None
    for t, p, _ in window:
        frac = (p - start) * sgn / span if span > 0 else 0.0
        if t10 is None and frac >= 0.1:
            t10 = t
        if t90 is None and frac >= 0.9:
            t90 = t
            break
    overshoot = max([0.0] + [(p - target) * sgn for _, p, _ in window])
    tail = [p - target for t, p, _ in window if end - 30.0 <= t <= end]
    duties = [u for _, _, u in window if u is not None]
    sat_time = sum(DT for u in duties if abs(u) >= sat_threshold)
    return {
        "t_10_s": None if t10 is None else round(t10, 2),
        "t_90_s": None if t90 is None else round(t90, 2),
        "rise_10_90_s": round(t90 - t10, 2) if (t10 is not None and t90 is not None) else None,
        "overshoot_c": round(overshoot, 3),
        "overshoot_pct": round(100.0 * overshoot / span, 2) if span > 0 else None,
        "steady_err_c": round(statistics.mean(tail), 3) if tail else None,
        "steady_sd_c": round(statistics.stdev(tail), 3) if len(tail) > 1 else None,
        "max_abs_duty": round(max((abs(u) for u in duties), default=0.0), 3),
        "sat_time_s": round(sat_time, 1),
        "end_s": round(end, 2),
    }


# ---------- device interfaces ----------

@dataclass
class Sample:
    t: float
    plate: Optional[float]
    duty: float = 0.0


class SerialDevice:
    """The real chamber: the Arduino over USB serial, same commands as maze.py."""

    def __init__(self, port, baud):
        import serial  # imported here so that --sim runs without pyserial
        self.serial = serial.Serial()
        self.serial.port = port
        self.serial.baudrate = baud
        self.serial.timeout = 0.5
        self.serial.dtr = False  # as maze.py: do not reset the Uno on open
        self.serial.rts = False
        self.serial.open()
        time.sleep(2.0)
        self.ready_line = self.serial.readline().decode("utf-8", "replace").strip()
        self.serial.reset_input_buffer()
        self._t0 = time.monotonic()
        self._next = self._t0

    def now(self):
        return time.monotonic() - self._t0

    def pace(self):
        self._next += DT
        wait = self._next - time.monotonic()
        if wait > 0:
            time.sleep(wait)
        else:
            self._next = time.monotonic()  # fell behind: resynchronise

    def _write(self, text):
        self.serial.write((text + "\n").encode("ascii"))

    def _query(self, text):
        self.serial.reset_input_buffer()
        self._write(text)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            line = self.serial.readline().decode("utf-8", "replace").strip()
            if line:
                return line
        return None

    def plate(self):
        """Plate temperature from GET_TEMP. Valid whether or not the controller runs."""
        try:
            value = float(self._query("GET_TEMP"))
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) else None

    def ctrl(self):
        """(plate temperature, applied duty) from GET_CTRL. Valid only while the controller runs."""
        raw = self._query("GET_CTRL")
        try:
            parts = [float(x) for x in raw.split(",")]
        except (AttributeError, ValueError):
            return None
        if len(parts) < 10 or not math.isfinite(parts[0]):
            return None
        return parts[0], parts[3]

    def force(self, u):
        self._write(f"FORCE_PWM,{u:.3f}")

    def set_target(self, value):
        self._write(f"SET_TEMP,{float(value):.3f}")

    def start(self):
        self._write("START")

    def stop(self):
        self._write("STOP")

    def en_mode(self, mode):
        return self._query(f"EN_MODE,{mode}")

    def info(self):
        return {"port": self.serial.port, "ready_line": self.ready_line}

    def close(self):
        try:
            self.stop()
        finally:
            self.serial.close()


class SimDevice:
    """Lumped model with a virtual clock: plate, heat block, water loop, ambient.

    The controller copies the firmware law (P, derivative on rate, I, DU_MAX). The
    plant is rough on purpose. This checks the logic, not the chamber's numbers."""

    CP, CH, CW = 30.0, 150.0, 5000.0       # heat capacities, J/K
    HMAX, CMAX = 20.0, 15.0                 # W at |u| = 1
    G_PL, G_BW, G_WA = 0.6, 20.0, 5.0       # conductances, W/K
    KP, KD, KI, I_MAX, DU_MAX = 0.25, 1.0, 0.025, 0.5, 0.15
    RATE_TAU = 1.2

    def __init__(self, seed=0, ambient=23.0, fault_window=None):
        self.rng = random.Random(seed)
        self.ta = ambient
        self.tp = self.th = self.tw = ambient
        self.t = 0.0
        self.force_u = 0.0
        self.force_until = -1.0
        self.running = False
        self.target = ambient
        self.i_term = 0.0
        self.u_applied = 0.0
        self.rate = 0.0
        self.prev_meas = None
        self.fault_window = fault_window

    def now(self):
        return self.t

    def _measure(self):
        noisy = self.tp + self.rng.gauss(0.0, 0.02)
        return round(noisy / ADC_STEP_C) * ADC_STEP_C

    def plate(self):
        if self.fault_window and self.fault_window[0] <= self.t < self.fault_window[1]:
            return None
        return self._measure()

    def ctrl(self):
        m = self.plate()
        if m is None:
            return None
        return m, self.u_applied

    def pace(self):
        if self.t < self.force_until:
            u = self.force_u
            self.u_applied = u
        elif self.running:
            u = self._control(self._measure())
        else:
            u = 0.0
            self.u_applied = 0.0
        self._plant(u)
        self.t += DT

    def _control(self, meas):
        if self.prev_meas is None:
            self.prev_meas = meas
        d = meas - self.prev_meas
        if abs(d) > 1.2:
            self.prev_meas = meas
        else:
            alpha = DT / (self.RATE_TAU + DT)
            self.rate += alpha * (d / DT - self.rate)
            self.prev_meas = meas
        error = self.target - meas
        u_unsat = self.KP * error - self.KD * self.rate + self.i_term
        u = max(-1.0, min(1.0, u_unsat))
        saturated = (u_unsat >= 1.0 and error > 0) or (u_unsat <= -1.0 and error < 0)
        if not saturated:
            self.i_term = max(-self.I_MAX, min(self.I_MAX, self.i_term + DT * self.KI * error))
        du = max(-self.DU_MAX, min(self.DU_MAX, u - self.u_applied))
        self.u_applied += du
        return self.u_applied

    def _plant(self, u):
        ta, th, tw, tp = self.ta, self.th, self.tw, self.tp
        f_h = min(1.0, max(0.25, 1.0 - max(0.0, ta - th) / 30.0))
        f_c = min(1.0, max(0.2, 1.0 - max(0.0, th - ta) / 25.0))
        if u > 0:
            p = self.HMAX * u * f_h
            q_block = -0.5 * p
        elif u < 0:
            p = -self.CMAX * (-u) * f_c
            q_block = -1.2 * p
        else:
            p = 0.0
            q_block = 0.0
        d_tp = (p - self.G_PL * (tp - ta)) / self.CP
        d_th = (q_block - self.G_BW * (th - tw)) / self.CH
        d_tw = (self.G_BW * (th - tw) - self.G_WA * (tw - ta)) / self.CW
        self.tp = tp + d_tp * DT
        self.th = th + d_th * DT
        self.tw = tw + d_tw * DT

    def set_target(self, value):
        value = float(value)
        if abs(value - self.target) > 1.0:
            self.i_term = 0.0          # firmware: integrator cleared on a jump above 1 C
        self.target = value

    def start(self):
        self.running = True

    def stop(self):
        self.running = False
        self.force_u = 0.0
        self.force_until = -1.0
        self.u_applied = 0.0

    def force(self, u):
        self.force_u = max(-1.0, min(1.0, float(u)))
        self.force_until = self.t + 4.0

    def en_mode(self, mode):
        return mode

    def info(self):
        return {"simulated": True}

    def close(self):
        self.stop()


# ---------- recording ----------

class Recorder:
    COLS = ["t_s", "phase", "event", "tag", "direction", "duty", "plate_c"]

    def __init__(self, outdir):
        os.makedirs(outdir, exist_ok=True)
        self.outdir = outdir
        self._f = open(os.path.join(outdir, CSV_PREFIX + "samples.csv"), "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(self.COLS)
        self._n = 0
        self.events = []
        self.bands = []
        self.steps = []
        self.notes = []          # skipped events and stopped ladders, for the summary

    def sample(self, s, phase, event, tag, direction):
        self._w.writerow([f"{s.t:.2f}", phase, event, tag, direction, f"{s.duty:.3f}", fmt(s.plate)])
        self._n += 1
        if self._n % 10 == 0:
            self._f.flush()

    def close(self, meta):
        self._f.close()
        write_rows(os.path.join(self.outdir, CSV_PREFIX + "events.csv"), self.events)
        write_rows(os.path.join(self.outdir, CSV_PREFIX + "open_bands.csv"), self.bands)
        write_rows(os.path.join(self.outdir, CSV_PREFIX + "steps.csv"), self.steps)
        with open(os.path.join(self.outdir, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2, ensure_ascii=False)


def write_rows(path, rows):
    if not rows:
        return
    keys = []
    for row in rows:
        for k in row:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)


# ---------- the session: acquisition, pauses, prep, sweeps, steps ----------

class Session:
    def __init__(self, dev, cfg, rec):
        self.dev = dev
        self.cfg = cfg
        self.rec = rec
        self.buf = deque()
        self.bad = 0
        self.phase = "INIT"
        self.event = ""
        self.tag = ""
        self.direction = 0
        self.seq = 0
        self.state = "rest"      # "driven" after a sweep, step or prep; "rest" after a pause or the baseline
        self.running = False     # drive on (closed loop or FORCE) as far as this script knows

    # ----- acquisition and safety -----
    def acquire(self, ctrl=False, duty=0.0):
        self.dev.pace()
        t = self.dev.now()
        if ctrl:
            res = self.dev.ctrl()
            plate, duty = res if res is not None else (None, 0.0)
        else:
            plate = self.dev.plate()
        self.bad = 0 if plate is not None else self.bad + 1
        s = Sample(t, plate, duty)
        self.buf.append(s)
        while self.buf and t - self.buf[0].t > 1500.0:
            self.buf.popleft()
        self.rec.sample(s, self.phase, self.event, self.tag, self.direction)
        if self.bad >= 3:
            raise Abort("plate reading invalid 3 times in a row")
        return s

    # ----- windows -----
    def _values(self, since, seconds):
        last = self.buf[-1].t
        lo = max(since, last - seconds)
        ts, vs = [], []
        for s in reversed(self.buf):
            if s.t < lo:
                break
            if s.plate is not None:
                ts.append(s.t)
                vs.append(s.plate)
        return ts[::-1], vs[::-1], lo

    def plate_slope(self, since, seconds=120.0):
        ts, vs, lo = self._values(since, seconds)
        if len(ts) < 20 or self.buf[-1].t - lo < 0.95 * seconds:
            return None
        return lin_slope(ts, vs)

    def mean_plate(self, seconds):
        _, vs, _ = self._values(self.buf[0].t, seconds)
        return sum(vs) / len(vs) if vs else None

    def _event(self, kind, eid, tag, t_begin, t_end, **extra):
        row = {"event": f"{kind}{eid}", "kind": kind, "tag": tag,
               "t_begin_s": round(t_begin, 2), "t_end_s": round(t_end, 2),
               "duration_s": round(t_end - t_begin, 2)}
        row.update(extra)
        self.rec.events.append(row)

    def stop_drive(self):
        self.dev.stop()
        self.running = False

    # ----- stages -----
    def baseline(self):
        cfg = self.cfg
        self.stop_drive()
        self.seq += 1
        eid = self.seq
        self.event = f"BASE{eid}"
        self.phase, self.tag, self.direction = "BASE", "baseline", 0
        t_begin = self.acquire(duty=0.0).t
        while True:
            self.acquire(duty=0.0)
            elapsed = self.buf[-1].t - t_begin
            slope = self.plate_slope(t_begin)
            if elapsed >= cfg.baseline_s and slope is not None and abs(slope) <= cfg.plate_slope_max:
                break
            if elapsed >= cfg.baseline_max_s:
                raise Abort("baseline did not become quasi-steady")
        t_end = self.buf[-1].t
        level = self.mean_plate(180.0)
        if level is None:
            raise Abort("no plate reading at the baseline")
        self._event("BASE", eid, "baseline", t_begin, t_end, plate_c=fmt(level, 2))
        self.state = "rest"
        print(f"baseline: plate {level:.2f} C after {(t_end - t_begin) / 60:.1f} min at duty 0")
        return level

    def pause(self, tag, seconds):
        """Duty 0 for a fixed time. Not gated on steadiness: the plate slope at the end is logged."""
        self.stop_drive()
        self.seq += 1
        eid = self.seq
        self.event = f"PAUSE{eid}"
        self.phase, self.tag, self.direction = "PAUSE", tag, 0
        first = self.acquire(duty=0.0)          # first reading after the stop
        since, start_c = first.t, first.plate
        while self.buf[-1].t - since < seconds:
            self.acquire(duty=0.0)
        end = self.buf[-1]
        slope = self.plate_slope(since)
        slope_min = None if slope is None else 60.0 * slope
        self._event("PAUSE", eid, tag, since, end.t, seconds=seconds,
                    plate_start_c=fmt(start_c, 2), plate_end_c=fmt(end.plate, 2),
                    slope_c_per_min=fmt(slope_min, 3))
        self.state = "rest"
        print(f"  pause [{tag}]: {seconds / 60:.1f} min at duty 0, plate {fmt(start_c, 2)} -> "
              f"{fmt(end.plate, 2)} C, slope {fmt(slope_min, 3)} C/min")

    def pause_if_driven(self, tag):
        """The fixed rest before the next event, if the event before drove the chamber."""
        if self.state == "driven":
            self.pause(tag, self.cfg.rest_fixed_s)

    def prep(self, target):
        """Closed-loop move to `target`. Not scored. The controller is left running there if the
        plate settles; otherwise it is stopped and the event is skipped (returns False)."""
        cfg = self.cfg
        self.seq += 1
        eid = self.seq
        self.event = f"PREP{eid}"
        self.phase, self.tag, self.direction = "PREP", f"prep_{target:g}", 0
        self.state = "driven"
        self.dev.set_target(target)
        self.dev.start()
        self.running = True
        t_cmd = self.buf[-1].t
        inside = None
        settled = False
        while True:
            s = self.acquire(ctrl=True)
            if s.plate is not None and abs(s.plate - target) <= cfg.settle_band_c:
                if inside is None:
                    inside = s.t
                if s.t - inside >= cfg.settle_hold_s:
                    settled = True
                    break
            else:
                inside = None
            if s.t - t_cmd >= cfg.prep_timeout_s:
                break
        if not settled:
            self.stop_drive()
        self._event("PREP", eid, f"prep_{target:g}", t_cmd, self.buf[-1].t,
                    target_c=target, settled=settled, plate_end_c=fmt(self.buf[-1].plate, 2))
        if not settled:
            print(f"  prep to {target:g} C did not settle in {cfg.prep_timeout_s:.0f} s: event skipped")
            self.rec.notes.append(f"준비 이동 {target:g} C 가 {cfg.prep_timeout_s:.0f} s 안에 정착하지 않아 그 이벤트를 건너뜀")
            return False
        print(f"  prep to {target:g} C: settled")
        return True

    def sweep(self, direction, bound, edges, tag, round_trip=None):
        """Open-loop full duty until the plate passes `bound` (or plateaus, or times out)."""
        cfg = self.cfg
        self.seq += 1
        eid = self.seq
        self.event = f"SWEEP{eid}"
        self.phase, self.tag, self.direction = "SWEEP", tag, direction
        level = self.mean_plate(10.0)
        if level is None:
            raise Abort("no plate reading before a sweep")
        t0 = self.buf[-1].t
        self.state = "driven"
        self.running = True
        self.dev.force(direction)
        last_force = t0
        crossings = {}
        window = deque()
        samples = []
        reason = "timeout"
        try:
            while True:
                s = self.acquire(duty=float(direction))
                samples.append(s)
                if s.t - last_force >= FORCE_REFRESH_S:
                    self.dev.force(direction)
                    last_force = s.t
                el = s.t - t0
                if s.plate is not None:
                    for e in edges:
                        if e not in crossings and passed(s.plate, e, direction):
                            crossings[e] = round(el, 2)
                    if passed(s.plate, bound, direction):
                        reason = "bound"
                        break
                    window.append((s.t, s.plate))
                    while window and s.t - window[0][0] > cfg.plateau_window_s:
                        window.popleft()
                if (el >= cfg.plateau_window_s and window
                        and window[-1][0] - window[0][0] >= 0.9 * cfg.plateau_window_s):
                    slope = lin_slope([t for t, _ in window], [p for _, p in window])
                    if slope is not None and abs(slope) <= cfg.plateau_slope_c_s:
                        reason = "plateau"
                        break
                if el >= cfg.sweep_timeout_s:
                    break
        finally:
            self.stop_drive()
        t_end = samples[-1].t if samples else t0
        prev_level, prev_t = level, 0.0
        for e in edges:
            if e not in crossings:
                break
            t_e = crossings[e]
            span = abs(e - prev_level)
            dur = t_e - prev_t
            self.rec.bands.append({
                "tag": tag, "direction": direction, "round_trip": round_trip,
                "band": f"{round(prev_level, 1):g}-{e:g}",
                "lo_c": round(min(prev_level, e), 2), "hi_c": round(max(prev_level, e), 2),
                "span_c": round(span, 3), "t_start_s": round(prev_t, 2), "t_end_s": round(t_e, 2),
                "duration_s": round(dur, 2),
                "rate_c_per_s": round(span / dur, 4) if dur > 0 else None,
                "reason": reason,
            })
            prev_level, prev_t = e, t_e
        end_plate = samples[-1].plate if samples else None
        self._event("SWEEP", eid, tag, t0, t_end, direction=direction, round_trip=round_trip, reason=reason,
                    start_c=round(level, 2), end_c=fmt(end_plate, 2))
        print(f"  sweep [{tag}] from {level:.1f} C: {reason}, ended at {fmt(end_plate, 2)} C "
              f"after {(t_end - t0):.0f} s")

    def step(self, start, target, tag, hold_s, clamped=False, round_trip=None, stop_after=True):
        """Closed-loop step with the controller in charge. Scored. stop_after=False leaves the
        controller running, so the next step starts from here with no rest in between."""
        cfg = self.cfg
        self.seq += 1
        eid = self.seq
        self.event = f"STEP{eid}"
        self.phase, self.tag = "STEP", tag
        self.direction = 1 if target > start else -1
        level = self.mean_plate(10.0)
        if level is None:
            level = start
        t_cmd = self.buf[-1].t
        self.state = "driven"
        self.dev.set_target(target)
        self.dev.start()
        self.running = True
        samples = []
        run_start = None      # start of the current run inside the band
        settle_abs = None     # start of the final in-band run, once it has lasted settle_hold_s
        reason = "timeout"
        completed = False
        try:
            while True:
                s = self.acquire(ctrl=True)
                samples.append(s)
                el = s.t - t_cmd
                if s.plate is not None and abs(s.plate - target) <= cfg.settle_band_c:
                    if run_start is None:
                        run_start = s.t
                    if settle_abs is None and s.t - run_start >= cfg.settle_hold_s:
                        settle_abs = run_start
                else:
                    run_start = None
                    settle_abs = None          # any excursion after that cancels the settle
                if settle_abs is not None and s.t - settle_abs >= hold_s:
                    reason = "held"
                    break
                if el >= cfg.step_timeout_s:
                    break
            completed = True
        finally:
            if stop_after or not completed:
                self.stop_drive()
        t_end = samples[-1].t if samples else t_cmd
        settle_rel = None if settle_abs is None else settle_abs - t_cmd
        m = step_metrics(samples, t_cmd, level, target, settle_rel, hold_s)
        row = {"tag": tag, "round_trip": round_trip, "direction": self.direction,
               "begin_c": round(start, 3), "end_c": round(target, 3),
               "measured_begin_c": round(level, 3), "clamped": clamped, "reason": reason,
               "settled": settle_rel is not None,
               "t_settle_s": None if settle_rel is None else round(settle_rel, 2)}
        row.update(m)
        self.rec.steps.append(row)
        self._event("STEP", eid, tag, t_cmd, t_end, direction=self.direction, round_trip=round_trip,
                    reason=reason, begin_c=round(start, 2), end_c=round(target, 2),
                    t_settle_s=row["t_settle_s"], rise_10_90_s=m["rise_10_90_s"],
                    overshoot_c=m["overshoot_c"])
        rt = "" if round_trip is None else f" (round trip {round_trip})"
        print(f"  step [{tag}]{rt} {start:.1f} -> {target:.1f} C: settle {fmt(row['t_settle_s'], 1)} s, "
              f"rise {fmt(m['rise_10_90_s'], 1)} s, overshoot {m['overshoot_c']:.2f} C ({reason})")
        return row


# ---------- protocol ----------

def run_ladder(sess, cfg, valid):
    """5 C closed-loop steps, with a duty-zero rest at every direction change."""
    up = list(valid)
    down = [(hi, lo) for lo, hi in reversed(valid)]
    for rt in range(1, cfg.ladder_round_trips + 1):
        print(f"5 C round trip {rt} of {cfg.ladder_round_trips}")
        if not sess.running and not sess.prep(up[0][0]):
            sess.rec.notes.append("5 C 사다리 중단: 가열 시작 온도 준비 실패")
            return False
        for a, b in up:
            sess.step(a, b, "up_5c", cfg.hold_5c_s, round_trip=rt, stop_after=False)
        sess.pause(f"ladder_after_heat_{rt}", cfg.rest_fixed_s)
        if not sess.prep(down[0][0]):
            sess.rec.notes.append("5 C 사다리 중단: 냉각 시작 온도 준비 실패")
            return False
        for a, b in down:
            sess.step(a, b, "down_5c", cfg.hold_5c_s, round_trip=rt, stop_after=False)
        if rt < cfg.ladder_round_trips:
            sess.pause(f"ladder_after_cool_{rt}", cfg.rest_fixed_s)
    sess.stop_drive()
    return True


def band_lo(row):
    """Lower edge of the 5 C band a ladder step covers: an up step starts there, a down step ends there."""
    return row["begin_c"] if row["direction"] > 0 else row["end_c"]


def ladder_cell(rows):
    if not rows:
        return "-"
    settled = [r["t_settle_s"] for r in rows if r["t_settle_s"] is not None]
    cell = (f"{mean_sd(settled, 1)} / {mean_sd([r['overshoot_c'] for r in rows], 2)} / "
            f"{mean_sd([r['steady_err_c'] for r in rows], 2)}")
    if len(settled) < len(rows):
        cell += f" (정착 {len(settled)}/{len(rows)})"
    return cell


def ladder_lines(steps, valid):
    rows = [r for r in steps if r["tag"] in ("up_5c", "down_5c")]
    if not rows:
        return []
    rts = sorted({r["round_trip"] for r in rows})

    def pick(direction, lo, rt=None):
        return [r for r in rows if r["direction"] == direction
                and round(band_lo(r), 1) == round(lo, 1)
                and (rt is None or r["round_trip"] == rt)]

    lines = [f"5 C 계단, 왕복 {len(rts)}회 (평균 ± 표준편차; 정착 s / 오버슈트 C / 정상상태 오차 C)"]
    for lo, hi in valid:
        lines.append(f"  {lo:g}-{hi:g} C  상승 {ladder_cell(pick(1, lo))}  |  하강 {ladder_cell(pick(-1, lo))}")
    if len(rts) >= 2:
        for direction, name in ((1, "상승"), (-1, "하강")):
            diffs = []
            for lo, _ in valid:
                first = [r["t_settle_s"] for r in pick(direction, lo, rts[0]) if r["t_settle_s"] is not None]
                last = [r["t_settle_s"] for r in pick(direction, lo, rts[-1]) if r["t_settle_s"] is not None]
                if first and last:
                    diffs.append((last[0] - first[0]) / first[0])
            if diffs:
                lines.append(f"  첫 왕복과 마지막 왕복의 정착 시간 차이 ({name}, 구간 평균): "
                             f"{statistics.mean(diffs):+.1%} (10 % 넘으면 드리프트 의심)")
    ref = [r["t_settle_s"] for r in steps if r["tag"] == "ref_end" and r["t_settle_s"] is not None]
    first = []
    if valid and rts:
        first = [r["t_settle_s"] for r in pick(1, valid[0][0], rts[0]) if r["t_settle_s"] is not None]
    if ref and first:
        lines.append(f"  첫 왕복과 끝 참조 계단({valid[0][0]:g}->{valid[0][1]:g} C) 정착 시간 차이: "
                     f"{(ref[0] - first[0]) / first[0]:+.1%}")
    return lines


def summarize(cfg, dew, low, valid, skipped, bands, steps, notes):
    lines = [f"이슬점 {dew:.1f} C, 측정 하한 {low:.1f} C (이슬점 + {cfg.dew_margin_c:g} C; 가열과 냉각 모두 이 온도부터)"]
    if skipped:
        lines.append("냉각 하한 아래라 건너뛴 5 C 구간: " + ", ".join(f"{lo:g}-{hi:g}" for lo, hi in skipped))
    lines.extend(notes)
    for direction, name in ((1, "가열"), (-1, "냉각")):
        rows = [r for r in bands if r["direction"] == direction and r["rate_c_per_s"] is not None]
        rows = [r for r in rows if r["lo_c"] >= cfg.target_min_c and r["hi_c"] <= cfg.target_max_c]
        if rows:
            lines.append(f"개방루프 구간별 속도 ({name}, 듀티 100 %):")
            for r in rows:
                partial = ", 부분 구간" if r["span_c"] < 4.5 else ""
                lines.append(f"  {r['band']} C: {r['rate_c_per_s']:.3f} C/s "
                             f"(구간 {r['span_c']:.1f} C, {r['duration_s']:.0f} s, {r['reason']}{partial})")
    for r in steps:
        if r["tag"].startswith("big"):
            if not r["clamped"]:
                clamp_note = ""
            elif r["tag"] == "big_up":
                clamp_note = " (시작을 측정 하한으로 올림)"
            else:
                clamp_note = " (끝을 측정 하한으로 올림)"
            lines.append(f"{r['tag']} {r['begin_c']:.1f} -> {r['end_c']:.1f} C: 상승(10-90 %) "
                         f"{fmt(r['rise_10_90_s'], 1)} s, 총 정착 {fmt(r['t_settle_s'], 1)} s, "
                         f"오버슈트 {r['overshoot_c']:.2f} C, 정상상태 오차 {fmt(r['steady_err_c'], 2)} C, "
                         f"최대 듀티 {r['max_abs_duty']:.2f}" + clamp_note)
    lines.extend(ladder_lines(steps, valid))
    lines.append("잠정 결과입니다. 실행 한 번으로 결론 내지 말고 다른 날 반복해서 확인하세요.")
    return lines


def make_figures(outdir):
    """Curves as PNG files, built from the CSV outputs. Needs matplotlib. Returns the file names."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def read(name):
        path = os.path.join(outdir, name)
        if not os.path.exists(path):
            return []
        with open(path, encoding="utf-8") as f:
            return list(csv.DictReader(f))

    written = []
    bands = read(CSV_PREFIX + "open_bands.csv")
    if bands:
        fig, ax = plt.subplots(figsize=(6, 4))
        for direction, label, color in ((1, "heating, full duty", "#D55E00"),
                                        (-1, "cooling, full duty", "#0072B2")):
            runs = sorted({r.get("round_trip", "") for r in bands if r["direction"] == str(direction)})
            for i, rt in enumerate(runs):
                pts = sorted(((float(r["lo_c"]) + float(r["hi_c"])) / 2.0, float(r["rate_c_per_s"]))
                             for r in bands if r["direction"] == str(direction) and r["rate_c_per_s"]
                             and r.get("round_trip", "") == rt and float(r["span_c"]) >= 4.5
                             and float(r["lo_c"]) >= 10 and float(r["hi_c"]) <= 40)
                if pts:
                    ax.plot([p[0] for p in pts], [p[1] for p in pts], "o-", color=color,
                            alpha=0.65, label=label if i == 0 else None)
        ax.set_xlabel("band midpoint (C)")
        ax.set_ylabel("mean rate in band (C/s)")
        ax.set_title("Open-loop rate per 5 C band")
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, "fig_open_rates.png"), dpi=150)
        plt.close(fig)
        written.append("fig_open_rates.png")

    steps = read(CSV_PREFIX + "steps.csv")
    ladder = [r for r in steps if r["tag"] in ("up_5c", "down_5c")]
    if ladder:
        fig, axes = plt.subplots(2, 2, figsize=(10, 7))
        keys = (("t_settle_s", "settling time (s)"), ("rise_10_90_s", "rise 10-90 % (s)"),
                ("overshoot_c", "overshoot (C)"), ("steady_err_c", "steady-state error (C)"))
        for ax, (key, ylabel) in zip(axes.flat, keys):
            for direction, label, color in ((1, "up", "#D55E00"), (-1, "down", "#0072B2")):
                groups = {}
                for r in ladder:
                    if r["direction"] != str(direction) or r[key] in ("", "None"):
                        continue
                    lo = float(r["begin_c"]) if direction > 0 else float(r["end_c"])
                    groups.setdefault(round(lo, 1), []).append(float(r[key]))
                xs = sorted(groups)
                if not xs:
                    continue
                means = [statistics.mean(groups[x]) for x in xs]
                sds = [statistics.stdev(groups[x]) if len(groups[x]) > 1 else 0.0 for x in xs]
                ax.errorbar([x + 2.5 for x in xs], means, yerr=sds, fmt="o-", color=color,
                            label=label, capsize=3)
            ax.set_xlabel("band midpoint (C)")
            ax.set_ylabel(ylabel)
            if ax.has_data():
                ax.legend()
        fig.suptitle("5 C steps, closed loop: mean ± SD over round trips")
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, "fig_5c_steps.png"), dpi=150)
        plt.close(fig)
        written.append("fig_5c_steps.png")

    big = [e for e in read(CSV_PREFIX + "events.csv") if e["kind"] == "STEP" and e["tag"] in ("big_up", "big_down")]
    if big:
        wanted = {e["event"]: e for e in big}
        traces = {name: [] for name in wanted}
        with open(os.path.join(outdir, CSV_PREFIX + "samples.csv"), encoding="utf-8") as f:
            for row in csv.DictReader(f):
                if row["event"] in traces and row["plate_c"]:
                    t_rel = float(row["t_s"]) - float(wanted[row["event"]]["t_begin_s"])
                    traces[row["event"]].append((t_rel, float(row["plate_c"])))
        fig, ax = plt.subplots(figsize=(7, 4))
        for (name, e), color in zip(wanted.items(), ("#D55E00", "#0072B2")):
            pts = traces[name]
            if pts:
                ax.plot([p[0] for p in pts], [p[1] for p in pts], color=color,
                        label=f"{e['tag']} ({e['event']})")
                ax.axhline(float(e["end_c"]), color=color, linestyle="--", linewidth=0.8)
        ax.set_xlabel("time since command (s)")
        ax.set_ylabel("plate temperature (C)")
        ax.set_title("Big bumps, closed loop (dashed: target)")
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, "fig_big_steps.png"), dpi=150)
        plt.close(fig)
        written.append("fig_big_steps.png")
    return written


def run_protocol(dev, cfg, outdir, meta):
    rec = Recorder(outdir)
    sess = Session(dev, cfg, rec)
    status = "completed"
    dew = low = None
    valid, skipped = [], []
    try:
        mode = dev.en_mode(cfg.en_mode)
        if mode is None or cfg.en_mode not in mode:
            raise Abort(f"EN_MODE check failed (reply {mode!r})")
        meta["en_mode_reply"] = mode
        dew, low = floor_info(cfg)
        bands = band_list(cfg)
        valid = [(lo, hi) for lo, hi in bands if lo >= low - 1e-9]
        skipped = [(lo, hi) for lo, hi in bands if lo < low - 1e-9]
        high = cfg.target_max_c
        clamped = low > cfg.target_min_c + 1e-9
        meta.update({"open_sweep_range_c": [cfg.open_min_c, cfg.open_max_c],
                     "open_analysis_range_c": [cfg.target_min_c, cfg.target_max_c],
                     "temperature_abort_enabled": False,
                     "dew_point_c": round(dew, 2), "cooling_floor_c": round(low, 2),
                     "valid_bands": valid, "skipped_bands": skipped})
        print(f"dew point {dew:.1f} C -> cooling floor {low:.1f} C; bands used {len(valid)}, skipped {len(skipped)}")

        def want(name):
            return not cfg.stages or name in cfg.stages

        sess.baseline()
        # Each round trip includes independent full-power heating and cooling.
        open_bands = [(cfg.open_min_c + i * cfg.band_width_c,
                       cfg.open_min_c + (i + 1) * cfg.band_width_c)
                      for i in range(int(round((cfg.open_max_c - cfg.open_min_c) / cfg.band_width_c)))]
        if want("open_heat") or want("open_cool"):
            for rt in range(1, cfg.open_round_trips + 1):
                for name, start, bound, direction in (
                    ("open_heat", cfg.open_min_c, cfg.open_max_c, 1),
                    ("open_cool", cfg.open_max_c, cfg.open_min_c, -1),
                ):
                    if not want(name):
                        continue
                    sess.pause_if_driven(f"before_{name}_{rt}")
                    if sess.prep(start):
                        edges = heat_edges(open_bands, start) if direction > 0 else cool_edges(open_bands, start, bound)
                        sess.sweep(direction, bound, edges, name, round_trip=rt)
        # 5 C ladder in round trips, then one reference step for drift
        ladder_ok = True
        if want("ladder") and valid:
            sess.pause_if_driven("before_ladder")
            print("5 C ladder")
            ladder_ok = run_ladder(sess, cfg, valid)
        if ladder_ok and cfg.include_ref_end and valid and want("ref_end"):
            a, b = valid[0]
            print("reference step at the end")
            if sess.prep(a):
                sess.step(a, b, "ref_end", cfg.hold_5c_s)
    except Abort as exc:
        status = f"aborted: {exc}"
        print(f"ABORT: {exc}")
    except KeyboardInterrupt:
        status = "interrupted by operator"
        print("interrupted by operator")
    finally:
        dev.stop()
        summary = []
        if dew is not None:
            summary = summarize(cfg, dew, low, valid, skipped, rec.bands, rec.steps, rec.notes)
        if status == "completed" and (
            any("준비 이동" in note or "사다리 중단" in note for note in rec.notes)
            or any(event.get("kind") == "SWEEP" and event.get("reason") != "bound" for event in rec.events)
            or any(step.get("reason") != "held" for step in rec.steps)
        ):
            status = "incomplete: one or more stages skipped or unfinished"
        meta["status"] = status
        meta["ended"] = datetime.now().isoformat(timespec="seconds")
        meta["summary"] = summary
        rec.close(meta)
        if cfg.make_plots:
            try:
                written = make_figures(outdir)
                print("figures: " + (", ".join(written) if written else "no data to plot"))
            except Exception as exc:  # plotting must never break a run
                print(f"figures skipped: {exc}")
    return status, summary


# ---------- operator and entry point ----------

def ask_number(prompt, lo, hi):
    while True:
        raw = input(prompt).strip()
        try:
            value = float(raw)
        except ValueError:
            print("  숫자를 입력하세요.")
            continue
        if lo <= value <= hi:
            return value
        print(f"  {lo:g} - {hi:g} 범위로 입력하세요.")


def operator_checks(cfg, notes):
    print("\n시작 전 확인 (습도는 냉각 하한을 정하므로 필수입니다)")
    cfg.room_c = ask_number("  실내 온도 (C): ", -10, 45)
    cfg.room_rh_pct = ask_number("  실내 상대습도 (%): ", 0, 100)
    pump = input("  수냉 펌프와 라디에이터 팬이 돌고 있습니까? [y/N]: ").strip().lower()
    notes["water_loop_running"] = pump == "y"
    if pump != "y":
        raise SystemExit("수냉을 먼저 켜세요.")
    dew, low = floor_info(cfg)
    notes.update(room_c=cfg.room_c, room_rh_pct=cfg.room_rh_pct,
                 dew_point_c=round(dew, 2), cooling_floor_c=round(low, 2))
    print(f"  이슬점 {dew:.1f} C -> 측정 하한 {low:.1f} C (가열과 냉각 모두 이 온도부터)")
    skipped = [(lo, hi) for lo, hi in band_list(cfg) if lo < low - 1e-9]
    if skipped:
        print("  이 온도 아래는 측정하지 않습니다: " + ", ".join(f"{lo:g}-{hi:g} C" for lo, hi in skipped))
    print(f"  폐회로 범위: {low:.1f}-{cfg.target_max_c:g} C; 개방루프: {cfg.open_min_c:g}-{cfg.open_max_c:g} C")
    print("  주의: 온도 기준 자동 중단 없음; 5 C 도달 시 결로 위험을 독립적으로 확인해야 합니다.")


def firmware_meta():
    return {"firmware": FIRMWARE, "firmware_sha1": firmware_sha1()}


def main(argv=None):
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")  # a cp949 console must not crash on a stray symbol
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="run against the simulated chamber")
    ap.add_argument("--config", help="JSON file with Config overrides")
    ap.add_argument("--plan", choices=["full", "pilot"], default="full",
                    help="pilot: one heating sweep only (a short first check run)")
    ap.add_argument("--chamber-id", help="unique physical chamber identifier, recorded in metadata")
    ap.add_argument("--port", help="serial port (default /dev/arduino)")
    ap.add_argument("--outdir", help="output folder")
    ap.add_argument("--seed", type=int, help="random seed (used by the simulator)")
    ap.add_argument("--sim-fault", type=float, nargs=2, metavar=("T_START", "T_END"),
                    help="sim: plate reads invalid between these times (s)")
    args = ap.parse_args(argv)

    cfg = Config()
    if args.config:
        with open(args.config, encoding="utf-8") as f:
            overrides = json.load(f)
        for key, value in overrides.items():
            if key not in Config.__dataclass_fields__:
                raise SystemExit(f"unknown config key: {key}")
            setattr(cfg, key, value)
    if cfg.ladder_round_trips < 1 or cfg.open_round_trips < 1:
        raise SystemExit("ladder_round_trips and open_round_trips must be 1 or more")
    if args.port:
        cfg.port = args.port
    if args.seed is not None:
        cfg.seed = args.seed
    if args.plan == "pilot":
        cfg.stages = ("open_heat",)
        cfg.open_round_trips = 1
        cfg.include_ref_end = False

    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    outdir = args.outdir or os.path.join(HERE, "perf_runs", f"PerfBands_{stamp}")
    notes = {}
    chamber_id = (args.chamber_id or "").strip()
    if not args.sim and not chamber_id:
        chamber_id = input("챔버 번호 또는 고유 ID: ").strip()
        if not chamber_id:
            raise SystemExit("챔버 ID가 필요합니다. --chamber-id 또는 입력으로 지정하세요.")
    if args.sim:
        if cfg.room_c is None or cfg.room_rh_pct is None:
            raise SystemExit("--sim needs room_c and room_rh_pct in --config")
        dev = SimDevice(seed=cfg.seed, fault_window=tuple(args.sim_fault) if args.sim_fault else None)
    else:
        operator_checks(cfg, notes)
        dev = SerialDevice(cfg.port, cfg.baud)
        go = input("\n시작할까요? [y/N]: ").strip().lower()
        if go != "y":
            dev.close()
            raise SystemExit("시작하지 않았습니다.")
    meta = {
        "started": datetime.now().isoformat(timespec="seconds"),
        "script": os.path.basename(__file__),
        "sim": bool(args.sim),
        "chamber_id": chamber_id or None,
        **firmware_meta(),
        "device": dev.info(),
        "config": asdict(cfg),
        "operator_notes": notes,
        "outdir": outdir,
    }
    print(f"\n출력 폴더: {outdir}")
    try:
        status, summary = run_protocol(dev, cfg, outdir, meta)
    finally:
        dev.close()
    print(f"\n상태: {status}")
    for line in summary:
        print(line)
    return 0 if status == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
