import csv
import json
import math
import os
import random
import re
import sys
import threading
import time

import maze

SENSOR_POLL_WAIT_MS = 50
SENSOR_POLL_WAIT_SEC = SENSOR_POLL_WAIT_MS / 1000.0
TL_COL_NAME = [
    "mouseID",
    "Day",
    "Task",
    "Trial",
    "Time",
    "Event",
    "Current_Temp",
    "Target_Temp",
    "Choice",
    "Bump",
    "RT",
    "OutcomeDelta",
    "OutcomeTarget_Temp",
]


# ---------------------------------------------------------------------------
# New-stage presets (TLS -> TLY -> TLC -> TLM). Floor 10 °C in every stage.
# Bumps and drops share one step size within each task (TLS/TLC: 1 °C, TLM: 2 °C) so the target
# never ends up on a different sub-lattice depending on which outcomes occurred.
# ---------------------------------------------------------------------------
TLS_PARAMS = dict(
    bump_choices=(5.0, 6.0, 7.0),
    no_choice_drop_choices=(1.0, 2.0, 3.0),
    choice_window=30.0,
    feedback_window_choice=10.0,
    feedback_window_no_choice=0.0,
    start_temp=24.0,
    temp_min=10.0,
    temp_max=34.0,
    task_name="TLS",
    poke_flash_s=0.5,
)
TLC_PARAMS = dict(
    bump_choices=(5.0, 6.0, 7.0),
    no_choice_drop_choices=(1.0, 2.0, 3.0),
    choice_window=25.0,
    feedback_window=10.0,
    start_temp=24.0,
    temp_min=10.0,
    temp_max=34.0,
    task_name="TLC",
    poke_flash_s=0.5,
)
TLM_PARAMS = dict(
    bump_choices=(2.0, 4.0, 6.0),
    no_choice_drop_choices=(0.0, 2.0, 4.0),
    choice_window=20.0,
    feedback_window=10.0,
    start_temp=24.0,
    temp_min=10.0,
    temp_max=38.0,
    task_name="TLM",
    poke_flash_s=0.5,
)


# ---------------------------------------------------------------------------
# Yoke schedules: every session saves the targets it commanded so a later TLY
# session can replay them for the same mouse. They are stored OUTSIDE the session
# folders (migrate_to_server.py may delete those after copying) in
#   <yoke_dir>/<mouse>/yoke_<mouse>_<session>_<task>_<timestamp>.json
# yoke_dir = test.json "yoke_dir" if set, else <folder above this code>/_yoke_schedules.
# ---------------------------------------------------------------------------
YOKE_SOURCE_TASKS = ("TLS",)
_STAMP_RE = re.compile(r"(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})")


def default_yoke_dir(config=None):
    if config and config.get("yoke_dir"):
        return os.path.abspath(os.path.expanduser(config["yoke_dir"]))
    return os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "_yoke_schedules"))


def save_yoke_file(config, mouse_id, session, td_base, task, schedule, task_time_min,
                   duration_s, completed, n_trials, n_pokes, temp_min, temp_max):
    m = _STAMP_RE.search(os.path.basename(td_base))
    stamp = m.group(1) if m else time.strftime("%Y-%m-%d_%H-%M-%S")
    folder = os.path.join(default_yoke_dir(config), str(mouse_id))
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, f"yoke_{mouse_id}_{session}_{task}_{stamp}.json")
    with open(path, "w") as f:
        json.dump({
            "mouse": mouse_id, "session": session, "task": task, "started_at": stamp,
            "td_file": os.path.abspath(td_base + "_trial-wise.csv"),
            "task_time_min": task_time_min, "duration_s": round(duration_s, 3), "completed": bool(completed),
            "n_trials": n_trials, "n_pokes": n_pokes, "temp_min": temp_min, "temp_max": temp_max,
            "schedule": schedule,
        }, f, indent=1)
    return path


def load_yoke_file(path):
    with open(path) as f:
        data = json.load(f)
    if not data.get("schedule"):
        raise ValueError(f"yoke file has no schedule: {path}")
    data["path"] = os.path.abspath(path)
    return data


def list_yoke_sources(mouse_id, config=None, source_tasks=YOKE_SOURCE_TASKS):
    """All completed source sessions of this mouse, newest first: [(started_at, path, data)]."""
    folder = os.path.join(default_yoke_dir(config), str(mouse_id))
    out = []
    if not os.path.isdir(folder):
        return out
    for name in os.listdir(folder):
        if not (name.startswith("yoke_") and name.endswith(".json")):
            continue
        path = os.path.join(folder, name)
        try:
            data = load_yoke_file(path)
        except Exception:
            continue
        if data.get("mouse") == mouse_id and data.get("task") in source_tasks and data.get("completed"):
            out.append((data.get("started_at", ""), path, data))
    return sorted(out, key=lambda x: x[0], reverse=True)


def find_yoke_source(mouse_id, config=None, source_tasks=YOKE_SOURCE_TASKS):
    """Path of the most recent completed TLS session of this mouse, or None."""
    found = list_yoke_sources(mouse_id, config, source_tasks)
    return found[0][1] if found else None


class Task:
    def __init__(
        self,
        json_dir,
        TrialData_file_name,
        mouseid,
        session,
        shared_data,
        dict_lock,
        start_time,
        peltier_queue,
        stop_event,
    ):
        with open(json_dir, "r") as config:
            self.data = json.load(config)
        self.json_dir = json_dir
        self.sensor = maze.Sensor(json_dir)
        self.led = maze.LED(json_dir)
        self.ttl = maze.TTL(json_dir)
        self.file_trialdata = TrialData_file_name
        self.mouseid = mouseid
        try:
            self.trainingstep = session.split("_")[1]
            self.day = int(session.split("_")[0].split("d")[1])
        except (IndexError, ValueError) as exc:
            raise ValueError("session must look like d18_p1 (e.g. d1_p1)") from exc
        self.shared_data = shared_data
        self.dict_lock = dict_lock
        self.start_time = start_time
        self.peltier_queue = peltier_queue
        self.stop_event = stop_event

    def task(self):
        pass

    def _temp_status_monitor(self, stop_event, interval_sec=30.0):
        pad = 80
        while True:
            curr_temp, target_temp = self._get_shared_temperatures()
            elapsed_min = (time.time() - self.start_time) / 60.0
            curr_s = f"{curr_temp:.2f}" if curr_temp is not None else "n/a"
            target_s = f"{target_temp:.2f}" if target_temp is not None else "n/a"
            msg = f"[TEMP] t={elapsed_min:.1f}min  avg={curr_s}°C  target={target_s}°C"
            sys.stdout.write("\r" + msg.ljust(pad))
            sys.stdout.flush()
            if stop_event.wait(interval_sec):
                break

    def run(self):
        status_stop = threading.Event()
        temp_status_proc = threading.Thread(
            target=self._temp_status_monitor,
            args=(status_stop,),
            daemon=True,
        )
        temp_status_proc.start()
        try:
            ttl_t = self._ttl_session_high()
            print(f"[TTL] HIGH t={ttl_t:.3f}s (session start)")
            self._log_ttl_event("TTLStart", ttl_t)
            self.task()
        except Exception as e:
            print(f"[Task Error] {e}")
        finally:
            ttl_t = self._ttl_session_low()
            print(f"[TTL] LOW t={ttl_t:.3f}s (session end)")
            self._log_ttl_event("TTLEnd", ttl_t)
            status_stop.set()
            temp_status_proc.join(timeout=2.0)
            sys.stdout.write("\n")
            sys.stdout.flush()
            print("done")
            self.led.off()

    def _sanitize_temperature(self, value):
        if value is None:
            return None
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) else None

    def _get_shared_temperatures(self):
        with self.dict_lock:
            curr_temp = self.shared_data["average_temp"]
            target_temp = self.shared_data["target_temp"]
        return self._sanitize_temperature(curr_temp), self._sanitize_temperature(target_temp)

    def _ttl_session_high(self):
        t = time.time() - self.start_time
        self.ttl.out_high()
        return t

    def _ttl_session_low(self):
        t = time.time() - self.start_time
        self.ttl.out_low()
        return t

    def _log_ttl_event(self, event, t):
        curr_temp, target_temp = self._get_shared_temperatures()
        path = self.file_trialdata + "_trial-wise.csv"
        directory = os.path.dirname(path) or "."
        row = [
            self.mouseid,
            self.day,
            self.trainingstep,
            0,
            t,
            event,
            curr_temp if curr_temp is not None else "n",
            target_temp if target_temp is not None else "n",
            "n",
            "n",
            "n",
            "n",
            "n",
        ]
        self.TrialData2CSV2(directory, path, row, TL_COL_NAME)

    def TrialData2CSV2(self, directory, filename, row, col):
        os.makedirs(directory, exist_ok=True)
        path = filename if filename.endswith(".csv") else filename + ".csv"
        write_header = not os.path.exists(path) or os.path.getsize(path) == 0
        with open(path, "a", newline="") as f:
            wr = csv.writer(f)
            if write_header:
                wr.writerow(col)
            wr.writerow(row)

    def _run_temperature_lift(
        self,
        bump_choices,
        start_temp=10.0,
        temp_min=10.0,
        temp_max=40.0,
        no_choice_drop_choices=(2.5, 3.0, 3.5),
        bump_balance_block=20,
        no_choice_balance_block=20,
        choice_window=20.0,
        feedback_window=40.0,
        feedback_window_choice=None,
        feedback_window_no_choice=None,
        task_time=60,
        task_name="TL",
        poke_flash_s=0.0,
        poke_flash_brightness=100.0,
        probe_min=0.0,
        probe_temp=None,
        yoke=None,
    ):
        """
        TL shared core. Single poke, trial-based.
        LED is ON during choice_window and OFF during feedback.

        poke_flash_s > 0: a poke is marked immediately by a bright LED flash
            (poke_flash_brightness %, poke_flash_s seconds) before the LED goes off,
            so the poke has its own instant feedback distinct from a missed window.
        probe_min > 0: for the first probe_min minutes the target is held at
            probe_temp (default start_temp) and outcomes are NOT applied (Bump and
            OutcomeDelta logged as 0). Trials are classified by their TrialStart time.
            "ProbeStart" / "ProbeEnd" rows mark the probe in the TD file.
        yoke: dict loaded by load_yoke_file(). The target follows the recorded
            target schedule of a previous session (replayed in time from session
            start) and pokes / missed windows never change the target.

        Every session writes <TD>_config.json and, at the end, a target schedule
        (yoke file) under the yoke directory so later TLY sessions can replay it.
        """
        fb_after_choice = (
            feedback_window if feedback_window_choice is None else feedback_window_choice
        )
        fb_after_no_choice = (
            feedback_window
            if feedback_window_no_choice is None
            else feedback_window_no_choice
        )
        file_name_td = self.file_trialdata + "_trial-wise.csv"
        directory = os.path.dirname(file_name_td)
        col_name_td = TL_COL_NAME

        no_choice_drop_bag = []
        bump_bag = []
        bump_block_index = 0
        no_choice_drop_block_index = 0

        def make_balanced_bag(values, block_size, block_index):
            values = list(values)
            if not values:
                return []
            block_size = max(1, int(block_size))
            base_count = block_size // len(values)
            extra_count = block_size % len(values)
            counts = [base_count] * len(values)
            for i in range(extra_count):
                counts[(block_index + i) % len(values)] += 1
            bag = []
            for value, count in zip(values, counts):
                bag.extend([value] * count)
            random.shuffle(bag)
            return bag

        def next_bump():
            nonlocal bump_bag, bump_block_index
            if not bump_bag:
                bump_bag = make_balanced_bag(
                    bump_choices, bump_balance_block, bump_block_index
                )
                bump_block_index += 1
            return bump_bag.pop()

        def next_no_choice_drop():
            nonlocal no_choice_drop_bag, no_choice_drop_block_index
            if not no_choice_drop_bag:
                no_choice_drop_bag = make_balanced_bag(
                    no_choice_drop_choices,
                    no_choice_balance_block,
                    no_choice_drop_block_index,
                )
                no_choice_drop_block_index += 1
            return no_choice_drop_bag.pop()

        def outcome_base_temp(curr_temp, target_temp):
            if target_temp is not None:
                return target_temp
            if curr_temp is not None:
                return curr_temp
            return start_temp

        def log_row(trial_no, t, event, curr, target, choice="n", bump="n", rt="n", delta="n", out_target="n"):
            row = [
                self.mouseid, self.day, self.trainingstep, trial_no, t, event,
                curr if curr is not None else "n", target if target is not None else "n",
                choice, bump, rt, delta, out_target,
            ]
            self.TrialData2CSV2(directory, file_name_td, row, col_name_td)

        shared_start_temp = None
        with self.dict_lock:
            shared_start_temp = self.shared_data.get("initial_target_temp")
        shared_start_temp = self._sanitize_temperature(shared_start_temp)
        if shared_start_temp is not None and yoke is None:
            if shared_start_temp < temp_min or shared_start_temp > temp_max:
                clamped_start = max(temp_min, min(shared_start_temp, temp_max))
                print(
                    f"[TL] initial target {shared_start_temp}°C is outside "
                    f"{temp_min}-{temp_max}°C; clamped to {clamped_start}°C"
                )
                start_temp = clamped_start
            else:
                start_temp = shared_start_temp
            print(f"[TL] using maintemp set-on target as start_temp: {start_temp}°C")

        if probe_min > 0 and probe_temp is None:
            probe_temp = start_temp
        if yoke is not None:
            first_target = float(yoke["schedule"][0][1])
        elif probe_min > 0:
            first_target = float(probe_temp)
        else:
            first_target = start_temp

        os.makedirs(directory or ".", exist_ok=True)
        with open(self.file_trialdata + "_config.json", "w") as f:
            json.dump({
                "task": task_name, "bump_choices": list(bump_choices),
                "no_choice_drop_choices": list(no_choice_drop_choices),
                "start_temp": first_target, "temp_min": temp_min, "temp_max": temp_max,
                "choice_window_s": choice_window, "feedback_after_poke_s": fb_after_choice,
                "feedback_after_no_choice_s": fb_after_no_choice, "task_time_min": task_time,
                "poke_flash_s": poke_flash_s, "poke_flash_brightness": poke_flash_brightness,
                "probe_min": probe_min, "probe_temp": probe_temp if probe_min > 0 else None,
                "yoke_source": None if yoke is None else yoke.get("path"),
                "yoke_source_session": None if yoke is None else yoke.get("session"),
            }, f, indent=1)

        # every target the task commands, as [seconds from task start, target]
        target_log = []
        start_Ex = time.time()

        def command_target(value):
            self.peltier_queue.put(("SET_TEMP", value))
            target_log.append([round(time.time() - start_Ex, 3), float(value)])

        self.peltier_queue.put(("SET_ATTENUATION_DIRECT", (0.0, temp_min, temp_max)))
        command_target(first_target)
        with self.dict_lock:
            self.shared_data["target_temp"] = first_target

        replay_stop = threading.Event()
        replay_thread = None
        if yoke is not None:
            def replay():
                for t_rel, value in yoke["schedule"][1:]:
                    wait = start_Ex + float(t_rel) - time.time()
                    if wait > 0 and replay_stop.wait(wait):
                        return
                    if replay_stop.is_set():
                        return
                    command_target(float(value))
            replay_thread = threading.Thread(target=replay, daemon=True)
            replay_thread.start()
            print(f"[{task_name}] yoked to {yoke.get('session')} ({len(yoke['schedule'])} target steps)")

        trial = 0
        n_pokes = 0
        probe_open = probe_min > 0 and yoke is None

        curr_temp, target_temp = self._get_shared_temperatures()
        if target_temp is None:
            target_temp = first_target
        if curr_temp is None:
            curr_temp = target_temp
        log_row(trial, time.time() - self.start_time, "SessionStart", curr_temp, target_temp)
        if probe_open:
            log_row(trial, time.time() - self.start_time, "ProbeStart", curr_temp, first_target)
            print(f"[{task_name}] probe: target held at {first_target}°C, outcomes off for {probe_min} min")

        session_done = False
        while (time.time() - start_Ex) < task_time * 60 and not session_done:
            trial += 1
            outcome_delta = "n"
            outcome_target = "n"

            trial_t0 = time.time()
            in_probe = probe_min > 0 and (trial_t0 - start_Ex) < probe_min * 60
            if probe_open and not in_probe:
                curr_temp, target_temp = self._get_shared_temperatures()
                log_row(0, time.time() - self.start_time, "ProbeEnd", curr_temp, target_temp)
                print(f"[{task_name}] probe ended; outcomes on")
                probe_open = False
            outcomes_live = yoke is None and not in_probe

            self.peltier_queue.put(("SET_ATTENUATION_DIRECT", (0.0, temp_min, temp_max)))
            curr_temp, target_temp = self._get_shared_temperatures()
            print(f"\n[{task_name}] --- Trial {trial} start (no drift) ---")
            log_row(trial, time.time() - self.start_time, "TrialStart", curr_temp, target_temp)

            cw_start = time.time()
            self.led.on()
            prev_poke = self.sensor.poked()
            choice = False
            poke_t = None
            while True:
                now = time.time()
                if (now - start_Ex) >= task_time * 60:
                    session_done = True
                    break
                if (now - cw_start) >= choice_window:
                    break

                poked = self.sensor.poked()
                if poked and not prev_poke:
                    choice = True
                    poke_t = now - self.start_time
                    break
                prev_poke = poked
                time.sleep(SENSOR_POLL_WAIT_SEC)

            flash_until = None
            if choice and poke_flash_s > 0:
                self.led.on(poke_flash_brightness)
                flash_until = time.time() + poke_flash_s
            else:
                self.led.off()

            if choice:
                n_pokes += 1
                curr_temp, target_temp = self._get_shared_temperatures()
                if curr_temp is None:
                    curr_temp = target_temp if target_temp is not None else start_temp
                base_temp = outcome_base_temp(curr_temp, target_temp)
                if outcomes_live:
                    bump = next_bump()
                    new_target = max(temp_min, min(base_temp + bump, temp_max))
                    command_target(new_target)
                else:
                    bump = 0.0
                    new_target = base_temp
                rt = round(poke_t - (cw_start - self.start_time), 3)
                outcome_delta = bump
                outcome_target = new_target

                print(
                    f"[{task_name}] trial {trial}: Poke, target={base_temp:.3f}°C + bump {bump} "
                    f"-> target {new_target:.3f}°C (RT={rt}s)"
                )
                log_row(trial, poke_t, "Poke", curr_temp, new_target, "poke", bump, rt, outcome_delta, outcome_target)
            else:
                if not session_done:
                    curr_temp, target_temp = self._get_shared_temperatures()
                    if curr_temp is None:
                        curr_temp = target_temp if target_temp is not None else start_temp
                    base_temp = outcome_base_temp(curr_temp, target_temp)
                    if outcomes_live:
                        drop = next_no_choice_drop()
                        new_target = max(temp_min, min(base_temp - drop, temp_max))
                        command_target(new_target)
                    else:
                        drop = 0.0
                        new_target = base_temp
                    outcome_delta = -drop if drop else 0.0
                    outcome_target = new_target
                    print(
                        f"[{task_name}] trial {trial}: NoChoice, target={base_temp:.3f} - drop {drop} "
                        f"-> target {new_target:.3f}"
                    )
                    log_row(trial, time.time() - self.start_time, "NoChoice", curr_temp, new_target,
                            "n", outcome_delta, "n", outcome_delta, outcome_target)

            if session_done:
                if flash_until is not None:
                    while time.time() < flash_until:
                        time.sleep(0.01)
                    self.led.off()
                break

            this_feedback = fb_after_choice if choice else fb_after_no_choice
            fb_start = time.time()
            while (time.time() - fb_start) < this_feedback:
                if flash_until is not None and time.time() >= flash_until:
                    self.led.off()
                    flash_until = None
                if (time.time() - start_Ex) >= task_time * 60:
                    session_done = True
                    break
                time.sleep(0.05)
            if flash_until is not None:
                while time.time() < flash_until:
                    time.sleep(0.01)
                self.led.off()

            curr_temp, target_temp = self._get_shared_temperatures()
            log_row(trial, time.time() - self.start_time, "FeedbackEnd", curr_temp, target_temp,
                    "poke" if choice else "n", "n", "n", outcome_delta, outcome_target)

            if session_done:
                break

        replay_stop.set()
        if replay_thread is not None:
            replay_thread.join(timeout=1.0)

        curr_temp, target_temp = self._get_shared_temperatures()
        if target_temp is None:
            target_temp = first_target
        if curr_temp is None:
            curr_temp = target_temp
        log_row(trial, time.time() - self.start_time, "SessionEnd", curr_temp, target_temp)

        duration_s = time.time() - start_Ex
        try:
            path = save_yoke_file(
                self.data, self.mouseid, self.trainingstep, self.file_trialdata, task_name,
                schedule=target_log, task_time_min=task_time, duration_s=duration_s,
                completed=duration_s >= task_time * 60 - 5.0, n_trials=trial, n_pokes=n_pokes,
                temp_min=temp_min, temp_max=temp_max,
            )
            print(f"[{task_name}] target schedule saved for yoking: {path}")
        except Exception as e:  # never let bookkeeping break the session end
            print(f"[{task_name}] could not save yoke schedule: {e}")

        self.stop_event.set()
        print(f"=== {task_name} Session Ended ===")

    def TLS(self):
        print("=== TLS (Shaping): LED on except 10s after a poke; every LED-on poke +5/6/7°C (0.5s flash), "
              "30s without poke -1/-2/-3°C, start 24°C, 10-34°C ===")
        self._run_temperature_lift(**TLS_PARAMS)

    def TLSB(self):
        print("=== TLSB: TLS with a 10-min baseline first (target held at 24°C, pokes/misses do nothing) ===")
        self._run_temperature_lift(**{**TLS_PARAMS, "task_name": "TLSB", "probe_min": 10.0})

    def TLY(self):
        yoke = getattr(self, "yoke", None)
        if yoke is None:
            found = find_yoke_source(self.mouseid, self.data)
            if found is None:
                raise RuntimeError(f"TLY: no completed TLS session found for mouse {self.mouseid}")
            yoke = load_yoke_file(found)
        if yoke.get("mouse") != self.mouseid:
            raise RuntimeError(f"TLY: yoke file belongs to {yoke.get('mouse')}, not {self.mouseid}")
        print(f"=== TLY (Yoked): replaying target schedule of {yoke.get('session')} "
              f"({yoke.get('started_at')}); pokes flash but never change the temperature ===")
        self._run_temperature_lift(**{
            **TLS_PARAMS, "task_name": "TLY", "yoke": yoke,
            "temp_min": float(yoke.get("temp_min", TLS_PARAMS["temp_min"])),
            "temp_max": float(yoke.get("temp_max", TLS_PARAMS["temp_max"])),
            "task_time": float(yoke.get("task_time_min", 60)),
        })

    def TLC(self):
        print("=== TLC (Cue): TLS outcomes (+5/6/7, -1/-2/-3) with LED windows 25s / 10s / 10s, start 24°C, 10-34°C ===")
        self._run_temperature_lift(**TLC_PARAMS)

    def TLM(self):
        print("=== TLM (Main): +2/4/6, -0/-2/-4, 20s / 10s / 10s, start 24°C, 10-38°C ===")
        self._run_temperature_lift(**TLM_PARAMS)


    def TL0(self):
        print("=== TL0: Temperature lift (+5/6/7, -1.5/-2/-2.5, 30s / poke 20s / no-choice 10s, max 35°C) ===")
        self._run_temperature_lift(
            bump_choices=(5.0, 6.0, 7.0),
            no_choice_drop_choices=(1.5, 2.0, 2.5),
            choice_window=30.0,
            feedback_window_choice=20.0,
            feedback_window_no_choice=10.0,
            temp_max=35.0,
            task_name="TL0",
        )

    def TL1(self):
        print("=== TL1: Temperature lift (+4.5/5/5.5, -1.5/-2/-2.5, 20s/20s) ===")
        self._run_temperature_lift(
            bump_choices=(4.5, 5.0, 5.5),
            no_choice_drop_choices=(1.5, 2.0, 2.5),
            choice_window=20.0,
            feedback_window=20.0,
            task_name="TL1",
        )

    def TL2(self):
        print("=== TL2: Temperature lift (+3/3.5/4, -1.5/-2/-2.5, 10s/20s) ===")
        self._run_temperature_lift(
            bump_choices=(3.0, 3.5, 4.0),
            no_choice_drop_choices=(1.5, 2.0, 2.5),
            choice_window=10.0,
            feedback_window=20.0,
            task_name="TL2",
        )


class TLS_Task(Task):
    def task(self):
        self.TLS()


class TLSB_Task(Task):
    def task(self):
        self.TLSB()


class TLY_Task(Task):
    def task(self):
        self.TLY()


class TLC_Task(Task):
    def task(self):
        self.TLC()


class TLM_Task(Task):
    def task(self):
        self.TLM()




class TL0_Task(Task):
    def task(self):
        self.TL0()


class TL1_Task(Task):
    def task(self):
        self.TL1()


class TL2_Task(Task):
    def task(self):
        self.TL2()
