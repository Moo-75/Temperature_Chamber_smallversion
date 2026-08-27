import csv
import json
import math
import os
import random
import sys
import threading
import time

import maze

SENSOR_POLL_WAIT_MS = 50
SENSOR_POLL_WAIT_SEC = SENSOR_POLL_WAIT_MS / 1000.0
TTL_PULSE_SEC = 0.05
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
            ttl_t = self._ttl_sync_pulse()
            print(f"[TTL] start pulse t={ttl_t:.3f}s")
            self._log_ttl_event("TTLStart", ttl_t)
            self.task()
        except Exception as e:
            print(f"[Task Error] {e}")
        finally:
            ttl_t = self._ttl_sync_pulse()
            print(f"[TTL] end pulse t={ttl_t:.3f}s")
            self._log_ttl_event("TTLEnd", ttl_t)
            status_stop.set()
            temp_status_proc.join(timeout=2.0)
            sys.stdout.write("\n")
            sys.stdout.flush()
            print("done")
            self.led.off()
            self.ttl.out_low()

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

    def _ttl_sync_pulse(self):
        t = time.time() - self.start_time
        self.ttl.pulse(TTL_PULSE_SEC)
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
        task_time=60,
    ):
        """
        TL shared core. Single poke, trial-based.
        LED is ON during choice_window and OFF during feedback_window / ITI.
        """
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

        shared_start_temp = None
        with self.dict_lock:
            shared_start_temp = self.shared_data.get("initial_target_temp")
        shared_start_temp = self._sanitize_temperature(shared_start_temp)
        if shared_start_temp is not None:
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

        self.peltier_queue.put(("SET_TEMP", start_temp))
        self.peltier_queue.put(("SET_ATTENUATION_DIRECT", (0.0, temp_min, temp_max)))
        with self.dict_lock:
            self.shared_data["target_temp"] = start_temp

        trial = 0
        start_Ex = time.time()

        curr_temp, target_temp = self._get_shared_temperatures()
        if target_temp is None:
            target_temp = start_temp
        if curr_temp is None:
            curr_temp = target_temp

        dt_row = [
            self.mouseid,
            self.day,
            self.trainingstep,
            trial,
            time.time() - self.start_time,
            "SessionStart",
            curr_temp,
            target_temp,
            "n",
            "n",
            "n",
            "n",
            "n",
        ]
        self.TrialData2CSV2(directory, file_name_td, dt_row, col_name_td)

        session_done = False
        while (time.time() - start_Ex) < task_time * 60 and not session_done:
            trial += 1
            outcome_delta = "n"
            outcome_target = "n"

            self.peltier_queue.put(("SET_ATTENUATION_DIRECT", (0.0, temp_min, temp_max)))
            curr_temp, target_temp = self._get_shared_temperatures()
            print(f"\n[TL] --- Trial {trial} start (no drift) ---")
            dt_row = [
                self.mouseid,
                self.day,
                self.trainingstep,
                trial,
                time.time() - self.start_time,
                "TrialStart",
                curr_temp,
                target_temp,
                "n",
                "n",
                "n",
                "n",
                "n",
            ]
            self.TrialData2CSV2(directory, file_name_td, dt_row, col_name_td)

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

            self.led.off()

            if choice:
                curr_temp, target_temp = self._get_shared_temperatures()
                if curr_temp is None:
                    curr_temp = target_temp if target_temp is not None else start_temp
                bump = next_bump()
                base_temp = outcome_base_temp(curr_temp, target_temp)
                new_target = max(temp_min, min(base_temp + bump, temp_max))
                rt = round(poke_t - (cw_start - self.start_time), 3)

                self.peltier_queue.put(("SET_ATTENUATION_DIRECT", (0.0, temp_min, temp_max)))
                self.peltier_queue.put(("SET_TEMP", new_target))
                outcome_delta = bump
                outcome_target = new_target

                print(
                    f"[TL] trial {trial}: Poke, target={base_temp:.3f}°C + bump {bump} "
                    f"-> target {new_target:.3f}°C (RT={rt}s)"
                )
                dt_row = [
                    self.mouseid,
                    self.day,
                    self.trainingstep,
                    trial,
                    poke_t,
                    "Poke",
                    curr_temp,
                    new_target,
                    "poke",
                    bump,
                    rt,
                    outcome_delta,
                    outcome_target,
                ]
                self.TrialData2CSV2(directory, file_name_td, dt_row, col_name_td)
            else:
                if not session_done:
                    curr_temp, target_temp = self._get_shared_temperatures()
                    if curr_temp is None:
                        curr_temp = target_temp if target_temp is not None else start_temp
                    drop = next_no_choice_drop()
                    base_temp = outcome_base_temp(curr_temp, target_temp)
                    new_target = max(temp_min, min(base_temp - drop, temp_max))
                    self.peltier_queue.put(("SET_ATTENUATION_DIRECT", (0.0, temp_min, temp_max)))
                    self.peltier_queue.put(("SET_TEMP", new_target))
                    outcome_delta = -drop
                    outcome_target = new_target
                    print(
                        f"[TL] trial {trial}: NoChoice, target={base_temp:.3f} - drop {drop} "
                        f"-> target {new_target:.3f}"
                    )
                    dt_row = [
                        self.mouseid,
                        self.day,
                        self.trainingstep,
                        trial,
                        time.time() - self.start_time,
                        "NoChoice",
                        curr_temp,
                        new_target,
                        "n",
                        -drop,
                        "n",
                        outcome_delta,
                        outcome_target,
                    ]
                    self.TrialData2CSV2(directory, file_name_td, dt_row, col_name_td)

            if session_done:
                break

            fb_start = time.time()
            while (time.time() - fb_start) < feedback_window:
                if (time.time() - start_Ex) >= task_time * 60:
                    session_done = True
                    break
                time.sleep(0.05)

            curr_temp, target_temp = self._get_shared_temperatures()
            dt_row = [
                self.mouseid,
                self.day,
                self.trainingstep,
                trial,
                time.time() - self.start_time,
                "FeedbackEnd",
                curr_temp,
                target_temp,
                "poke" if choice else "n",
                "n",
                "n",
                outcome_delta,
                outcome_target,
            ]
            self.TrialData2CSV2(directory, file_name_td, dt_row, col_name_td)

            if session_done:
                break

        curr_temp, target_temp = self._get_shared_temperatures()
        if target_temp is None:
            target_temp = start_temp
        if curr_temp is None:
            curr_temp = target_temp
        dt_row = [
            self.mouseid,
            self.day,
            self.trainingstep,
            trial,
            time.time() - self.start_time,
            "SessionEnd",
            curr_temp,
            target_temp,
            "n",
            "n",
            "n",
            "n",
            "n",
        ]
        self.TrialData2CSV2(directory, file_name_td, dt_row, col_name_td)
        self.stop_event.set()
        print("=== TL Session Ended ===")

    def TL1(self):
        print("=== TL1: Temperature lift (+4.5/5/5.5, -1.5/-2/-2.5, 20s/20s) ===")
        self._run_temperature_lift(
            bump_choices=(4.5, 5.0, 5.5),
            no_choice_drop_choices=(1.5, 2.0, 2.5),
            choice_window=20.0,
            feedback_window=20.0,
        )

    def TL2(self):
        print("=== TL2: Temperature lift (+3/3.5/4, -1.5/-2/-2.5, 10s/20s) ===")
        self._run_temperature_lift(
            bump_choices=(3.0, 3.5, 4.0),
            no_choice_drop_choices=(1.5, 2.0, 2.5),
            choice_window=10.0,
            feedback_window=20.0,
        )


class TL1_Task(Task):
    def task(self):
        self.TL1()


class TL2_Task(Task):
    def task(self):
        self.TL2()
