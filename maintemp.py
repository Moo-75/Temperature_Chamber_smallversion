from datetime import datetime
from pytz import timezone
import os
import sys
import multiprocessing
from queue import Empty
import data_export
import math
import json
import csv
import maze
from task_temp import (
    TL0_Task, TL1_Task, TL2_Task, TLS_Task, TLSB_Task, TLY_Task, TLC_Task, TLM_Task,
    list_yoke_sources, load_yoke_file,
)
import time

SENSOR_LOG_INTERVAL_MS = 100
SENSOR_LOG_INTERVAL_SEC = SENSOR_LOG_INTERVAL_MS / 1000.0

_MAX_ATTENUATION_DT_SEC = 0.5
_MAX_QUEUE_DRAIN = 48


def is_valid_temperature(value):
    return value is not None and math.isfinite(value)


def sensor_worker(start_time, sensor_file_name, stop_event, json_dir):
    sensor = maze.Sensor(json_dir)
    with open(sensor_file_name, "a", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["time(s)", "sensor_poke"])
        while not stop_event.is_set():
            wr.writerow([time.time() - start_time, sensor.get()])
            f.flush()
            time.sleep(SENSOR_LOG_INTERVAL_SEC)


def _apply_peltier_queue_item(peltier, command, value, shared_data, dict_lock, result_queue):
    if command == "SET_TEMP":
        peltier.set_target_temperature(value)
    elif command == "SET_ON_WAIT":
        peltier.set_target_temperature(value)
        peltier.start_control()
        result_queue.put(
            peltier.temperature_seton(
                value, shared_data=shared_data, dict_lock=dict_lock
            )
        )
    elif command == "SET_ATTENUATION":
        peltier.set_temperature_attenuation(value)
        peltier.use_attenuation_func = True
    elif command == "SET_ATTENUATION_DIRECT":
        if isinstance(value, tuple) and len(value) == 3:
            rate, att_min, att_max = value
            peltier.set_temperature_attenuation(rate)
            peltier.att_min = att_min
            peltier.att_max = att_max
        else:
            peltier.set_temperature_attenuation(value)
            peltier.att_min = 15.0
            peltier.att_max = 45.0
        peltier.use_attenuation_func = False
    elif command == "TEMP_UPDOWN":
        peltier.temp_updown(value)
    elif command == "STOP":
        peltier.stop_control()
        return "stop"
    return None


def peltier_worker(command_queue, result_queue, shared_data, dict_lock, stop_event, json_dir):
    peltier = maze.Peltier_module(json_dir)
    peltier.use_attenuation_func = False
    peltier.start_control()
    time.sleep(0.1)
    prev_time = time.time()
    command = None

    while not stop_event.is_set():
        try:
            batch = []
            while len(batch) < _MAX_QUEUE_DRAIN:
                try:
                    batch.append(command_queue.get_nowait())
                except Empty:
                    break
            command = batch[-1][0] if batch else None
            for cmd_item in batch:
                command, value = cmd_item
                if _apply_peltier_queue_item(
                    peltier, command, value, shared_data, dict_lock, result_queue
                ) == "stop":
                    peltier.close()
                    return

            ctrl = peltier.get_ctrl()
            curr_temp = None if ctrl is None else ctrl["sensor_temp"]
            temps_valid = is_valid_temperature(curr_temp)

            with dict_lock:
                shared_data["sensor_temp"] = curr_temp if temps_valid else None
                shared_data["average_temp"] = curr_temp if temps_valid else None
                if ctrl is None:
                    shared_data["heat_duty"] = 0.0
                    shared_data["cool_duty"] = 0.0
                    shared_data["t_pred"] = None
                    shared_data["rate"] = None
                    shared_data["u"] = 0.0
                    shared_data["f_hat"] = None
                    shared_data["i_term"] = None
                    shared_data["adc"] = None
                    shared_data["ohm"] = None
                    shared_data["pwm"] = 0
                else:
                    shared_data["heat_duty"] = ctrl["heat_duty"]
                    shared_data["cool_duty"] = ctrl["cool_duty"]
                    shared_data["t_pred"] = ctrl["t_pred"]
                    shared_data["rate"] = ctrl["rate"]
                    shared_data["u"] = ctrl["u"]
                    shared_data["f_hat"] = ctrl["f_hat"]
                    shared_data["i_term"] = ctrl["i_term"]
                    shared_data["adc"] = ctrl["adc"]
                    shared_data["ohm"] = ctrl["ohm"]
                    shared_data["pwm"] = ctrl["pwm"]

            target_temperature = peltier.target_temp
            elapsed_since_prev = time.time() - prev_time
            if elapsed_since_prev < 0:
                elapsed_since_prev = 0.0
            elapsed_since_prev = min(elapsed_since_prev, _MAX_ATTENUATION_DT_SEC)
            attenuation_delta = peltier.attenuation * elapsed_since_prev
            new_target = target_temperature + attenuation_delta
            att_min = getattr(peltier, "att_min", 10.0)
            att_max = getattr(peltier, "att_max", 45.0)
            new_target = max(att_min, min(new_target, att_max))

            if abs(new_target - target_temperature) > 1e-4:
                peltier.set_target_temperature(new_target)

            with dict_lock:
                shared_data["target_temp"] = peltier.target_temp

            prev_time = time.time()
            time.sleep(0.1)
        except Exception as e:
            print(command)
            print(f"[Peltier Process] Error: {e}")

    peltier.close()


def check_starting(experiment_start, csv_write_dir, mouse_id, session):
    if experiment_start not in ("y", "Y"):
        raise ValueError(f"Experiment not confirmed (got '{experiment_start}'). Enter 'y' to start.")
    experiment_time = datetime.now(timezone("Asia/Seoul"))
    experiment_time_str = experiment_time.strftime("%Y-%m-%d_%H-%M-%S")
    TrialData_file_name = os.path.join(csv_write_dir, f"TD_{mouse_id}_{session}_{experiment_time_str}")
    Temperature_file_name = os.path.join(
        csv_write_dir, f"Temperature_{mouse_id}_{session}_{experiment_time_str}.csv"
    )
    SensorTime_file_name = os.path.join(
        csv_write_dir, f"SensorTime_{mouse_id}_{session}_{experiment_time_str}.csv"
    )
    return Temperature_file_name, TrialData_file_name, SensorTime_file_name


def create_path(csv_write_dir, file_name):
    os.makedirs(csv_write_dir, exist_ok=True)
    open(file_name, "w").close()
    return file_name


if __name__ == "__main__":
    json_dir = input("Please enter your json file :")

    with open(json_dir, "r") as json_:
        data = json.load(json_)

    while True:
        protocol = input("Please type protocol (e.g. OBT, TBT): ")
        protocol = "../" + protocol
        break

    mouse_id = input("Please enter the mouse id : ")
    session = input("Please enter the session (ex. d18_p1) : ")

    while mouse_id == "" or session == "":
        mouse_id = input("Please reenter the mouse id (do not use _ in the name) : ")
        session = input("Please reenter the session : ")

    temperature_seton = input("If do you need temperature set on, press y  ")
    need_seton = False
    initial_target_temp = None

    if temperature_seton in ("y", "Y"):
        target_temp = input("Please enter the target temperature : ")
        while True:
            try:
                target_temp = float(target_temp)
                need_seton = True
                initial_target_temp = target_temp
                break
            except ValueError:
                target_temp = input("Please reenter the target temperature : ")

    multi_var_manager = multiprocessing.Manager()
    shared_data = multi_var_manager.dict({
        "sensor_temp": 0.0,
        "average_temp": 0.0,
        "target_temp": 0.0,
        "heat_duty": 0.0,
        "cool_duty": 0.0,
        "t_pred": 0.0,
        "rate": 0.0,
        "u": 0.0,
        "f_hat": 0.0,
        "i_term": 0.0,
        "adc": 0.0,
        "ohm": 0.0,
        "pwm": 0,
        "initial_target_temp": initial_target_temp,
    })
    dict_lock = multiprocessing.Lock()
    stop_event = multi_var_manager.Event()
    peltier_queue = multiprocessing.Queue()
    result_queue = multiprocessing.Queue()
    peltier_process = multiprocessing.Process(
        target=peltier_worker,
        args=(peltier_queue, result_queue, shared_data, dict_lock, stop_event, json_dir),
    )
    peltier_process.start()

    if need_seton:
        peltier_queue.put(("SET_ON_WAIT", target_temp))
        try:
            seton_success = result_queue.get(timeout=300)
        except Empty:
            seton_success = False
        if not seton_success:
            stop_event.set()
            peltier_process.join(timeout=5)
            raise RuntimeError(f"Failed to reach target temperature {target_temp}.")

    while True:
        experiment_start = input("If the experiment start, press y  ")
        print(mouse_id, session)
        print(experiment_start)
        csv_write_dir = os.path.join(protocol, mouse_id + "_" + session)
        try:
            Temperature_file_name, TrialData_file_name, SensorTime_file_name = check_starting(
                experiment_start, csv_write_dir, mouse_id, session
            )
            break
        except ValueError as e:
            print(e)
    SensorTime_file_name = create_path(csv_write_dir, SensorTime_file_name)

    task_classes = {
        "TLS": TLS_Task,
        "TLSB": TLSB_Task,
        "TLY": TLY_Task,
        "TLC": TLC_Task,
        "TLM": TLM_Task,
        "TL0": TL0_Task,
        "TL1": TL1_Task,
        "TL2": TL2_Task,
    }

    while True:
        task = input(
            """
To start enter the task name you want to run
    [TLS]  Shaping: LED on except 10s after a poke; every LED-on poke +5/6/7°C (0.5s flash); 30s without poke -1/-2/-3°C (start 24°C, 10-34°C)
    [TLSB] TLS with a 10-min baseline first (held at 24°C, pokes do nothing) - use on day 1
    [TLY]  Yoked: replays this mouse's last completed TLS target schedule; pokes flash but never change temperature
    [TLC]  Cue: TLS outcomes with LED windows (25s / 10s / 10s, start 24°C, 10-34°C)
    [TLM]  Main: poke +2/4/6°C, no-choice -0/-2/-4°C (20s / 10s / 10s, start 24°C, 10-38°C)
    [TL0] TL0: poke-only, choice +5/6/7°C, no-choice -1.5/-2/-2.5°C (30s / poke 20s, no-choice 10s, max 35°C)
    [TL1] TL1: poke-only, choice +4.5/5/5.5°C, no-choice -1.5/-2/-2.5°C (20s / 20s)
    [TL2] TL2: poke-only, choice +3/3.5/4°C, no-choice -1.5/-2/-2.5°C (10s / 20s)
    [0] Exit
"""
        ).strip()
        if task == "0":
            print("Selected Exit")
            break
        if task not in task_classes:
            print("Wrong input, please try again\n")
            continue

        yoke = None
        if task == "TLY":
            # Resolve the yoke source BEFORE any logger starts, so a missing source
            # never leaves a half-started session behind.
            sources = list_yoke_sources(mouse_id, data)
            if not sources:
                print(f"[TLY] No completed TLS session found for mouse '{mouse_id}'. Run TLS first.")
                continue
            print(f"[TLY] Completed TLS sessions for {mouse_id} (newest first):")
            for i, (stamp, path, info) in enumerate(sources[:5]):
                print(f"   [{i}] {info.get('session')}  {stamp}  pokes={info.get('n_pokes')}  "
                      f"trials={info.get('n_trials')}  ({os.path.basename(path)})")
            pick = input("Press Enter to yoke to [0] (most recent), type another index, or 'n' to cancel: ").strip()
            if pick.lower() == "n":
                continue
            idx = int(pick) if pick.isdigit() and int(pick) < len(sources[:5]) else 0
            yoke = load_yoke_file(sources[idx][1])
            print(f"[TLY] yoked to {yoke.get('session')} ({yoke.get('started_at')})")

        start_time = time.time()
        sensor_process = multiprocessing.Process(
            target=sensor_worker,
            args=(start_time, SensorTime_file_name, stop_event, json_dir),
        )
        sensor_process.start()

        instance = task_classes[task](
            json_dir,
            TrialData_file_name,
            mouse_id,
            session,
            shared_data,
            dict_lock,
            start_time,
            peltier_queue,
            stop_event,
        )
        if yoke is not None:
            instance.yoke = yoke

        task_ = multiprocessing.Process(
            target=data_export.write_every_n_miliseconds,
            args=(
                Temperature_file_name,
                data["min"],
                shared_data,
                dict_lock,
                start_time,
                stop_event,
            ),
        )
        task_.start()
        instance.run()
        task_.join()
        break

    stop_event.set()
    print("Proccess made an end\nShutting down...\n")
    sys.exit()
