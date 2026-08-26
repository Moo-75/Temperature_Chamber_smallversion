import csv
from datetime import datetime
import time


def write_every_n_miliseconds(file_name, minutes, shared_data: dict, dict_lock, start_time, stop_event):
    f = open(file_name, "a", newline="")
    wr = csv.writer(f)
    wr.writerow([
        "time(s)",
        "target_temp",
        "sensor_temp",
        "t_pred",
        "rate_c_per_s",
        "u",
        "pwm",
        "heat_duty",
        "cool_duty",
        "f_hat",
        "i_term",
        "adc",
        "ohm",
    ])

    start_date = datetime.now()
    print("csv writing start", start_date)
    time_end = start_time + minutes * 60

    while not stop_event.is_set():
        now = time.time() - start_time
        with dict_lock:
            sensor_temp = shared_data.get("sensor_temp")
            target_temp = shared_data.get("target_temp")
            heat_duty = shared_data.get("heat_duty", 0)
            cool_duty = shared_data.get("cool_duty", 0)
            t_pred = shared_data.get("t_pred")
            rate = shared_data.get("rate")
            u = shared_data.get("u", 0)
            pwm = shared_data.get("pwm", 0)
            f_hat = shared_data.get("f_hat")
            i_term = shared_data.get("i_term")
            adc = shared_data.get("adc")
            ohm = shared_data.get("ohm")

        wr.writerow([
            now,
            target_temp,
            sensor_temp,
            t_pred,
            rate,
            u,
            pwm,
            heat_duty,
            cool_duty,
            f_hat,
            i_term,
            adc,
            ohm,
        ])
        f.flush()

        if now > time_end:
            print("data_export")
            print(datetime.now())
            break
        time.sleep(0.5)

    f.close()
