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
        "average_temp",
        "heat_duty",
        "cool_duty",
    ])

    start_date = datetime.now()
    print("csv writing start", start_date)
    time_end = start_time + minutes * 60

    while not stop_event.is_set():
        now = time.time() - start_time
        with dict_lock:
            sensor_temp = shared_data.get("sensor_temp")
            avg_temp = shared_data.get("average_temp")
            target_temp = shared_data.get("target_temp")
            heat_duty = shared_data.get("heat_duty", 0)
            cool_duty = shared_data.get("cool_duty", 0)

        wr.writerow([now, target_temp, sensor_temp, avg_temp, heat_duty, cool_duty])
        f.flush()

        if now > time_end:
            print("data_export")
            print(datetime.now())
            break
        time.sleep(0.5)

    f.close()
