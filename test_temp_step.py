import csv
from datetime import datetime
import os
import sys
import time
import maze


def main():
    json_dir = input("json file [test.json]: ").strip() or "test.json"
    peltier = maze.Peltier_module(json_dir)

    timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_filename = f"temp_test_{timestamp_str}.csv"

    print("\n=== 단독 온도 제어 테스트 & CSV 로거 ===")
    print(f"로그 저장 경로: {os.path.abspath(csv_filename)}")

    try:
        raw_target = input("목표 온도를 입력하세요 (°C) [25.0]: ").strip()
        target_temp = float(raw_target) if raw_target else 25.0

        raw_duration = input("테스트 지속 시간(분)을 입력하세요 [5]: ").strip()
        duration_min = float(raw_duration) if raw_duration else 5.0
    except ValueError:
        print("숫자 입력이 올바르지 않습니다.")
        peltier.close()
        return

    peltier.start_control()
    peltier.set_target_temperature(target_temp)

    start_time = time.time()
    end_time = start_time + duration_min * 60.0

    print(f"\n제어 시작: 목표 {target_temp}°C, {duration_min}분 동안 실행 (중단: Ctrl+C)")
    print("-" * 75)
    print(f"{'Time(s)':>8} | {'Target':>6} | {'Sensor':>6} | {'T_pred':>6} | {'u(%)':>6} | {'PWM':>6} | {'ADC':>5} | {'Ohm':>7}")
    print("-" * 75)

    with open(csv_filename, "w", newline="") as f:
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

        try:
            while time.time() < end_time:
                now = time.time() - start_time
                ctrl = peltier.get_ctrl()

                if ctrl is not None:
                    wr.writerow([
                        round(now, 2),
                        ctrl["target_temp"],
                        ctrl["sensor_temp"],
                        ctrl["t_pred"],
                        ctrl["rate"],
                        ctrl["u"],
                        ctrl["pwm"],
                        ctrl["heat_duty"],
                        ctrl["cool_duty"],
                        ctrl["f_hat"],
                        ctrl["i_term"],
                        ctrl["adc"],
                        ctrl["ohm"],
                    ])
                    f.flush()

                    print(
                        f"{now:8.1f} | {ctrl['target_temp']:6.1f} | {ctrl['sensor_temp']:6.2f} | "
                        f"{ctrl['t_pred']:6.2f} | {ctrl['u']*100:6.1f} | {int(ctrl['pwm']):6d} | "
                        f"{ctrl['adc']:5.0f} | {ctrl['ohm']:7.0f}"
                    )
                time.sleep(0.5)

        except KeyboardInterrupt:
            print("\n[사용자 중단 (Ctrl+C)]")
        finally:
            peltier.stop_control()
            peltier.close()

    print("-" * 75)
    print(f"테스트 완료. CSV 파일이 저장되었습니다: {csv_filename}\n")


if __name__ == "__main__":
    main()
