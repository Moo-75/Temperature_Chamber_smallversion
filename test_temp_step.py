import csv
from datetime import datetime
import os
from pathlib import Path
import sys
import time
import maze


def get_default_save_dir(timestamp_str: str) -> tuple[Path, Path]:
    """
    migrate_to_server.py가 인식할 수 있도록 2단계 계층 구조를 생성합니다:
    Desktop/TEMP_TEST (Parent) / test_YYYY-MM-DD_HH-MM-SS (Session) / Temperature_*.csv
    """
    home = Path.home()
    desktop = home / "Desktop"
    if desktop.is_dir():
        base_dir = desktop
    else:
        base_dir = Path.cwd().parent

    parent_dir = base_dir / "TEMP_TEST"
    session_dir = parent_dir / f"test_{timestamp_str}"
    session_dir.mkdir(parents=True, exist_ok=True)
    csv_file = session_dir / f"Temperature_test_{timestamp_str}.csv"
    return session_dir, csv_file


def main():
    json_dir = input("json file [test.json]: ").strip() or "test.json"
    peltier = maze.Peltier_module(json_dir)

    timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    session_dir, csv_filepath = get_default_save_dir(timestamp_str)

    print("\n=== 단독 온도 제어 테스트 & CSV 로거 ===")
    print(f"세션 폴더 경로: {session_dir}")
    print(f"CSV 저장 파일:  {csv_filepath}")
    print("(migrate_to_server.py 실행 시 'TEMP_TEST' 폴더를 선택하면 서버로 전송됩니다.)\n")

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

    with open(csv_filepath, "w", newline="") as f:
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
    print(f"테스트 완료. CSV 파일이 저장되었습니다: {csv_filepath}\n")


if __name__ == "__main__":
    main()
