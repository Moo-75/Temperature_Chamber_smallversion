import csv
from datetime import datetime
import os
from pathlib import Path
import sys
import time
import numpy as np
import maze


def get_default_save_dir(timestamp_str: str) -> tuple[Path, Path]:
    """
    migrate_to_server.py가 인식할 수 있도록 2단계 계층 구조를 생성합니다:
    Desktop/TEMP_TEST (Parent) / test_YYYY-MM-DD_HH-MM-SS (Session) / Temperature_test_*.csv
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


PRESET_PROFILES = {
    "1": {
        "name": "Full Range Multi-Step (25°C -> 35°C -> 40°C -> 20°C -> 10°C -> 25°C)",
        "steps": [
            (25.0, 60),
            (35.0, 60),
            (40.0, 75),
            (20.0, 60),
            (10.0, 75),
            (25.0, 60),
        ],
    },
    "2": {
        "name": "Extreme Jump Test (25°C -> 40°C -> 10°C -> 25°C)",
        "steps": [
            (25.0, 45),
            (40.0, 90),
            (10.0, 100),
            (25.0, 60),
        ],
    },
    "3": {
        "name": "Precision Small Steps (25°C -> 28°C -> 22°C -> 26°C -> 25°C)",
        "steps": [
            (25.0, 45),
            (28.0, 45),
            (22.0, 45),
            (26.0, 45),
            (25.0, 45),
        ],
    },
}


def parse_custom_profile(text: str):
    """
    Format: "25:60, 35:60, 40:70, 10:80" (temp:duration_sec)
    """
    steps = []
    tokens = text.replace(";", ",").split(",")
    for t in tokens:
        t = t.strip()
        if not t:
            continue
        parts = t.split(":")
        if len(parts) == 2:
            temp = float(parts[0])
            dur = float(parts[1])
            steps.append((temp, dur))
        else:
            raise ValueError(f"올바르지 않은 형식입니다: '{t}' (예: 25:60)")
    return steps


def analyze_step_performance(step_records, target_temp, prev_target):
    """
    각 스텝 구간의 성능 지표 분석:
    - 도달 시간 (Settling time to ±0.5°C)
    - 오버슛/언더슛 (Peak error beyond target)
    - 정상상태 오차 및 표준편차 (마지막 20초)
    - 최대 온도 변화율 (Peak Rate °C/s)
    """
    if not step_records:
        return {}

    times = [r["time"] for r in step_records]
    t0 = times[0]
    rel_times = [t - t0 for t in times]
    temps = [r["sensor_temp"] for r in step_records]
    rates = [abs(r["rate"]) for r in step_records]

    start_temp = temps[0]
    is_heating = target_temp > (prev_target if prev_target is not None else start_temp)

    # 1. 도달 시간 (오차 ±0.5°C 이내 진입 시점)
    settling_time = None
    for rt, temp in zip(rel_times, temps):
        if abs(temp - target_temp) <= 0.5:
            settling_time = rt
            break

    # 2. 오버슛 / 언더슛
    if is_heating:
        peak_temp = max(temps)
        overshoot = max(0.0, peak_temp - target_temp)
    else:
        peak_temp = min(temps)
        overshoot = max(0.0, target_temp - peak_temp)

    # 3. 마지막 20초 안정화 상태 분석
    tail_records = [r for r in step_records if (r["time"] - t0) >= (rel_times[-1] - 20.0)]
    if not tail_records:
        tail_records = step_records[-10:]

    tail_temps = [r["sensor_temp"] for r in tail_records]
    mean_tail = float(np.mean(tail_temps))
    std_tail = float(np.std(tail_temps))
    ss_error = mean_tail - target_temp

    return {
        "target": target_temp,
        "duration": rel_times[-1],
        "start_temp": start_temp,
        "settling_time": settling_time,
        "peak_temp": peak_temp,
        "overshoot": overshoot,
        "ss_mean": mean_tail,
        "ss_std": std_tail,
        "ss_error": ss_error,
        "max_rate": max(rates) if rates else 0.0,
    }


def main():
    json_dir = input("json file [test.json]: ").strip() or "test.json"
    peltier = maze.Peltier_module(json_dir)

    timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    session_dir, csv_filepath = get_default_save_dir(timestamp_str)

    print("\n" + "=" * 80)
    print("      다중 온도 프로파일 자동 테스트 & 성능 분석기")
    print("=" * 80)
    print(f"세션 폴더: {session_dir}")
    print(f"CSV 저장:  {csv_filepath}")
    print("(테스트 완료 후 migrate_to_server.py로 서버에 바로 전송할 수 있습니다.)\n")

    print("테스트 프로파일을 선택하세요:")
    print("  [1] Full Range Multi-Step: 25 -> 35 -> 40 -> 20 -> 10 -> 25°C (각 60~75초, 총 6.5분)")
    print("  [2] Extreme Jump Test:     25 -> 40 -> 10 -> 25°C (각 45~100초, 총 4.9분)")
    print("  [3] Precision Small Steps: 25 -> 28 -> 22 -> 26 -> 25°C (각 45초, 총 3.75분)")
    print("  [4] 단일 온도 유지 (Single Step Test)")
    print("  [5] 직접 입력 (Custom Sequence e.g. 25:60, 35:60, 10:80)")

    choice = input("\n선택 [1]: ").strip() or "1"

    if choice in PRESET_PROFILES:
        profile = PRESET_PROFILES[choice]
        steps = profile["steps"]
        print(f"\n선택된 프로파일: {profile['name']}")
    elif choice == "4":
        raw_target = input("목표 온도 (°C) [25.0]: ").strip()
        target_temp = float(raw_target) if raw_target else 25.0
        raw_dur = input("테스트 지속 시간(초) [120]: ").strip()
        dur_sec = float(raw_dur) if raw_dur else 120.0
        steps = [(target_temp, dur_sec)]
    elif choice == "5":
        raw_seq = input("시퀀스 입력 (형식: 온도:초, 온도:초): ").strip()
        try:
            steps = parse_custom_profile(raw_seq)
        except Exception as e:
            print(f"입력 오류: {e}")
            peltier.close()
            return
    else:
        print("잘못된 선택입니다. 기본 프로파일 1번으로 진행합니다.")
        steps = PRESET_PROFILES["1"]["steps"]

    total_duration = sum(s[1] for s in steps)
    print(f"\n총 {len(steps)}단계 스텝, 총 소요 시간: {total_duration:.0f}초 ({total_duration/60:.1f}분)")
    for idx, (temp, dur) in enumerate(steps, 1):
        print(f"  Step {idx}: 목표 {temp:4.1f}°C ({dur:3.0f}초)")

    input("\n[Enter]를 누르면 온도 제어 테스트를 시작합니다...")

    peltier.start_control()

    all_records = []
    step_summaries = []

    test_start_time = time.time()

    with open(csv_filepath, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow([
            "time(s)",
            "step_idx",
            "target_temp",
            "sensor_temp",
            "t_pred",
            "rate_c_per_s",
            "u",
            "pwm",
            "heat_duty",
            "cool_duty",
            "d_term",
            "i_term",
            "adc",
            "ohm",
        ])

        try:
            prev_target = None
            for step_idx, (target_temp, step_dur) in enumerate(steps, 1):
                peltier.set_target_temperature(target_temp)
                step_start_time = time.time()
                step_end_time = step_start_time + step_dur
                step_records = []

                print("\n" + "-" * 80)
                print(f"[Step {step_idx}/{len(steps)}] 목표 온도: {target_temp}°C (지속 {step_dur:.0f}초)")
                print("-" * 80)
                print(f"{'Time(s)':>8} | {'Step':>4} | {'Target':>6} | {'Sensor':>6} | {'T_pred':>6} | {'u(%)':>6} | {'PWM':>6} | {'Rate(°C/s)':>10} | {'i_term':>6}")
                print("-" * 80)

                while time.time() < step_end_time:
                    curr_time = time.time()
                    elapsed_total = curr_time - test_start_time
                    ctrl = peltier.get_ctrl()

                    if ctrl is not None:
                        record = {
                            "time": round(elapsed_total, 2),
                            "step": step_idx,
                            "target_temp": ctrl["target_temp"],
                            "sensor_temp": ctrl["sensor_temp"],
                            "t_pred": ctrl["t_pred"],
                            "rate": ctrl["rate"],
                            "u": ctrl["u"],
                            "pwm": ctrl["pwm"],
                            "heat_duty": ctrl["heat_duty"],
                            "cool_duty": ctrl["cool_duty"],
                            "d_term": ctrl.get("f_hat", 0.0),  # d_term from arduino
                            "i_term": ctrl["i_term"],
                            "adc": ctrl["adc"],
                            "ohm": ctrl["ohm"],
                        }
                        step_records.append(record)
                        all_records.append(record)

                        wr.writerow([
                            record["time"],
                            record["step"],
                            record["target_temp"],
                            record["sensor_temp"],
                            record["t_pred"],
                            record["rate"],
                            record["u"],
                            record["pwm"],
                            record["heat_duty"],
                            record["cool_duty"],
                            record["d_term"],
                            record["i_term"],
                            record["adc"],
                            record["ohm"],
                        ])
                        f.flush()

                        print(
                            f"{record['time']:8.1f} | {step_idx:4d} | {ctrl['target_temp']:6.1f} | "
                            f"{ctrl['sensor_temp']:6.2f} | {ctrl['t_pred']:6.2f} | {ctrl['u']*100:6.1f} | "
                            f"{int(ctrl['pwm']):6d} | {ctrl['rate']:10.3f} | {ctrl['i_term']:6.2f}"
                        )

                    time.sleep(0.5)

                # Analyze completed step
                summary = analyze_step_performance(step_records, target_temp, prev_target)
                summary["step_idx"] = step_idx
                step_summaries.append(summary)
                prev_target = target_temp

                settle_str = f"{summary['settling_time']:.1f}s" if summary.get("settling_time") is not None else "미도달"
                print(f">> Step {step_idx} 완료: 도달시간={settle_str}, 오버슛={summary['overshoot']:.2f}°C, "
                      f"안정기 온도={summary['ss_mean']:.2f}±{summary['ss_std']:.2f}°C (오차={summary['ss_error']:+.2f}°C)")

        except KeyboardInterrupt:
            print("\n\n[사용자 중단 (Ctrl+C)] 현재까지의 데이터를 저장하고 종료합니다.")
        finally:
            peltier.stop_control()
            peltier.close()

    # Final 종합 보고서 출력
    print("\n" + "=" * 80)
    print("                       온도 제어 성능 종합 분석 보고서")
    print("=" * 80)
    print(f"{'Step':<5} | {'Target':<7} | {'Start':<6} | {'도달시간':<8} | {'오버슛':<8} | {'최대속도':<10} | {'안정상태 온도 (평균±표준편차)':<25}")
    print("-" * 80)

    for s in step_summaries:
        settle_str = f"{s['settling_time']:.1f}s" if s.get("settling_time") is not None else ">최대시간"
        print(f"Step {s['step_idx']:<2} | {s['target']:5.1f}°C | {s['start_temp']:5.1f} | "
              f"{settle_str:<8} | {s['overshoot']:5.2f}°C  | {s['max_rate']:6.3f}°C/s | "
              f"{s['ss_mean']:5.2f} ± {s['ss_std']:.2f}°C (오차: {s['ss_error']:+.2f}°C)")

    print("=" * 80)
    print(f"CSV 전체 로그 저장 완료: {csv_filepath}")
    print("서버 전송 명령어: python3 migrate_to_server.py\n")


if __name__ == "__main__":
    main()
