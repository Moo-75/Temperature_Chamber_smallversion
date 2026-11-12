#!/usr/bin/env python3
"""Heat and cool at several duties, scoring the sensor after its lag.

The plate takes about 5 s to start moving, so each direction is held
longer than that and the verdict uses temperature after the lag, not the
first sample. FORCE_PWM expires after 4 s and is refreshed during the hold.

Stop maintemp.py / the task first. Serial is exclusive.

    python3 diagnose_hiccup.py
    python3 diagnose_hiccup.py --hold 20 --lag 5
    python3 diagnose_hiccup.py --duties 0.15,0.30,0.50,1
"""

from __future__ import annotations

import argparse
import sys
import time

from diagnose_driver import (
    PWM_COMMANDED_TICKS,
    SerialException,
    cmd,
    find_serial_port,
    open_serial,
    parse_ctrl,
    write_line,
)

DEFAULT_DUTIES = (0.15, 0.30, 0.50, 1.00)
THERMAL_MOVE_C = 0.15
ABORT_HI_C = 42.0
ABORT_LO_C = 8.0
REST_SEC = 3.0


def read_state(ser):
    return parse_ctrl(cmd(ser, "GET_CTRL"))


def parse_duties(text):
    duties = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        value = float(part)
        if value > 1.0:
            value = value / 100.0
        if not 0.05 <= value <= 1.0:
            raise argparse.ArgumentTypeError(f"듀티는 0.05~1 입니다: {part}")
        duties.append(value)
    if not duties:
        raise argparse.ArgumentTypeError("듀티가 비어 있습니다.")
    return duties


def temp_near(samples, target):
    finite = [(elapsed, state["t"]) for elapsed, state in samples if state and state["t_finite"]]
    if not finite:
        return None
    _elapsed, temp = min(finite, key=lambda item: abs(item[0] - target))
    return temp


def hold_direction(ser, duty, seconds):
    samples = []
    aborted = False
    start = time.time()
    while True:
        elapsed = time.time() - start
        if elapsed >= seconds:
            break
        cmd(ser, f"FORCE_PWM,{duty:.2f}", expect_reply=False)
        if not samples:
            time.sleep(0.25)
        state = read_state(ser)
        samples.append((time.time() - start, state))
        if state and state["t_finite"]:
            t_txt = f"{state['t']:7.3f}"
        else:
            t_txt = "      ?"
        pwm = state["pwm"] if state else 0
        flag = ""
        if state and abs(state["pwm"]) < PWM_COMMANDED_TICKS:
            flag = "  [PWM≈0]"
        print(f"  t={elapsed:5.1f}s  T={t_txt} °C  pwm={pwm:+5.0f}{flag}")
        if state and state["t_finite"] and (state["t"] >= ABORT_HI_C or state["t"] <= ABORT_LO_C):
            print(f"  온도 {state['t']:.2f}°C 가 안전 범위 밖입니다. 이 단계를 끊습니다.")
            aborted = True
            break
        spent = time.time() - start - elapsed
        remain = seconds - (time.time() - start)
        nap = min(remain, max(0.0, 2.0 - spent))
        if nap > 0:
            time.sleep(nap)
    cmd(ser, "STOP", expect_reply=False)
    return samples, aborted


def score_step(duty, direction, samples, lag, aborted):
    t0 = temp_near(samples, 0.0)
    t_lag = temp_near(samples, lag)
    past_lag = [state["t"] for elapsed, state in samples if state and state["t_finite"] and elapsed >= lag]
    t_end = past_lag[-1] if past_lag else None
    last_elapsed = samples[-1][0] if samples else 0.0
    reached = last_elapsed >= lag + 2.0 and t_lag is not None and t_end is not None
    dt = (t_end - t_lag) if reached else None
    if not reached:
        label = "중단" if aborted else "측정부족"
    elif abs(dt) < THERMAL_MOVE_C:
        label = "정체"
    elif (direction == "가열" and dt > 0) or (direction == "냉각" and dt < 0):
        label = "따라감"
    else:
        label = "반대"
    return {
        "duty": abs(duty),
        "direction": direction,
        "t0": t0,
        "t_lag": t_lag,
        "t_end": t_end,
        "dt": dt,
        "label": label,
    }


def fmt_t(value):
    return f"{value:7.3f}" if value is not None else "    n/a"


def fmt_dt(value):
    return f"{value:+7.3f}" if value is not None else "    n/a"


def rest(ser, seconds):
    cmd(ser, "STOP", expect_reply=False)
    print(f"  정지 {seconds:.0f}s")
    time.sleep(seconds)


def interpret(rows):
    print("\n" + "=" * 72)
    print("  판정")
    print("=" * 72)
    usable = [row for row in rows if row["dt"] is not None and row["label"] != "중단"]
    if not usable:
        print("  5초 이후 온도를 비교할 단계가 없습니다.")
        print("=" * 72)
        return

    followed = [row for row in usable if row["label"] == "따라감"]
    flipped = [row for row in usable if row["label"] == "반대"]
    flat = [row for row in usable if row["label"] == "정체"]

    if len(flat) == len(usable):
        print(f"  모든 단계에서 5초 이후 변화가 {THERMAL_MOVE_C:.2f}°C 미만입니다.")
        print("  센서 기준으로는 가열도 냉각도 충분히 전달되지 않았습니다.")
        print("  SMPS가 3초 주기로 깜빡이면 평균 전력이 작아서 이렇게 나옵니다.")
    elif followed and not flipped:
        print(f"  따라간 단계 {len(followed)}개, 정체 {len(flat)}개.")
        print("  센서가 붙은 면은 명령 방향과 같습니다.")
        heats = [row for row in followed if row["direction"] == "가열"]
        cools = [row for row in followed if row["direction"] == "냉각"]
        if heats and not cools:
            print("  가열만 센서에 나타났습니다. 냉각 단계는 평균 전력이 더 약합니다.")
        elif cools and not heats:
            print("  냉각만 센서에 나타났습니다. 가열 단계는 평균 전력이 더 약합니다.")
        by_duty = {}
        for row in followed:
            by_duty.setdefault(row["duty"], []).append(abs(row["dt"]))
        ordered = sorted(by_duty)
        if len(ordered) >= 2 and by_duty[ordered[-1]] and by_duty[ordered[0]]:
            if max(by_duty[ordered[-1]]) > max(by_duty[ordered[0]]) + 0.05:
                print("  듀티가 클수록 5초 이후 변화도 큽니다. 평균 전력은 PWM을 따라갑니다.")
    elif flipped and not followed:
        print("  센서 변화가 명령과 반대입니다.")
        print("  센서가 붙은 면 기준으로 가열/냉각 극성이 바뀌어 있습니다.")
    else:
        print(f"  따라감 {len(followed)}개, 반대 {len(flipped)}개, 정체 {len(flat)}개.")
        print("  방향이 단계마다 섞여 있습니다. 위 표의 Δ(5초→끝) 부호를 보세요.")

    weak = [row for row in followed if abs(row["dt"]) < 0.5]
    if followed and len(weak) == len(followed):
        print("  따라간 변화도 0.5°C 미만입니다. 펠티어는 살아 있고, 전달 전력은 작습니다.")
    print("=" * 72)


def main():
    parser = argparse.ArgumentParser(description="Bidirectional PWM vs sensor-lag thermal probe")
    parser.add_argument("--json", default="test.json")
    parser.add_argument("--duties", default="0.15,0.30,0.50,1", help="쉼표로 구분. 0~1 또는 퍼센트")
    parser.add_argument("--hold", type=float, default=20.0, help="방향마다 유지 시간(초). 지연보다 길어야 합니다.")
    parser.add_argument("--lag", type=float, default=5.0, help="판정에서 빼는 초기 지연(초)")
    args = parser.parse_args()
    try:
        duties = parse_duties(args.duties)
    except (ValueError, argparse.ArgumentTypeError) as exc:
        print(f"[오류] {exc}")
        return 2
    if args.hold <= args.lag + 5.0:
        print(f"[오류] --hold 는 --lag+5초보다 길어야 합니다. 지금 hold={args.hold:.0f} lag={args.lag:.0f}")
        return 2

    print("=" * 72)
    print("  PWM별 가열/냉각 — 센서 지연 반영")
    print("=" * 72)
    print("  maintemp.py 와 task 를 먼저 종료하세요.")
    print(f"  듀티 {', '.join(f'{d:.0%}' for d in duties)}")
    runtime = len(duties) * 2 * (args.hold + REST_SEC)
    print(f"  각 방향 {args.hold:.0f}초 유지. 처음 {args.lag:.0f}초는 지연으로 보고, 그 이후 온도 변화로 판정합니다.")
    print(f"  예상 시간 약 {runtime / 60:.0f}분. 2초마다 센서 온도를 출력합니다.")
    print("  순서는 듀티마다 가열 다음 냉각. 단계 사이 3초 정지.")
    print("  드라이버가 견디기 힘들면 Ctrl+C. 즉시 STOP 합니다.")
    print()

    candidates, baudrate = find_serial_port(args.json)
    ser = None
    for port in candidates:
        try:
            print(f"연결 {port} ...", end=" ", flush=True)
            ser, banners = open_serial(port, baudrate)
            print("OK")
            if banners:
                print(f"  부팅: {banners[-1]!r}")
            break
        except SerialException as e:
            print(f"실패 ({e})")
            ser = None
    if ser is None:
        print("[실패] 시리얼을 열 수 없습니다.")
        return 2

    rows = []
    steps = [(duty, sign, name) for duty in duties for sign, name in ((1.0, "가열"), (-1.0, "냉각"))]
    try:
        cmd(ser, "STOP", expect_reply=False)
        time.sleep(0.3)
        idle = read_state(ser)
        if idle and idle["t_finite"]:
            print(f"시작 온도 {idle['t']:.3f} °C  adc={idle['adc']:.0f}")
        else:
            print("시작 온도를 읽지 못했습니다. 센서 응답을 확인하세요.")
        if sys.stdin.isatty():
            input("Enter 를 누르면 시작합니다. ")

        for index, (duty, sign, name) in enumerate(steps, start=1):
            print(f"\n--- [{index}/{len(steps)}] {name} {duty:.0%}  {args.hold:.0f}s ---")
            samples, aborted = hold_direction(ser, sign * duty, args.hold)
            row = score_step(duty, name, samples, args.lag, aborted)
            rows.append(row)
            print(
                f"  {name} {duty:.0%}  "
                f"T0={fmt_t(row['t0'])}  T{args.lag:.0f}s={fmt_t(row['t_lag'])}  "
                f"끝={fmt_t(row['t_end'])}  Δ(지연 이후)={fmt_dt(row['dt'])}  {row['label']}"
            )
            if index != len(steps):
                rest(ser, REST_SEC)

        print("\n" + "=" * 72)
        print("  기록  (Δ는 지연 이후 ~ 끝)")
        print("=" * 72)
        print(f"  {'duty':>6}  {'방향':>4}  {'T0':>8}  {'T지연':>8}  {'Tend':>8}  {'Δ':>8}  판정")
        for row in rows:
            print(
                f"  {row['duty']:6.0%}  {row['direction']:>4}  "
                f"{fmt_t(row['t0'])}  {fmt_t(row['t_lag'])}  {fmt_t(row['t_end'])}  "
                f"{fmt_dt(row['dt'])}  {row['label']}"
            )
        interpret(rows)
        return 0
    finally:
        try:
            write_line(ser, "STOP")
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        print("\n종료 (STOP).")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n중단됨.")
        sys.exit(130)
