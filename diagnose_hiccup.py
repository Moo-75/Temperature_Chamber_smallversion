#!/usr/bin/env python3
"""Duty sweep: PSU hiccup vs BTS7960 heating vs plate temperature.

The Arduino cannot see the 12 V adapter LED or the driver heatsink.
FORCE_PWM steps the duty while this script logs plate temperature, then
asks what the LED and the heatsink did.

Why moderate duty is a separate question from "PWM too high":
Timer1 here is about 5 Hz, so the ON slice still puts full 12 V across the
Peltier. Duty reduces average current, not the peak. A supply that hiccups
on peak current blinks even at 8%. A driver that only overheats because the
PID sits at 100% stays tolerable around 30%, and the plate temperature moves.

FORCE_PWM expires after 4 s, so each step refreshes it.
Stop maintemp.py / the task first. Serial is exclusive.

    python3 diagnose_hiccup.py
    python3 diagnose_hiccup.py --json test.json
    python3 diagnose_hiccup.py --full
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

# (duty, seconds, why this step exists)
STEPS = (
    (0.08, 8.0, "낮은 듀티. 여기서 LED가 깜빡이면 피크 전류에서 이미 히컵입니다."),
    (0.15, 10.0, "낮은 쪽. 평균 전류는 100%의 약 1/6 입니다."),
    (0.30, 18.0, "적당한 PWM. 온도가 오르는지, 드라이버 발열이 덜한지 보는 구간입니다."),
    (0.50, 10.0, "중간 위. 30%는 괜찮은데 여기부터 뜨거우면 상한은 30% 근처입니다."),
    (1.00, 6.0, "100%. 포화 시 드라이버가 얼마나 빨리 뜨거워지는지 짧게만 봅니다."),
)

THERMAL_MOVE_C = 0.25
ABORT_TEMP_C = 42.0
LED_OK = ("on", "blink", "off")
HEAT_OK = ("cool", "warm", "hot")


def read_state(ser):
    return parse_ctrl(cmd(ser, "GET_CTRL"))


def ask(prompt, allowed):
    if not sys.stdin.isatty():
        return ""
    allowed_txt = "/".join(allowed)
    while True:
        try:
            answer = input(prompt).strip().lower()
        except EOFError:
            return ""
        if answer == "":
            return ""
        if answer in allowed:
            return answer
        print(f"  {allowed_txt} 또는 Enter")


def hold_duty(ser, duty, seconds):
    cmd(ser, f"FORCE_PWM,{duty:.2f}", expect_reply=False)
    time.sleep(0.2)
    first = read_state(ser)
    samples = [(0.0, first)]
    start = time.time()
    while True:
        elapsed = time.time() - start
        if elapsed >= seconds:
            break
        cmd(ser, f"FORCE_PWM,{duty:.2f}", expect_reply=False)
        state = read_state(ser)
        samples.append((elapsed, state))
        t_txt = f"{state['t']:.3f}" if state and state["t_finite"] else "?"
        pwm = state["pwm"] if state else 0
        left = seconds - elapsed
        flag = ""
        if state and abs(state["pwm"]) < PWM_COMMANDED_TICKS:
            flag = "  [PWM이 거의 0 — FORCE가 끊긴 것처럼 보임]"
        print(f"  t={elapsed:5.1f}s  T={t_txt}  pwm={pwm:+5.0f}  남은 {left:4.1f}s{flag}")
        if state and state["t_finite"] and state["t"] >= ABORT_TEMP_C:
            print(f"  플레이트 {state['t']:.1f}°C. 이 단계는 여기서 끊습니다.")
            break
        time.sleep(min(2.0, max(0.0, seconds - (time.time() - start))))
    cmd(ser, "STOP", expect_reply=False)
    return samples


def delta_t(samples):
    temps = []
    for _elapsed, state in samples:
        if state and state["t_finite"]:
            temps.append(state["t"])
    if len(temps) < 2:
        return None, None, None
    return temps[0], temps[-1], temps[-1] - temps[0]


def interpret(rows):
    print("\n" + "=" * 72)
    print("  판정")
    print("=" * 72)
    if not rows:
        print("  측정된 단계가 없습니다.")
        return

    moderate = next((r for r in rows if abs(r["duty"] - 0.30) < 0.01), None)
    low = next((r for r in rows if abs(r["duty"] - 0.08) < 0.01), None)
    full = next((r for r in rows if abs(r["duty"] - 1.00) < 0.01), None)

    if low and low["led"] == "blink":
        print("  8%에서도 어댑터 LED가 깜빡입니다.")
        print("  듀티를 낮춰도 켜진 순간의 전류는 100%와 같습니다 (PWM 약 5 Hz).")
        print("  적당한 PWM으로는 이 히컵이 사라지지 않습니다.")
        print("  어댑터 전류가 펠티어 피크보다 작거나, 출력 단락/브리지 관통입니다.")
    elif moderate and moderate["led"] == "blink":
        print("  낮은 듀티는 버텼는데 30%에서 히컵이 시작됐습니다.")
        print("  어댑터 평균 전류 한계입니다. 100% 포화는 이 전원으로 버티지 못합니다.")
        print("  상한은 히컵이 생기기 전 단계로 둬야 합니다.")
    elif moderate and moderate["led"] == "on":
        moved = moderate["dt"] is not None and moderate["dt"] >= THERMAL_MOVE_C
        if moved and moderate["heat"] in ("cool", "warm"):
            print("  30%에서 온도가 오르고, 드라이버 발열은 견딜 만합니다.")
            print("  100% 포화가 이 챔버 발열의 직접 원인으로 보입니다.")
            print("  다른 챔버는 목표 근처라 듀티가 낮게 머물고, 이 챔버는 오차가 크면 1초 안에 100%로 붙습니다.")
        elif moved and moderate["heat"] == "hot":
            print("  30%에서도 플레이트는 가열되는데 드라이버가 뜨겁습니다.")
            print("  듀티를 더 낮추거나, 펠티어 개수/방열을 줄여야 합니다. 30%는 아직 부담입니다.")
        elif moved and moderate["heat"] == "":
            print("  30%에서 온도는 목표 방향으로 움직였습니다.")
            print("  발열을 적지 않아서, 이 듀티에서 드라이버가 괜찮은지는 이 기록만으로 알 수 없습니다.")
        elif not moved and moderate["heat"] == "hot":
            print("  30%에서 온도는 거의 안 오르는데 드라이버는 뜨겁습니다.")
            print("  전류가 펠티어가 아니라 드라이버 안에서 열로 바뀌는 쪽에 가깝습니다.")
            print("  PWM 상한만으로는 해결되지 않습니다.")
        elif not moved and moderate["heat"] in ("cool", "warm", ""):
            print("  30%에서 LED는 켜져 있는데 온도가 거의 안 움직입니다.")
            if moderate["heat"] in ("cool", "warm"):
                print("  발열도 약하면 부하 전류가 작은 상태입니다. M+/M− 단선 또는 센서가 플레이트 밖입니다.")
            else:
                print("  발열 입력이 없습니다. M+/M− 전압을 이 단계에서 재면 전류 유무가 갈립니다.")
    elif moderate and moderate["led"] == "off":
        print("  30% 명령 중에도 어댑터 LED가 꺼져 있습니다.")
        print("  12 V가 부하로 나가지 않는 상태입니다. 듀티 문제가 아닙니다.")
    elif moderate and moderate["led"] == "":
        print("  LED 입력이 없습니다. 온도만으로 히컵과 단선을 구분할 수 없습니다.")
        if moderate["dt"] is not None and moderate["dt"] >= THERMAL_MOVE_C:
            print("  다만 30%에서 플레이트 온도는 올랐습니다. 그 구간에서는 전류가 펠티어까지 갔습니다.")
        elif moderate["dt"] is not None:
            print("  30%에서 플레이트 온도는 거의 그대로입니다.")

    if full and full["heat"] == "hot" and moderate and moderate["heat"] in ("cool", "warm"):
        print("  100%만 뜨겁고 30%는 그렇지 않습니다. 포화 듀티가 발열을 키운 경우와 맞습니다.")
    elif full and full["led"] == "blink" and moderate and moderate["led"] == "on":
        print("  100%에서만 LED가 깜빡입니다. 전원은 중간 듀티는 버티고 포화는 못 버팁니다.")

    print("=" * 72)


def main():
    parser = argparse.ArgumentParser(description="PSU hiccup / moderate PWM heating probe")
    parser.add_argument("--json", default="test.json")
    parser.add_argument(
        "--full",
        action="store_true",
        help="100% 단계를 묻지 않고 6초간 실행",
    )
    args = parser.parse_args()

    print("=" * 72)
    print("  12 V 히컵 / 적당한 PWM 발열 진단")
    print("=" * 72)
    print("  maintemp.py 와 task 를 먼저 종료하세요. 시리얼을 같이 못 씁니다.")
    print("  어댑터 LED와 BTS7960 방열판을 보면서 진행합니다.")
    print("  LED: on=켜진 채 유지, blink=몇 초 간격 깜빡임, off=꺼짐")
    print("  발열: cool / warm / hot. 방열판을 오래 잡지 마세요.")
    print("  가열 방향만 켭니다. 단계 사이에 STOP 합니다.")
    print("  Ctrl+C 면 즉시 STOP.")
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
    try:
        cmd(ser, "STOP", expect_reply=False)
        time.sleep(0.3)
        if sys.stdin.isatty():
            input("Enter 를 누르면 8%부터 시작합니다. ")

        for duty, seconds, why in STEPS:
            if abs(duty - 1.0) < 0.01 and not args.full:
                go = ask("100%를 6초만 켤까요? 이미 뜨거우면 n 또는 Enter [y/n]: ", ("y", "n"))
                if go != "y":
                    print("  100% 단계는 건너뜁니다.\n")
                    continue

            print(f"\n--- FORCE_PWM {duty:.0%}  {seconds:.0f}s ---")
            print(f"  {why}")
            samples = hold_duty(ser, duty, seconds)
            t0, t1, dt = delta_t(samples)
            if dt is None:
                print("  온도 응답 없음")
            else:
                print(f"  온도 {t0:.2f} → {t1:.2f} °C  (Δ={dt:+.2f})")
            led = ask("  이 단계 LED [on/blink/off] (Enter=모름): ", LED_OK)
            heat = ask("  이 단계 드라이버 발열 [cool/warm/hot] (Enter=모름): ", HEAT_OK)
            rows.append({"duty": duty, "dt": dt, "t0": t0, "t1": t1, "led": led, "heat": heat})
            print("  STOP. 2초 쉽니다.")
            time.sleep(2.0)

        print("\n" + "=" * 72)
        print("  기록")
        print("=" * 72)
        print(f"  {'duty':>6}  {'ΔT':>8}  {'LED':>6}  {'heat':>6}")
        for row in rows:
            dt_txt = f"{row['dt']:+.2f}" if row["dt"] is not None else "  n/a"
            print(f"  {row['duty']:6.0%}  {dt_txt:>8}  {row['led'] or '-':>6}  {row['heat'] or '-':>6}")
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
