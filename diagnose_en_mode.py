#!/usr/bin/env python3
"""Compare IBT-2 both-enable vs one-enable using the plate sensor.

BOTH (firmware default, unchanged for other chambers):
    heat drives D9 and holds R_EN and L_EN high.
ONE (this test only):
    heat drives D9 with R_EN high and L_EN low. Cool is the mirror.
    Some clones brake when the idle EN pin is also high: the driver heats
    and the Peltier current collapses.

The script restores EN_MODE,BOTH before it exits.
The Uno must be flashed after this firmware change:

    ./upload_arduino.sh --kill
    python3 diagnose_en_mode.py
"""

from __future__ import annotations

import argparse
import sys
import time

from diagnose_driver import (
    SerialException,
    cmd,
    find_serial_port,
    fmt_drive,
    open_serial,
    parse_drive,
    write_line,
)
from diagnose_hiccup import (
    fmt_dt,
    fmt_t,
    hold_direction,
    parse_duties,
    rest,
    score_step,
)


def set_mode(ser, mode):
    reply = cmd(ser, f"EN_MODE,{mode}")
    print(f"  EN_MODE,{mode} -> {reply!r}")
    return reply == mode


def expected_enable(mode, direction):
    if mode == "BOTH":
        return 1, 1
    if direction == "가열":
        return 1, 0
    return 0, 1


def confirm_pins(ser, signed, mode, direction):
    cmd(ser, f"FORCE_PWM,{signed:.2f}", expect_reply=False)
    time.sleep(0.25)
    drive = parse_drive(cmd(ser, "GET_DRIVE"))
    print(fmt_drive(drive))
    if drive is None:
        print("  GET_DRIVE 응답이 없습니다.")
        return False
    want_r, want_l = expected_enable(mode, direction)
    ok = drive["en_r"] == want_r and drive["en_l"] == want_l
    if not ok:
        print(f"  enable 불일치. 기대 R_EN={want_r} L_EN={want_l}, 실제 R_EN={drive['en_r']} L_EN={drive['en_l']}")
    return ok


def interpret(rows):
    print("\n" + "=" * 72)
    print("  판정")
    print("=" * 72)
    usable = {}
    for mode in ("BOTH", "ONE"):
        usable[mode] = [row for row in rows if row["mode"] == mode and row["label"] in ("따라감", "반대", "정체")]
    if not usable["BOTH"] or not usable["ONE"]:
        print("  두 모드를 비교할 만큼 측정이 끝나지 않았습니다.")
        print("=" * 72)
        return

    def followed(mode):
        return [row for row in usable[mode] if row["label"] == "따라감"]

    def flat(mode):
        return [row for row in usable[mode] if row["label"] == "정체"]

    both_follow = followed("BOTH")
    one_follow = followed("ONE")
    if one_follow and not both_follow:
        print("  ONE에서만 센서가 명령을 따라갑니다.")
        print("  이 보드는 enable을 한쪽만 켜야 합니다. 둘 다 켜면 반대편이 브레이크가 됩니다.")
        print("  드라이버만 덥고 펠티어 전류는 거의 없는 증상과 맞습니다.")
        print("  기본값은 아직 BOTH 입니다. 잘 되는 챔버를 그대로 두기 위해서입니다.")
    elif both_follow and not one_follow:
        print("  BOTH에서만 센서가 명령을 따라갑니다.")
        print("  이 보드는 양쪽 enable이 필요합니다. 한쪽만 켜면 전류 귀로가 없습니다.")
    elif both_follow and one_follow:
        print("  두 방식 모두 센서가 따라갑니다.")
        print("  이 듀티에서는 enable 개수가 온도를 막고 있지 않습니다.")
    else:
        print("  두 방식 모두 5초 이후 센서 변화가 기준 미만입니다.")
        print("  enable 개수만으로는 이번 온도 기록이 갈리지 않습니다.")
        print("  ONE 동안 SMPS LED가 깜빡임을 멈췄는지를 같이 봐 주세요.")
    mismatched = [row for row in rows if not row["pins_ok"]]
    if mismatched:
        print("  일부 단계에서 enable 핀이 요청과 달랐습니다. 업로드된 펌웨어를 확인하세요.")
    print("=" * 72)


def main():
    parser = argparse.ArgumentParser(description="Compare IBT-2 EN_MODE BOTH vs ONE")
    parser.add_argument("--json", default="test.json")
    parser.add_argument("--duties", default="0.50")
    parser.add_argument("--hold", type=float, default=20.0)
    parser.add_argument("--lag", type=float, default=5.0)
    args = parser.parse_args()
    try:
        duties = parse_duties(args.duties)
    except (ValueError, argparse.ArgumentTypeError) as exc:
        print(f"[오류] {exc}")
        return 2
    if args.hold <= args.lag + 5.0:
        print(f"[오류] --hold 는 --lag+5초보다 길어야 합니다. 지금 hold={args.hold:.0f} lag={args.lag:.0f}")
        return 2

    steps = [
        (mode, duty, sign, name)
        for mode in ("BOTH", "ONE")
        for duty in duties
        for sign, name in ((1.0, "가열"), (-1.0, "냉각"))
    ]
    runtime = len(steps) * (args.hold + 3.0)

    print("=" * 72)
    print("  IBT-2 enable 한쪽 / 양쪽 비교")
    print("=" * 72)
    print("  maintemp.py 와 task 를 먼저 종료하세요.")
    print("  BOTH = R_EN과 L_EN을 둘 다 켬 (지금 기본값)")
    print("  ONE  = PWM이 나가는 쪽 enable만 켬")
    print(f"  듀티 {', '.join(f'{duty:.0%}' for duty in duties)}, 방향마다 {args.hold:.0f}초.")
    print(f"  판정은 처음 {args.lag:.0f}초를 뺀 센서 변화. 예상 약 {runtime / 60:.0f}분.")
    print("  각 단계에서 SMPS LED가 3초 깜빡임인지, 켜진 채인지를 봐 두세요.")
    print("  끝나면 enable 모드는 BOTH로 되돌립니다. Ctrl+C 도 같습니다.")
    print()

    candidates, baudrate = find_serial_port(args.json)
    ser = None
    for port in candidates:
        try:
            print(f"연결 {port} ...", end=" ", flush=True)
            ser, _banners = open_serial(port, baudrate)
            print("OK")
            break
        except SerialException as exc:
            print(f"실패 ({exc})")
            ser = None
    if ser is None:
        print("[실패] 시리얼을 열 수 없습니다.")
        return 2

    rows = []
    try:
        cmd(ser, "STOP", expect_reply=False)
        time.sleep(0.2)
        current = cmd(ser, "EN_MODE")
        if current not in ("BOTH", "ONE"):
            print(f"  EN_MODE 응답 {current!r}")
            print("  이 아두이노에는 EN_MODE가 없습니다. 펌웨어를 올린 뒤 다시 실행하세요.")
            print("    ./upload_arduino.sh --kill")
            return 2
        print(f"  현재 EN_MODE={current}. 실험 기본 동작은 BOTH로 유지됩니다.")
        if sys.stdin.isatty():
            input("Enter 를 누르면 BOTH부터 시작합니다. ")

        for index, (mode, duty, sign, name) in enumerate(steps, start=1):
            if index == 1 or steps[index - 2][0] != mode:
                print(f"\n======== {mode} ========")
                if not set_mode(ser, mode):
                    print("  모드 전환 실패.")
                    return 2
            print(f"\n--- [{index}/{len(steps)}] {mode} {name} {duty:.0%}  {args.hold:.0f}s ---")
            pins_ok = confirm_pins(ser, sign * duty, mode, name)
            samples, aborted = hold_direction(ser, sign * duty, args.hold)
            row = score_step(duty, name, samples, args.lag, aborted)
            row["mode"] = mode
            row["pins_ok"] = pins_ok
            rows.append(row)
            print(
                f"  {mode} {name} {duty:.0%}  "
                f"T0={fmt_t(row['t0'])}  T{args.lag:.0f}s={fmt_t(row['t_lag'])}  "
                f"끝={fmt_t(row['t_end'])}  Δ={fmt_dt(row['dt'])}  {row['label']}"
            )
            if index != len(steps):
                rest(ser, 3.0)

        print("\n" + "=" * 72)
        print("  기록  (Δ는 지연 이후 ~ 끝)")
        print("=" * 72)
        print(f"  {'mode':>4}  {'duty':>6}  {'방향':>4}  {'T0':>8}  {'T지연':>8}  {'Tend':>8}  {'Δ':>8}  판정")
        for row in rows:
            print(
                f"  {row['mode']:>4}  {row['duty']:6.0%}  {row['direction']:>4}  "
                f"{fmt_t(row['t0'])}  {fmt_t(row['t_lag'])}  {fmt_t(row['t_end'])}  "
                f"{fmt_dt(row['dt'])}  {row['label']}"
            )
        interpret(rows)
        return 0
    finally:
        try:
            write_line(ser, "EN_MODE,BOTH")
        except Exception:
            pass
        try:
            write_line(ser, "STOP")
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        print("\n종료 (EN_MODE,BOTH / STOP).")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n중단됨.")
        sys.exit(130)
