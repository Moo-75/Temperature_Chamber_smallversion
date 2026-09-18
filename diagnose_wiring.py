#!/usr/bin/env python3
"""Probe BTS7960/IBT-2 logic wiring and optional current-sense pins.

PIN_PROBE uses no extra jumpers: it checks whether D7/D8/D9/D10 see the
module input pulldowns (signal wires actually attached).

GET_IS needs two Dupont wires:
    IBT-2 R_IS -> Arduino A1
    IBT-2 L_IS -> Arduino A2
Then FORCE_PWM can tell load-current vs BTS7960 fault (often missing 12 V).

Requires the latest peltier_operating_system.ino on the Uno.
Stop maintemp.py first.
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

PIN_LABELS = (
    ("D7 R_EN", "hold7", "pu7"),
    ("D8 L_EN", "hold8", "pu8"),
    ("D9 RPWM heat", "hold9", "pu9"),
    ("D10 LPWM cool", "hold10", "pu10"),
)


def parse_probe(raw):
    if not raw:
        return None
    parts = [p.strip() for p in raw.split(",") if p.strip() != ""]
    if len(parts) != 8:
        return None
    try:
        vals = [int(float(p)) for p in parts]
    except ValueError:
        return None
    keys = ["hold7", "pu7", "hold8", "pu8", "hold9", "pu9", "hold10", "pu10"]
    return dict(zip(keys, vals))


def parse_is(raw):
    if not raw:
        return None
    parts = [p.strip() for p in raw.split(",") if p.strip() != ""]
    if len(parts) < 2:
        return None
    try:
        return float(parts[0]), float(parts[1])
    except ValueError:
        return None


def sample_is(ser, n=5):
    rows = []
    for _ in range(n):
        parsed = parse_is(cmd(ser, "GET_IS"))
        if parsed is not None:
            rows.append(parsed)
        time.sleep(0.08)
    return rows


def is_floating(rows):
    if len(rows) < 3:
        return True
    a1 = [r[0] for r in rows]
    a2 = [r[1] for r in rows]
    span = max(max(a1) - min(a1), max(a2) - min(a2))
    mean = (sum(a1) + sum(a2)) / (2 * len(rows))
    return span > 80.0 and 40.0 < mean < 980.0


def classify_is(adc):
    if adc < 25:
        return "zero", "전류/고장 전류 없음"
    if adc < 700:
        return "load", "부하 전류로 보임 (드라이버+펠티어가 전류를 흘리는 중)"
    return "fault", "BTS7960 고장 플래그 (12V 없음/과온/단락 가능)"


def hold_output(ser, seconds, duty):
    print(f"\nFORCE_PWM {duty:+.2f} 를 {seconds:.0f}초 유지합니다.")
    print("  합선하지 마세요. 멀티미터로만 재세요.")
    print("  1) IBT-2 나사단자 B+ 와 B- 사이  → 약 12 V 여야 함 (FORCE와 무관)")
    print("  2) 굵은 출력선 M+ 와 M- 사이      → 가열이면 약 12 V, 꺼지면 0 V")
    print("  얇은 8핀 로직 선을 재면 안 됩니다.")
    deadline = time.time() + seconds
    while time.time() < deadline:
        cmd(ser, f"FORCE_PWM,{duty:.2f}", expect_reply=False)
        drive = parse_drive(cmd(ser, "GET_DRIVE"))
        left = max(0.0, deadline - time.time())
        print(f"  남은 {left:4.1f}s  {fmt_drive(drive)}")
        time.sleep(min(2.5, left) if left > 0 else 0)
    cmd(ser, "STOP", expect_reply=False)
    print("  STOP. 이제 M+/M- 는 다시 0 V 여야 합니다.")


def main():
    parser = argparse.ArgumentParser(description="BTS7960 wiring / driver probe")
    parser.add_argument(
        "--hold",
        type=float,
        default=0.0,
        metavar="SEC",
        help="프로브 후 가열 PWM을 SEC초 유지 (멀티미터용). 예: --hold 20",
    )
    args = parser.parse_args()

    print("=" * 72)
    print("  BTS7960 배선 / 드라이버 소프트웨어 프로브")
    print("=" * 72)
    print("  1) Arduino IDE에서 최신 peltier_operating_system.ino 를 업로드하세요.")
    print("  2) maintemp.py 는 종료하세요.")
    print("  3) (선택) IBT-2 R_IS→A1, L_IS→A2 듀폰을 꽂으면 전류까지 봅니다.")
    print()

    candidates, baudrate = find_serial_port()
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

    try:
        cmd(ser, "STOP", expect_reply=False)
        time.sleep(0.2)

        print("\n1) PIN_PROBE  (점퍼 없이 D7/D8/D9/D10 풀다운 검출)")
        raw = cmd(ser, "PIN_PROBE")
        print(f"   raw: {raw!r}")
        probe = parse_probe(raw)
        if probe is None:
            print("   PIN_PROBE 미지원. 최신 펌웨어를 업로드해야 합니다.")
            return 2

        attached = 0
        for label, hold_k, pu_k in PIN_LABELS:
            hold = probe[hold_k]
            pu = probe[pu_k]
            if hold == 0:
                attached += 1
                state = "연결됨 (모듈 입력 풀다운이 보임)"
            else:
                state = "떠 있음 (아두이노 핀이 IBT-2에 안 붙은 것처럼 보임)"
            print(f"   {label:16}  hold={hold} pu={pu}  → {state}")

        print("\n2) GET_DRIVE  (Timer1 / enable 레지스터)")
        cmd(ser, "FORCE_PWM,0.80", expect_reply=False)
        time.sleep(0.2)
        drive = parse_drive(cmd(ser, "GET_DRIVE"))
        print("  " + fmt_drive(drive))
        cmd(ser, "STOP", expect_reply=False)
        time.sleep(0.15)

        print("\n3) GET_IS  (A1=R_IS, A2=L_IS — 선이 없으면 떠 있는 ADC)")
        idle_rows = sample_is(ser)
        print(f"   idle samples: {idle_rows}")
        jumpered = not is_floating(idle_rows)

        heat_is = cool_is = None
        if jumpered:
            print("   IS 핀이 안정적으로 읽힘. 전류 프로브 진행.")
            cmd(ser, "FORCE_PWM,0.90", expect_reply=False)
            time.sleep(0.35)
            heat_is = parse_is(cmd(ser, "GET_IS"))
            print(f"   HEAT FORCE R_IS,L_IS = {heat_is}")
            cmd(ser, "FORCE_PWM,-0.90", expect_reply=False)
            time.sleep(0.35)
            cool_is = parse_is(cmd(ser, "GET_IS"))
            print(f"   COOL FORCE R_IS,L_IS = {cool_is}")
            cmd(ser, "STOP", expect_reply=False)
        else:
            print("   A1/A2가 떠 있습니다. R_IS/L_IS 듀폰을 안 꽂은 상태입니다.")
            print("   신호선 연결 여부만 PIN_PROBE로 판정합니다.")

        print("\n" + "=" * 72)
        print("  판정")
        print("=" * 72)
        if attached == 0:
            print("  네 핀 모두 떠 있음 → Arduino D7/D8/D9/D10 이 IBT-2 입력에 안 닿아 있습니다.")
            print("  눈으로 맞아 보여도 헤더 한 줄 어긋남, 반대쪽 커넥터, 접촉 불량이 흔합니다.")
            print("  RPWM/LPWM/R_EN/L_EN/GND 를 핀 번호로 다시 꽂으세요. VCC(5V)도 7번.")
        elif attached < 4:
            print(f"  {attached}/4 핀만 풀다운이 보입니다. 빠진 선이 있습니다.")
            print("  hold=1 인 핀의 듀폰을 다시 꽂으세요.")
        else:
            print("  네 신호선(D7/D8/D9/D10)은 IBT-2 입력 풀다운에 붙어 있습니다.")
            print("  Arduino ↔ 드라이버 로직 배선은 소프트웨어상 연결로 나옵니다.")
            if drive and (drive["ocr1a"] < 80 or drive["en_r"] != 1 or drive["en_l"] != 0):
                print("  다만 FORCE 중 OCR/EN이 비정상입니다. 펌웨어 타이머를 의심하세요.")
            elif not jumpered:
                print("  드라이버 고장 vs 12V/펠티어 단선은 이 상태로는 구분 못 합니다.")
                print("  구분하려면 IBT-2 헤더의 R_IS→A1, L_IS→A2 를 꽂고 이 스크립트를 다시 실행하세요.")
                print("  (IS는 전류/고장 플래그입니다. 신호선 RPWM과 다른 핀입니다.)")
            else:
                heat_kind = classify_is(heat_is[0]) if heat_is else ("?", "")
                cool_kind = classify_is(cool_is[1]) if cool_is else ("?", "")
                print(f"  가열 IS: {heat_kind[0]} — {heat_kind[1]}")
                print(f"  냉각 IS: {cool_kind[0]} — {cool_kind[1]}")
                kinds = {heat_kind[0], cool_kind[0]}
                if "load" in kinds:
                    print("  파워단이 전류를 흘리고 있습니다. 드라이버는 살아 있습니다.")
                    print("  온도가 안 바뀌면 펠티어가 센서와 다른 면이거나, 열전달이 안 되는 쪽입니다.")
                elif kinds <= {"fault"}:
                    print("  BTS7960이 고장 전류를 내고 있습니다.")
                    print("  가장 흔한 원인: B+ 12V 없음, B− GND 없음, 또는 칩 보호 동작.")
                elif kinds <= {"zero"}:
                    print("  로직 선은 연결됐는데 출력 전류가 0입니다.")
                    print("  → IBT-2 VCC(5V) 누락, 12V는 있으나 M+/M− 펠티어 단선,")
                    print("    또는 하프브릿지 FET 고장.")
        print("=" * 72)
        if args.hold > 0:
            hold_output(ser, args.hold, 0.90)
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
