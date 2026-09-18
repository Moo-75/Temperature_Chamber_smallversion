#!/usr/bin/env python3
"""Software-side Peltier / BTS7960 PWM diagnostic.

Does not need a scope. Talks to the Arduino over the same serial protocol as
maze.Peltier_module and reports whether firmware is actually commanding PWM.

Usage on the Pi (stop maintemp.py / task first — serial is exclusive):
    python3 diagnose_driver.py
    python3 diagnose_driver.py --json test.json
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
import time

try:
    import serial
    from serial import SerialException
except ImportError:
    print("[오류] pyserial 이 없습니다.  pip install pyserial")
    sys.exit(1)

PWM_TOP = 3124
DEADBAND_U = 0.02
PWM_COMMANDED_TICKS = 80  # ~2.5% of PWM_TOP; above deadband after slew
THERMAL_MOVE_C = 0.25
SAFE_T_MIN = 8.0
SAFE_T_MAX = 42.0


def find_serial_port(json_dir="test.json"):
    configured_port = "/dev/arduino"
    baudrate = 115200
    if os.path.exists(json_dir):
        try:
            with open(json_dir, "r") as f:
                cfg = json.load(f)
                configured_port = cfg.get("arduino", {}).get("port", configured_port)
                baudrate = int(cfg.get("arduino", {}).get("baudrate", baudrate))
        except Exception:
            pass

    candidates = []
    if os.path.exists(configured_port):
        candidates.append(configured_port)
    for p in glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*") + glob.glob("COM*"):
        if p not in candidates:
            candidates.append(p)
    return candidates, baudrate


def open_serial(port, baudrate):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baudrate
    ser.timeout = 1.0
    ser.dtr = False
    ser.rts = False
    ser.open()
    time.sleep(0.4)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    return ser


def write_line(ser, command):
    ser.write(f"{command}\n".encode("utf-8"))
    ser.flush()


def read_line(ser, timeout_sec=1.2):
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if ser.in_waiting:
            return ser.readline().decode("utf-8", errors="replace").strip()
        time.sleep(0.02)
    return ""


def cmd(ser, command, expect_reply=True, timeout_sec=1.2):
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    write_line(ser, command)
    if not expect_reply:
        return ""
    return read_line(ser, timeout_sec=timeout_sec)


def parse_ctrl(raw):
    if not raw:
        return None
    parts = [p.strip() for p in raw.split(",") if p.strip() != ""]
    if len(parts) < 10:
        return None
    try:
        vals = [float(p) for p in parts[:10]]
    except ValueError:
        return None
    t = vals[0]
    u = vals[3]
    pwm = vals[9]
    return {
        "raw": raw,
        "t": t,
        "t_finite": math.isfinite(t),
        "t_pred": vals[1],
        "rate": vals[2],
        "u": u,
        "d_term": vals[4],
        "i_term": vals[5],
        "target": vals[6],
        "adc": vals[7],
        "ohm": vals[8],
        "pwm": pwm,
        "heat_pct": max(0.0, u) * 100.0,
        "cool_pct": max(0.0, -u) * 100.0,
        "error": (vals[6] - t) if math.isfinite(t) else float("nan"),
    }


def parse_drive(raw):
    if not raw:
        return None
    parts = [p.strip() for p in raw.split(",") if p.strip() != ""]
    if len(parts) < 10:
        return None
    try:
        running = int(float(parts[0]))
        force = int(float(parts[1]))
        ocr_a = int(float(parts[2]))
        ocr_b = int(float(parts[3]))
        en_r = int(float(parts[4]))
        en_l = int(float(parts[5]))
        u = float(parts[6])
        pwm = int(float(parts[7]))
        target = float(parts[8])
        t = float(parts[9])
    except ValueError:
        return None
    return {
        "raw": raw,
        "running": running,
        "force": force,
        "ocr1a": ocr_a,
        "ocr1b": ocr_b,
        "en_r": en_r,
        "en_l": en_l,
        "u": u,
        "pwm": pwm,
        "target": target,
        "t": t,
    }


def fmt_ctrl(c):
    if c is None:
        return "  (응답 없음/파싱 실패)"
    t = f"{c['t']:.3f}" if c["t_finite"] else str(c["t"])
    return (
        f"  T={t:>8}  target={c['target']:6.2f}  err={c['error']:6.2f}  "
        f"u={c['u']:+6.3f}  pwm={c['pwm']:+5.0f}  "
        f"heat={c['heat_pct']:5.1f}%  cool={c['cool_pct']:5.1f}%  "
        f"adc={c['adc']:.0f}"
    )


def fmt_drive(d):
    if d is None:
        return "  (GET_DRIVE 없음 — 펌웨어 미업로드)"
    heat_pct = 100.0 * d["ocr1a"] / PWM_TOP
    cool_pct = 100.0 * d["ocr1b"] / PWM_TOP
    return (
        f"  run={d['running']} force={d['force']}  "
        f"OCR1A(D9 heat)={d['ocr1a']:4d} ({heat_pct:5.1f}%)  "
        f"OCR1B(D10 cool)={d['ocr1b']:4d} ({cool_pct:5.1f}%)  "
        f"R_EN={d['en_r']} L_EN={d['en_l']}  u={d['u']:+.3f}  pwm={d['pwm']:+d}"
    )


def poll_loop(ser, seconds, have_drive, label):
    rows = []
    t0 = time.time()
    print(f"\n--- {label} ({seconds:.0f}s) ---")
    while time.time() - t0 < seconds:
        ctrl = parse_ctrl(cmd(ser, "GET_CTRL"))
        drive = parse_drive(cmd(ser, "GET_DRIVE")) if have_drive else None
        elapsed = time.time() - t0
        print(f"  t={elapsed:5.1f}s{fmt_ctrl(ctrl)}")
        if have_drive:
            print(f"           {fmt_drive(drive)}")
        rows.append({"t": elapsed, "ctrl": ctrl, "drive": drive})
        time.sleep(0.45)
    return rows


def max_abs_pwm(rows):
    vals = []
    for r in rows:
        c = r.get("ctrl")
        if c is not None:
            vals.append(abs(c["pwm"]))
        d = r.get("drive")
        if d is not None:
            vals.append(abs(d["ocr1a"]))
            vals.append(abs(d["ocr1b"]))
    return max(vals) if vals else 0.0


def first_last_temp(rows):
    temps = [r["ctrl"]["t"] for r in rows if r.get("ctrl") and r["ctrl"]["t_finite"]]
    if len(temps) < 2:
        return None, None
    return temps[0], temps[-1]


def signed_pwm_peak(rows):
    signed = [r["ctrl"]["pwm"] for r in rows if r.get("ctrl")]
    if not signed:
        return 0.0
    return max(signed, key=lambda x: abs(x))


def main():
    parser = argparse.ArgumentParser(description="Peltier PWM / driver software diagnostic")
    parser.add_argument("--json", default="test.json")
    parser.add_argument("--delta", type=float, default=8.0, help="closed-loop setpoint offset (°C)")
    parser.add_argument("--hold", type=float, default=8.0, help="seconds per closed-loop step")
    parser.add_argument("--force-u", type=float, default=0.60, help="FORCE_PWM magnitude 0..1")
    parser.add_argument("--no-force", action="store_true", help="skip open-loop FORCE_PWM even if firmware supports it")
    args = parser.parse_args()

    print("=" * 72)
    print("  펠티어 / BTS7960 드라이버 소프트웨어 진단")
    print("=" * 72)
    print("  실험(maintemp.py / task)이 돌아가면 시리얼을 독점하므로 먼저 종료하세요.")
    print("  이 스크립트는 실제 가열/냉각을 몇 초간 겁니다. 플레이트가 뜨거워질 수 있습니다.")
    print()

    candidates, baudrate = find_serial_port(args.json)
    print(f"1) 시리얼 후보: {candidates}  baud={baudrate}")
    if not candidates:
        print("[실패] 포트를 못 찾았습니다. USB와 ./link_arduino.sh 를 확인하세요.")
        return 2

    ser = None
    port = None
    for p in candidates:
        try:
            print(f"   연결 시도: {p} ...", end=" ", flush=True)
            ser = open_serial(p, baudrate)
            port = p
            print("OK")
            break
        except SerialException as e:
            print(f"실패 ({e})")
            if "Permission" in str(e) or "Access" in str(e) or "busy" in str(e).lower():
                print("   → 다른 프로세스가 포트를 잡고 있을 수 있습니다 (maintemp.py).")
            ser = None
    if ser is None:
        print("[실패] 시리얼을 열 수 없습니다.")
        return 2

    findings = []
    have_drive = False
    try:
        print(f"\n2) 포트 {port} — 펌웨어 handshake")
        raw_ctrl = cmd(ser, "GET_CTRL")
        print(f"   GET_CTRL raw: {raw_ctrl!r}")
        ctrl0 = parse_ctrl(raw_ctrl)
        if ctrl0 is None:
            print("   [실패] GET_CTRL 응답이 없습니다.")
            print("   → peltier_operating_system.ino 가 안 올라가 있거나 baud가 다릅니다.")
            findings.append("serial_fail")
            return 2

        raw_drive = cmd(ser, "GET_DRIVE")
        drive0 = parse_drive(raw_drive)
        have_drive = drive0 is not None
        if have_drive:
            print(f"   GET_DRIVE raw: {raw_drive!r}")
            print(fmt_drive(drive0))
        else:
            print("   GET_DRIVE 미지원 (구 펌웨어). PWM은 GET_CTRL의 pwm 필드로만 봅니다.")
            print("   핀/타이머까지 보려면 최신 peltier_operating_system.ino 를 업로드하세요.")

        print("\n3) 센서 스냅샷")
        print(fmt_ctrl(ctrl0))
        if not ctrl0["t_finite"]:
            print("   [원인] 온도가 NaN 입니다. 펌웨어는 이 경우 PWM을 강제로 0으로 둡니다.")
            findings.append("sensor_nan")
        elif not (10.0 <= ctrl0["adc"] <= 1010.0):
            print("   [주의] ADC가 정상 범위를 벗어났습니다. diagnose_sensor.py 도 같이 보세요.")
            findings.append("adc_edge")
        else:
            print("   센서 ADC는 정상 범위입니다. 이번 문제는 드라이버/PWM 쪽을 의심합니다.")

        t_now = ctrl0["t"] if ctrl0["t_finite"] else 25.0
        heat_set = min(SAFE_T_MAX, t_now + abs(args.delta))
        cool_set = max(SAFE_T_MIN, t_now - abs(args.delta))
        if abs(heat_set - t_now) < 3.0 and abs(cool_set - t_now) < 3.0:
            print("   [주의] 현재 온도가 안전 한계에 붙어 목표 오프셋을 거의 못 줍니다.")

        print("\n4) STOP 확인 — PWM이 0으로 떨어지는지")
        cmd(ser, "STOP", expect_reply=False)
        time.sleep(0.25)
        stop_rows = poll_loop(ser, 1.4, have_drive, "STOP")
        stop_pwm = max_abs_pwm(stop_rows)
        if stop_pwm > PWM_COMMANDED_TICKS:
            print("   [이상] STOP 이후에도 PWM이 남아 있습니다.")
            findings.append("stop_stuck")
        else:
            print("   STOP 후 PWM=0. 펌웨어가 enable/타이머를 끄는 경로는 동작합니다.")

        print(f"\n5) 닫힌루프 가열  START + SET_TEMP,{heat_set:.1f}")
        cmd(ser, "START", expect_reply=False)
        cmd(ser, f"SET_TEMP,{heat_set:.2f}", expect_reply=False)
        time.sleep(0.15)
        echoed = cmd(ser, "GET_TARGET")
        print(f"   GET_TARGET={echoed!r}  (SET_TEMP가 먹혔는지)")
        try:
            echoed_t = float(echoed) if echoed else None
        except ValueError:
            echoed_t = None
        if echoed_t is None or abs(echoed_t - heat_set) > 0.2:
            print("   [원인] SET_TEMP가 target에 반영되지 않았습니다. 명령 파싱/다른 프로세스 간섭.")
            findings.append("set_temp_ignored")
        heat_rows = poll_loop(ser, args.hold, have_drive, f"HEAT → {heat_set:.1f}°C")
        heat_pwm = signed_pwm_peak(heat_rows)
        h0, h1 = first_last_temp(heat_rows)
        heat_cmd = abs(heat_pwm) >= PWM_COMMANDED_TICKS and heat_pwm > 0
        heat_move = (h0 is not None) and ((h1 - h0) >= THERMAL_MOVE_C)
        if heat_cmd:
            print(f"   소프트웨어 가열 PWM 명령됨 (peak pwm={heat_pwm:.0f}).")
        else:
            print(f"   [원인] 가열 PWM이 거의 안 나감 (peak pwm={heat_pwm:.0f}).")
            findings.append("heat_pwm_zero")
        if h0 is not None:
            print(f"   온도 {h0:.2f} → {h1:.2f} °C  (Δ={h1 - h0:+.2f})")
            if heat_cmd and not heat_move:
                findings.append("heat_no_thermal")

        print(f"\n6) 닫힌루프 냉각  SET_TEMP,{cool_set:.1f}")
        cmd(ser, f"SET_TEMP,{cool_set:.2f}", expect_reply=False)
        cool_rows = poll_loop(ser, args.hold, have_drive, f"COOL → {cool_set:.1f}°C")
        cool_pwm = signed_pwm_peak(cool_rows)
        c0, c1 = first_last_temp(cool_rows)
        cool_cmd = abs(cool_pwm) >= PWM_COMMANDED_TICKS and cool_pwm < 0
        cool_move = (c0 is not None) and ((c0 - c1) >= THERMAL_MOVE_C)
        if cool_cmd:
            print(f"   소프트웨어 냉각 PWM 명령됨 (peak pwm={cool_pwm:.0f}).")
        else:
            print(f"   [원인] 냉각 PWM이 거의 안 나감 (peak pwm={cool_pwm:.0f}).")
            findings.append("cool_pwm_zero")
        if c0 is not None:
            print(f"   온도 {c0:.2f} → {c1:.2f} °C  (Δ={c1 - c0:+.2f})")
            if cool_cmd and not cool_move:
                findings.append("cool_no_thermal")

        force_ok = False
        if have_drive and not args.no_force:
            fu = max(0.15, min(1.0, abs(args.force_u)))
            print(f"\n7) 개방루프 FORCE_PWM (PID 우회, {fu:.2f} / {-fu:.2f}, 각 3.2s, 자동 4s 타임아웃)")
            cmd(ser, f"FORCE_PWM,{fu:.2f}", expect_reply=False)
            time.sleep(0.15)
            fh = parse_drive(cmd(ser, "GET_DRIVE"))
            print("   HEAT force " + fmt_drive(fh))
            if fh and fh["force"] == 1 and fh["ocr1a"] > PWM_COMMANDED_TICKS and fh["en_r"] == 1 and fh["en_l"] == 1:
                force_ok = True
                print("   Timer1 OCR1A(가열) + R_EN/L_EN=HIGH. 펌웨어가 핀을 실제로 켰습니다.")
            else:
                print("   [원인] FORCE_PWM 가열에서 OCR1A 또는 enable이 안 켜짐.")
                findings.append("force_heat_fail")
            time.sleep(3.0)

            cmd(ser, f"FORCE_PWM,{-fu:.2f}", expect_reply=False)
            time.sleep(0.15)
            fc = parse_drive(cmd(ser, "GET_DRIVE"))
            print("   COOL force " + fmt_drive(fc))
            if fc and fc["ocr1b"] > PWM_COMMANDED_TICKS and fc["en_r"] == 1 and fc["en_l"] == 1:
                print("   Timer1 OCR1B(냉각) + enable HIGH.")
            else:
                print("   [원인] FORCE_PWM 냉각에서 OCR1B 또는 enable이 안 켜짐.")
                findings.append("force_cool_fail")
            time.sleep(3.0)
        elif not have_drive:
            print("\n7) FORCE_PWM 생략 — 펌웨어에 GET_DRIVE/FORCE_PWM이 없습니다.")
            print("   Arduino IDE에서 최신 peltier_operating_system.ino 업로드 후 다시 실행하세요.")

        print("\n8) 정리 STOP")
        cmd(ser, "STOP", expect_reply=False)
        time.sleep(0.2)
        final = parse_ctrl(cmd(ser, "GET_CTRL"))
        print(fmt_ctrl(final))

        print("\n" + "=" * 72)
        print("  판정")
        print("=" * 72)
        pwm_any = heat_cmd or cool_cmd or force_ok
        thermal_any = heat_move or cool_move

        if "serial_fail" in findings:
            print("  시리얼/펌웨어 응답 실패.")
        elif "sensor_nan" in findings:
            print("  센서가 NaN → 펌웨어가 모터를 의도적으로 끕니다.")
            print("  diagnose_sensor.py 로 A0 배선을 먼저 고치세요.")
        elif "set_temp_ignored" in findings and not pwm_any:
            print("  SET_TEMP가 아두이노 target에 안 들어갑니다.")
            print("  다른 프로그램이 같은 포트를 쓰거나, 펌웨어가 구버전일 수 있습니다.")
        elif not pwm_any:
            print("  소프트웨어가 PWM을 명령하지 않습니다 (u≈0, pwm≈0).")
            print("  가능한 원인:")
            print("    - START가 안 먹음 / 곧이어 STOP 됨")
            print("    - 목표와 현재 온도 차가 너무 작음 (|u|<0.02 데드밴드)")
            print("    - 센서 NaN으로 매 사이클 stopMotor()")
            if have_drive:
                last_d = parse_drive(cmd(ser, "GET_DRIVE"))
                print(fmt_drive(last_d))
        elif pwm_any and not thermal_any:
            print("  소프트웨어 PWM은 나갑니다. 플레이트 온도는 안 변합니다.")
            print("  → 펌웨어/시리얼 문제가 아니라 파워단 하드웨어입니다.")
            print("    1) 펠티어 12V 어댑터가 켜져 있는지 (BTS7960 VM / B+)")
            print("    2) Arduino GND 와 12V PSU GND 가 공통인지")
            print("    3) BTS7960: RPWM←D9, LPWM←D10, R_EN←D7, L_EN←D8")
            print("    4) 드라이버 보드 LED / 발열 / 팬 유무")
            if have_drive:
                print("    GET_DRIVE 에서 OCR1A/OCR1B 와 R_EN/L_EN=1 이면")
                print("    아두이노 핀 출력은 정상, BTS7960 이후를 의심하세요.")
        else:
            print("  소프트웨어 PWM도 나가고, 온도도 목표 방향으로 움직입니다.")
            print("  제어 루프는 살아 있습니다. 체감이 없다면 목표-현재 차가 작거나")
            print("  실험 코드가 START 없이 SET_TEMP만 보내고 있는지 확인하세요.")

        if findings:
            print(f"\n  flags: {', '.join(findings)}")
        print("=" * 72)
        return 0 if pwm_any else 1
    finally:
        try:
            write_line(ser, "STOP")
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        print("\n진단 종료 (STOP 전송, 포트 닫음).")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n중단됨.")
        sys.exit(130)
