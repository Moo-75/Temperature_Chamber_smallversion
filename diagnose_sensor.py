import glob
import math
import time
import json
import os
import sys

try:
    import serial
except ImportError:
    print("[오류] pyserial 모듈이 설치되어 있지 않습니다. 'pip install pyserial'을 실행하세요.")
    sys.exit(1)

def find_serial_port(json_dir="test.json"):
    # 1. config 파일에서 포트 확인
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

    # 2. 포트 존재 여부 및 후보 탐색
    candidates = []
    if os.path.exists(configured_port):
        candidates.append(configured_port)
    for p in glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*") + glob.glob("COM*"):
        if p not in candidates:
            candidates.append(p)

    return candidates, baudrate

def main():
    print("=" * 65)
    print("      [온도 챔버 센서 & 아두이노 정밀 진단 도구]")
    print("=" * 65)

    candidates, baudrate = find_serial_port()
    print(f"1. 감지된 시리얼 포트 후보: {candidates}")
    if not candidates:
        print("\n[오류] 아두이노 시리얼 포트를 찾을 수 없습니다!")
        print("  - USB 케이블이 제대로 꽂혀 있는지 확인하세요.")
        print("  - 라즈베리파이에서 'ls -l /dev/ttyACM* /dev/ttyUSB*' 를 확인하세요.")
        print("  - './link_arduino.sh' 를 실행하여 /dev/arduino 링크를 생성했는지 확인하세요.")
        return

    active_ser = None
    target_port = None
    for port in candidates:
        try:
            print(f"   포트 시도 중: {port} ({baudrate} bps)...", end=" ")
            ser = serial.Serial(port, baudrate=baudrate, timeout=1.5)
            time.sleep(2.0)  # 아두이노 DTR 리셋 대기
            ser.reset_input_buffer()
            # 초기 메시지 확인
            line = ser.readline().decode("utf-8", errors="replace").strip()
            print(f"연결 성공! (부팅 메시지: '{line}')")
            active_ser = ser
            target_port = port
            break
        except Exception as e:
            print(f"실패 ({e})")

    if active_ser is None:
        print("\n[오류] 아두이노와 시리얼 통신을 열 수 없습니다.")
        return

    print(f"\n2. 포트 '{target_port}' 연결 완료. 센서 원시 데이터 조회 시작...\n")

    try:
        # GET_CTRL 명령 전송
        active_ser.write(b"GET_CTRL\n")
        active_ser.flush()
        time.sleep(0.1)
        resp_ctrl = active_ser.readline().decode("utf-8", errors="replace").strip()
        print(f"-> [아두이노 원시 응답 - GET_CTRL]:\n   \"{resp_ctrl}\"")

        # GET_TEMP 명령 전송
        active_ser.write(b"GET_TEMP\n")
        active_ser.flush()
        time.sleep(0.1)
        resp_temp = active_ser.readline().decode("utf-8", errors="replace").strip()
        print(f"-> [아두이노 원시 응답 - GET_TEMP]:\n   \"{resp_temp}\"\n")

        print("-" * 65)
        print("                    [진단 결과 분석]")
        print("-" * 65)

        if not resp_ctrl:
            print("[진단 결과: 시리얼 응답 없음]")
            print("  - 아두이노가 명령에 응답하지 않습니다.")
            print("  - 원인: 아두이노에 최신 펌웨어(peltier_operating_system.ino)가 업로드되지 않았거나,")
            print("         보레이트가 115200이 아닌 다른 코드가 실행 중일 수 있습니다.")
            print("  - 조치: Arduino IDE에서 peltier_operating_system.ino를 다시 업로드하세요.")
            return

        parts = [p.strip() for p in resp_ctrl.split(",")]
        # 포맷: t, t_pred, rate, u, d_term, i_term, target, adc, ohm, pwm
        if len(parts) >= 10:
            t_str = parts[0]
            adc_str = parts[7]
            ohm_str = parts[8]
            print(f"  * 측정한 온도(t):        {t_str} °C")
            print(f"  * 측정한 ADC 원시값:     {adc_str} (전체 0 ~ 1023 범위 중)")
            print(f"  * 계산된 서미스터 저항:  {ohm_str} Ω")

            try:
                adc_val = float(adc_str)
            except ValueError:
                adc_val = None

            print("\n[원인 판정]:")
            if adc_val is not None:
                if adc_val < 1.0:
                    print("  ★ ADC 값이 0.0 근처 (0V) 입니다!")
                    print("    -> A0 핀의 전압이 0V로 완전히 떨어져 있습니다.")
                    print("    -> 원인 1: 아날로그 신호선(A0)이 GND로 쇼트(합선)되었음.")
                    print("    -> 원인 2: 서미스터 모듈의 VCC(5V) 선이 단선되어 전원이 안 들어감.")
                    print("    -> 원인 3: 모듈의 서미스터 소자가 단선(Open)되어 저항 무한대 상태임.")
                elif adc_val > 1022.0:
                    print("  ★ ADC 값이 1023.0 근처 (5V) 입니다!")
                    print("    -> A0 핀의 전압이 5V로 완전히 붙어 있습니다.")
                    print("    -> 원인 1: 아날로그 신호선(A0)이 5V 선에 쇼트(합선)되었음.")
                    print("    -> 원인 2: 서미스터 모듈의 GND(-) 선이 단선(접촉 불량)되어 풀업만 걸림.")
                    print("    -> 원인 3: 서미스터 모듈의 핀 배열이 다른 2대 챔버와 다름 (GND와 Signal 바뀜 등).")
                elif 10.0 <= adc_val <= 1010.0:
                    print(f"  ★ ADC 값이 {adc_val:.1f}로 정상 전압 범위 내에 있습니다.")
                    print("    -> 하드웨어 연결 및 전압 분압은 정상입니다.")
                    if t_str.lower() in ("nan", "ovf"):
                        print("    -> 저항 계산식이나 스케인하트 수식에서 범위를 벗어났습니다.")
                else:
                    print(f"  ★ 경계값 ADC: {adc_val}")
            else:
                print("  ★ ADC 값을 숫자로 파싱할 수 없습니다.")
        else:
            print(f"[진단 결과: 비정상 데이터 포맷] 응답 항목 개수={len(parts)}")
            print("  -> 최신 펌웨어(peltier_operating_system.ino)를 다시 업로드해야 합니다.")

        print("-" * 65)
        print("\n3. 실시간 ADC 및 온도 모니터링 (3초간 0.5초 간격 측정):")
        for i in range(6):
            active_ser.write(b"GET_CTRL\n")
            active_ser.flush()
            time.sleep(0.5)
            r = active_ser.readline().decode("utf-8", errors="replace").strip()
            pts = [x.strip() for x in r.split(",")]
            if len(pts) >= 10:
                print(f"  [{i+1}/6] Temp={pts[0]:>6} °C | ADC={pts[7]:>6} | Ohm={pts[8]:>8} Ω | PWM={pts[9]:>5}")
            else:
                print(f"  [{i+1}/6] Raw: {r}")

    finally:
        active_ser.close()
        print("\n진단 종료.")

if __name__ == "__main__":
    main()
