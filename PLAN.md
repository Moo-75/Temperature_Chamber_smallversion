# Temperature Chamber small version — 하드웨어/소프트웨어 계획

날짜: 2026-08-25 (Arduino 온도/드라이버로 수정)

## 역할 분담

- **Arduino:** KY-013 1개 아날로그 읽기 + BTS7960 PWM (존 제어). USB 시리얼 `/dev/arduino`
- **Raspberry Pi:** poke 1개, LED, TTL in/out, TL1/TL2. 온도 제어 명령만 보냄 (`GET_TEMP`, `SET_TEMP`, `START`, `STOP`)

ADS1115는 쓰지 않는다. Pi GPIO는 디지털만 있으므로 아날로그 온도는 Arduino A0로 읽는다.

## Pi GPIO (BCM)

| 기능 | BCM | 물리핀 | 연결 |
|---|---|---|---|
| poke OUT | 17 | 11 | 센서 OUT (5 V면 분압) |
| LED | 27 | 13 | 220 Ω → LED → GND |
| TTL OUT | 23 | 16 | 330 Ω → BNC 중심 |
| TTL IN | 24 | 18 | BNC → 10 k / 20 k 분압 |

GPIO 14/15는 시리얼 콘솔용으로 비움. 펠티어 PWM은 Arduino 핀이다.

## Arduino 핀

| 기능 | 핀 |
|---|---|
| KY-013 S | A0 |
| KY-013 VCC / GND | 5V / GND (가운데 핀이 VCC) |
| BTS7960 RPWM (냉각) | D9 |
| BTS7960 LPWM (가열) | D10 |
| BTS7960 R_EN | D7 |
| BTS7960 L_EN | D8 |

펠티어 파워(B+/B−)는 **12 V PSU**. Arduino/Pi와 GND만 공통.

## BNC

- **OUT:** GPIO23 → 330 Ω → BNC 중심, 실드 → GND. 3.3 V TTL.
- **IN:** BNC 중심 → 10 kΩ → GPIO24, GPIO24 → 20 kΩ → GND. 5 V 직결 금지.

## 전원

Pi 5 V 3 A는 Pi + Arduino USB + 로직에 충분. 펠티어 4개는 12 V 전용.

## 소프트웨어

유지: TL1/TL2, poke 로그, 온도 CSV, `migrate_to_server.py`, LED on/off, TTL API.
Arduino 펌웨어: `peltier_operating_system.ino`. 링크: `link_arduino.sh`.
