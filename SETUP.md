# Temperature Chamber small version — Pi 세팅

- **하드웨어:** Raspberry Pi 4 + Arduino Uno (KY-013 + BTS7960)
- **OS:** Raspberry Pi OS (64-bit)
- **전원:** Pi는 공식 5 V 3 A. 펠티어는 별도 12 V.
- 배선: `PLAN.md`

| 담당 | 역할 | 코드 |
|---|---|---|
| Raspberry Pi | poke, LED, TTL, TL1/TL2, Arduino 시리얼 명령 | `maze.py`, `maintemp.py`, `task_temp.py` |
| Arduino | KY-013 온도 읽기, 모터드라이버 PWM | `peltier_operating_system.ino` |

프로토콜: `GET_TEMP` / `SET_TEMP,xx` / `START` / `STOP` → `/dev/arduino`

## Phase 0 — 이미지

Raspberry Pi Imager: 64-bit OS, 사용자 `pi`, SSH, Wi-Fi KR, 시간대 Asia/Seoul.
디스플레이는 필요 없다. Arduino는 USB로 Pi에 연결한다.

## Phase 1 — 셋업

```bash
cd ~/Desktop
git clone https://github.com/Moo-75/Temperature_Chamber_smallversion.git
cd Temperature_Chamber_smallversion
sed -i 's/\r$//' setup_new_pi.sh link_arduino.sh
chmod +x setup_new_pi.sh link_arduino.sh
./setup_new_pi.sh
sudo reboot
```

스크립트: classic RPi.GPIO, pyserial, gpio/dialout 그룹, `/dev/arduino` udev, ModemManager 끄기, 시리얼 콘솔, 타임존.

## Phase 2 — Arduino 펌웨어

PC의 Arduino IDE에서 `peltier_operating_system.ino`를 Uno에 업로드.

배선 (Arduino):

- KY-013: S→A0, 가운데→5V, −→GND
- BTS7960: RPWM→D9, LPWM→D10, R_EN→D7, L_EN→D8, 파워단은 12 V, GND 공통

재부팅 후 Pi에서:

```bash
cd ~/Desktop/Temperature_Chamber_smallversion
./link_arduino.sh
ls -l /dev/arduino
python3 test_GPIO.py
python3 maze.py
python3 maintemp.py
```

클론 Arduino는 VID/PID가 다를 수 있다. 그때는 `link_arduino.sh`가 `/dev/ttyACM*`를 `/dev/arduino`로 직접 연결한다.

## 트러블슈팅

| 증상 | 조치 |
|---|---|
| `/dev/arduino` 없음 | `./link_arduino.sh`, USB 재연결 |
| 온도 읽히고 제어 안 됨 | `peltier_worker`의 `start_control()` / Arduino `START` |
| `lgpio.error: 'GPIO busy'` | classic RPi.GPIO, rpi-lgpio 제거 |
| 온도가 반대로 움직임 | KY-013 S/− 핀 확인 |
| 시리얼 권한 | dialout 그룹 후 재부팅 |
