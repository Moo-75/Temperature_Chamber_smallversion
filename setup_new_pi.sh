#!/usr/bin/env bash
# Temperature Chamber small version - Raspberry Pi 4 setup
# Target : Raspberry Pi OS (64-bit)
# Repo   : https://github.com/Moo-75/Temperature_Chamber_smallversion.git
#
#   chmod +x setup_new_pi.sh
#   ./setup_new_pi.sh
#   sudo reboot
set -euo pipefail

REPO="https://github.com/Moo-75/Temperature_Chamber_smallversion.git"
CLONE_DIR="${HOME}/Desktop/Temperature_Chamber_smallversion"

SERVER_TARGET="user@10.140.5.118"
SERVER_PORT="6022"
SERVER_DEST="/data/Siheon_chamber_data"
SERVER_PASSWORD="Dsng1830"

echo "================================================================"
echo " Temperature Chamber small version 세팅 시작"
echo "================================================================"

echo ""
echo "[1/8] 라이브러리 설치 ..."
sudo apt update
sudo apt install -y git python3-dev python3-serial python3-pytz python3-numpy

sudo apt remove -y python3-rpi-lgpio 2>/dev/null || true
pip3 install RPi.GPIO --break-system-packages
echo "  - RPi.GPIO 동작 확인:"
python3 -c "import RPi.GPIO as G; G.setwarnings(False); G.setmode(G.BCM); \
G.setup(17,G.IN,pull_up_down=G.PUD_DOWN); \
print('    OK, BCM17 input =', G.input(17)); G.cleanup()" \
  || echo "  ! RPi.GPIO 초기화 실패 → SETUP.md 참고"

echo ""
echo "[2/8] 사용자(${USER}) 그룹 권한 추가 ..."
sudo usermod -aG gpio,dialout,video,i2c,spi,input "${USER}"

echo ""
echo "[3/8] Arduino udev 규칙(/dev/arduino) 설치 ..."
sudo tee /etc/udev/rules.d/99-arduino.rules >/dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="arduino", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF
sudo udevadm control --reload && sudo udevadm trigger

echo ""
echo "[4/8] ModemManager 비활성화 ..."
sudo systemctl disable --now ModemManager 2>/dev/null || true

echo ""
echo "[5/8] 시리얼 콘솔 / 시간대 / WiFi 국가 ..."
if sudo raspi-config nonint do_serial_hw 0 2>/dev/null && \
   sudo raspi-config nonint do_serial_cons 0 2>/dev/null ; then
    echo "  - 시리얼(하드웨어+콘솔) 활성화 완료"
else
    sudo raspi-config nonint do_serial 0 || true
fi
sudo raspi-config nonint do_change_timezone Asia/Seoul || true
sudo raspi-config nonint do_wifi_country KR || true

echo ""
echo "[6/8] 챔버 코드 받기 -> ${CLONE_DIR}"
mkdir -p "$(dirname "${CLONE_DIR}")"
if [ -d "${CLONE_DIR}/.git" ]; then
    git -C "${CLONE_DIR}" pull --ff-only || true
else
    git clone "${REPO}" "${CLONE_DIR}"
fi
chmod +x "${CLONE_DIR}/link_arduino.sh" 2>/dev/null || true
chmod +x "${CLONE_DIR}/setup_new_pi.sh" 2>/dev/null || true

echo ""
echo "[7/8] 데이터 이전(migrate_to_server) 무입력 세팅 ..."
sudo apt install -y sshpass
if grep -q 'CHAMBER_SERVER_PASSWORD' "${HOME}/.bashrc" 2>/dev/null; then
    echo "  - 서버 접속 환경변수 이미 있음"
else
    {
        echo ''
        echo '# Temperature Chamber small version: migrate_to_server.py'
        echo "export CHAMBER_SERVER_TARGET='${SERVER_TARGET}'"
        echo "export CHAMBER_SERVER_PORT='${SERVER_PORT}'"
        echo "export CHAMBER_SERVER_DEST='${SERVER_DEST}'"
        echo "export CHAMBER_SERVER_PASSWORD='${SERVER_PASSWORD}'"
    } >> "${HOME}/.bashrc"
    echo "  - 서버 접속 환경변수 추가됨 (재로그인 후 적용)"
fi

echo ""
echo "[8/8] 완료"
echo "================================================================"
echo " 재부팅:   sudo reboot"
echo " 재부팅 후:"
echo "   Arduino IDE로 peltier_operating_system.ino 업로드"
echo "   cd ${CLONE_DIR} && ./link_arduino.sh"
echo "   python3 test_GPIO.py"
echo "   python3 maintemp.py"
echo "================================================================"
