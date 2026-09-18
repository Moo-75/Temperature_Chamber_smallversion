#!/usr/bin/env bash
# Raspberry Pi CLI에서 peltier_operating_system.ino 를 Arduino Uno로 업로드한다.
#
#   ./upload_arduino.sh
#   ./upload_arduino.sh --kill          # 포트를 잡고 있는 프로세스 종료 후 업로드
#   ./upload_arduino.sh --port /dev/ttyACM0
#
# 최초 실행은 arduino-cli / AVR 코어 설치 때문에 인터넷이 필요하다.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
SKETCH="${ROOT}/peltier_operating_system"
FQBN="${ARDUINO_FQBN:-arduino:avr:uno}"
CLI_BIN_DIR="${HOME}/.local/bin"
PORT=""
KILL_BUSY=0

usage() {
  cat <<'EOF'
Usage: ./upload_arduino.sh [--kill] [--port DEVICE] [--help]

  --kill          시리얼을 쓰는 프로세스(maintemp.py 등)를 끊고 업로드
  --port DEVICE   기본: /dev/arduino, 없으면 /dev/ttyACM* 또는 /dev/ttyUSB*
  --help          이 도움말

환경변수:
  ARDUINO_FQBN    기본 arduino:avr:uno
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --kill) KILL_BUSY=1; shift ;;
    --port)
      PORT="${2:-}"
      if [ -z "${PORT}" ]; then
        echo "--port 뒤에 장치 경로가 필요합니다."
        exit 2
      fi
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "알 수 없는 옵션: $1"
      usage
      exit 2
      ;;
  esac
done

if [ ! -f "${SKETCH}/peltier_operating_system.ino" ]; then
  echo "스케치를 찾지 못했습니다: ${SKETCH}/peltier_operating_system.ino"
  exit 1
fi

pick_dev() {
  local d
  if [ -e /dev/arduino ]; then
    echo /dev/arduino
    return 0
  fi
  for d in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0 /dev/ttyUSB1; do
    if [ -e "$d" ]; then
      echo "$d"
      return 0
    fi
  done
  return 1
}

if [ -z "${PORT}" ]; then
  PORT="$(pick_dev || true)"
fi
if [ -z "${PORT}" ] || [ ! -e "${PORT}" ]; then
  echo "Arduino 시리얼 장치를 찾지 못했습니다."
  echo "  USB 연결 후: ls -l /dev/ttyACM* /dev/ttyUSB* /dev/arduino"
  echo "  ./link_arduino.sh 를 실행해도 됩니다."
  exit 1
fi

REAL_PORT="${PORT}"
if [ -L "${PORT}" ]; then
  REAL_PORT="$(readlink -f "${PORT}")"
fi

echo "================================================================"
echo " Arduino 업로드"
echo "  sketch : ${SKETCH}"
echo "  fqbn   : ${FQBN}"
echo "  port   : ${PORT}  (-> ${REAL_PORT})"
echo "================================================================"

if command -v fuser >/dev/null 2>&1 && fuser "${REAL_PORT}" >/dev/null 2>&1; then
  echo ""
  echo "포트가 다른 프로세스에 잡혀 있습니다:"
  fuser -v "${REAL_PORT}" 2>&1 || true
  if [ "${KILL_BUSY}" -eq 1 ]; then
    echo "--kill: 해당 프로세스를 종료합니다."
    fuser -k "${REAL_PORT}" >/dev/null 2>&1 || true
    sleep 1
  else
    echo "maintemp.py / 진단을 종료하거나,  ./upload_arduino.sh --kill  을 쓰세요."
    exit 1
  fi
fi

export PATH="${CLI_BIN_DIR}:${PATH}"

ensure_cli() {
  if command -v arduino-cli >/dev/null 2>&1; then
    return 0
  fi
  echo ""
  echo "[1] arduino-cli 설치 -> ${CLI_BIN_DIR}"
  mkdir -p "${CLI_BIN_DIR}"
  if command -v curl >/dev/null 2>&1; then
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
      | BINDIR="${CLI_BIN_DIR}" sh
  fi
  export PATH="${CLI_BIN_DIR}:${PATH}"
  if command -v arduino-cli >/dev/null 2>&1; then
    return 0
  fi
  echo "공식 설치 스크립트가 실패했습니다. apt로 시도합니다."
  sudo apt-get update
  sudo apt-get install -y arduino-cli curl
  if command -v arduino-cli >/dev/null 2>&1; then
    return 0
  fi
  echo "arduino-cli 를 설치하지 못했습니다."
  exit 1
}

ensure_cli
echo ""
echo "arduino-cli: $(command -v arduino-cli)"
arduino-cli version || true

if ! arduino-cli core list 2>/dev/null | grep -q '^arduino:avr'; then
  echo ""
  echo "[2] AVR 코어 설치 (최초 1회, 인터넷 필요)"
  arduino-cli core update-index
  arduino-cli core install arduino:avr
else
  echo "AVR 코어: 이미 설치됨"
fi

echo ""
echo "[3] 컴파일"
arduino-cli compile --fqbn "${FQBN}" "${SKETCH}"

echo ""
echo "[4] 업로드"
arduino-cli upload --fqbn "${FQBN}" -p "${REAL_PORT}" "${SKETCH}"

echo ""
echo "완료. 시리얼에서 'Arduino Ready. Command-based control.' 이 나와야 합니다."
echo "  python3 diagnose_wiring.py"
echo "================================================================"
