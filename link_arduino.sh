#!/usr/bin/env bash
# Arduino USB 시리얼을 /dev/arduino 로 링크한다.
# 펌웨어는 ./upload_arduino.sh 로 올린다.
set -euo pipefail

pick_dev() {
  local d
  for d in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0 /dev/ttyUSB1; do
    if [ -e "$d" ]; then
      echo "$d"
      return 0
    fi
  done
  return 1
}

DEV="$(pick_dev || true)"
if [ -z "${DEV}" ]; then
  echo "Arduino tty 장치를 찾지 못했습니다. USB 연결을 확인하세요."
  echo "  ls -l /dev/ttyACM* /dev/ttyUSB*"
  exit 1
fi

sudo ln -sfn "${DEV}" /dev/arduino
sudo chmod 666 "${DEV}" || true
echo "linked ${DEV} -> /dev/arduino"
ls -l /dev/arduino
