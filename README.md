# Temperature_Chamber_smallversion

Small temperature chamber for TL1 / TL2.

- Raspberry Pi: poke, LED, TTL, task
- Arduino: KY-013 temperature + one BTS7960 motor driver
- No display, camera, or reward port

## Setup

```bash
cd ~/Desktop
git clone https://github.com/Moo-75/Temperature_Chamber_smallversion.git
cd Temperature_Chamber_smallversion
chmod +x setup_new_pi.sh link_arduino.sh upload_arduino.sh
./setup_new_pi.sh
sudo reboot
```

Then on the Pi, flash firmware over USB and link the serial port:

```bash
cd ~/Desktop/Temperature_Chamber_smallversion
./upload_arduino.sh
./link_arduino.sh
```

```bash
python3 test_GPIO.py
python3 maintemp.py
# json -> test.json
# task -> TL1 or TL2
```

Wiring: `PLAN.md` / `SETUP.md`
