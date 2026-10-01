# Temperature_Chamber_smallversion

Small temperature chamber for the Temperature Lift tasks (TLS / TLSX / TL0 / TL1 / TL2).

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
# task -> TLS, TLSX, TL0, TL1 or TL2
```

Tasks (all trial-based, same TD format; parameters saved to `TD_*_config.json`).
Training order: TLS(B) → TLY → TLS → TLC → TLM. Floor 10 °C in every stage.

| Task | LED / timing | Poke | 30 s / window without poke | Start, range |
|---|---|---|---|---|
| `TLS` Shaping | on except 10 s after a poke | +5/6/7 °C | −1/2/3 °C, next window at once | 24 °C, 10–34 °C |
| `TLSB` | TLS, first 10 min = baseline (held at 24 °C, outcomes off) | | | |
| `TLY` Yoked | as TLS | no effect | no effect | replays the mouse's last completed TLS target schedule |
| `TLC` Cue | 25 s window, 10 s feedback | +5/6/7 °C | −1/2/3 °C | 24 °C, 10–34 °C |
| `TLM` Main | 20 s window, 10 s feedback | +2/4/6 °C | −0/2/4 °C | 24 °C, 10–38 °C |
| `TL0` / `TL1` / `TL2` | earlier stages | | | |

TLS–TLM mark a poke at once with a 0.5 s LED flash at 100 % (normal brightness is
`led_brightness` in test.json). Probe and yoked trials log Bump/OutcomeDelta = 0;
`ProbeStart` / `ProbeEnd` rows mark the probe.

Yoking: at the end of every session the commanded target schedule is saved to
`<yoke_dir>/<mouse>/yoke_<mouse>_<session>_<task>_<timestamp>.json`
(`yoke_dir` from test.json, default `../_yoke_schedules`, i.e. outside the session
folders, so `migrate_to_server.py` never deletes it). When `TLY` is chosen,
maintemp lists that mouse's completed TLS sessions (newest first) and asks which to
replay; it refuses to start if none exist. The source is recorded in the TLY config.

Name the session after the task (e.g. `d13_TLS`) so the TD `Task` column records it.

Wiring: `PLAN.md` / `SETUP.md`
