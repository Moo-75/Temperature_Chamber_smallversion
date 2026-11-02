# Temperature_Chamber_smallversion

Small temperature chamber for the Temperature Lift tasks (TLS / TLSB / TLY / TLC / TLM / TLV / TL0 / TL1 / TL2).

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
# task -> TLS, TLSB, TLY, TLC, TLM, TLV (or the earlier TL0, TL1, TL2)
```

All tasks start at a target of 24 °C and use a target range of 10–36 °C. The optional
preheat also targets 24 °C. Task parameters and stop criteria are saved to
`TD_*_config.json`; trial events use the same TD CSV format. Training order:
TLS(B) → TLY → TLS → TLC → TLM → TLV.

| Task | Timing and temperature contingency |
|---|---|
| `TLS` Shaping | 30 s choice window; poke +5/6/7 °C; no choice −1/2/3 °C; 10 s feedback after a poke. LED is on during the window except for 10 s after a poke. |
| `TLSB` | TLS, with the first 10 min as baseline: target held at 24 °C and outcomes off. |
| `TLY` Yoked | Pokes have no temperature effect; starts at 24 °C and replays the selected TLS target schedule, clamped to 10–36 °C. |
| `TLC` Cue | 25 s choice window and 10 s feedback; poke +5/6/7 °C; no choice −1/2/3 °C. |
| `TLM` Main | 20 s choice window and 10 s feedback; poke +2/4/6 °C; no choice 0/−2/−4 °C; outcome starts 1 s after the response/window and about 10 % of pokes omit the bump. |
| `TLV` Value cue | TLM plus LED flicker during the choice window announces the bump (steady / 2 Hz / 5 Hz, order flipped for odd-digit-sum mouse IDs); about 10 % of trials add a non-contingent ±4 °C shift held for 10 s with the LED off. |
| `TL0` | No choice window. Target drifts continuously at −3 °C/min, the theoretical mean TL2 no-choice decrement (mean −2 °C per 40 s trial). Each poke raises the target by +5/6/7 °C and holds the new target for 60 s. Pokes during that hold are accepted, add another increment, and restart the 60 s hold. |
| `TL1` | 40 s choice window and 10 s feedback; poke +5/6/7 °C; no choice −1/2/3 °C. |
| `TL2` | 30 s choice window and 10 s feedback; poke +2/4/6 °C; no choice 0/−2/−4 °C. |

In trial-based tasks, an outcome changes the target at the response or window end. The
controller then drives the chamber toward that target during feedback and keeps holding it
after feedback until a later outcome changes it. TLS–TLV mark a poke with a 0.5 s LED flash
at 100 % (normal brightness is `led_brightness` in test.json). Probe and yoked trials log
Bump/OutcomeDelta = 0; `ProbeStart` / `ProbeEnd` rows mark the probe.

Every task also ends at the 60 min task-time limit, if measured temperature stays at or below
10.5 °C for 20 min continuously (lower-bound disengagement), or if the rolling mean measured
temperature reaches 24 °C. The mean uses the latest 30 min for all tasks except TL2, which uses
45 min; it is evaluated after a full window of measurements is available. The 10.5 °C cutoff
allows 0.5 °C tolerance around the 10 °C lower bound.

TLM/TLV extra TD rows: `OutcomeOn` (moment the delayed bump/drop is commanded), `Omission`
(poke whose drawn Bump was not delivered; its Poke row has OutcomeDelta = 0),
`PerturbStart` / `PerturbEnd` (non-contingent shift; Bump = shift). The `Cue` column holds the
trial's LED flicker Hz (TLV) and the cue map is saved in `TD_*_config.json`.

Yoking: at the end of every session the commanded target schedule is saved to
`<yoke_dir>/<mouse>/yoke_<mouse>_<session>_<task>_<timestamp>.json`
(`yoke_dir` from test.json, default `../_yoke_schedules`, i.e. outside the session
folders, so `migrate_to_server.py` never deletes it). When `TLY` is chosen,
maintemp lists that mouse's completed TLS sessions (newest first) and asks which to
replay; it refuses to start if none exist. The source is recorded in the TLY config.

Name the session after the task (e.g. `d13_TLS`) so the TD `Task` column records it.

Wiring: `PLAN.md` / `SETUP.md`
