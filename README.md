# Temperature_Chamber_smallversion

Small temperature chamber for Temperature Lift and Cooling tasks (TLS / TLSB / TLY / TLC / TLM / TLV / TL0 / TL1 / TL2 / TLC0 / TLC1 / TLC2).

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
# task -> TLS, TLSB, TLY, TLC, TLM, TLV, TL0, TL1, TL2, TLC0, TLC1, TLC2
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
| `TLC0` Continuous cooling | Each poke lowers the target by 3, 4, or 5 °C and restarts a 60 s hold. The target rises at 1.5 °C/min outside the hold. |
| `TLC1` Cooling training | 40 s choice and 10 s feedback; poke −3/−4/−5 °C; no choice 0/+2 °C. |
| `TLC2` Cooling experiment | 30 s choice and 10 s feedback; poke −2/−4/−6 °C; no choice 0/+2 °C. |

TLC0, TLC1 and TLC2 start at a 24 °C target and clamp target commands to 16–40 °C.
For TLC1 and TLC2, no-choice changes are shuffled in blocks of 20 with ten 0 °C and ten +2 °C outcomes.
TLC1 targets lie on a 1 °C grid and TLC2 targets lie on a 2 °C grid.

In trial-based tasks, an outcome changes the target at the response or window end. The
controller then drives the chamber toward that target during feedback and keeps holding it
after feedback until a later outcome changes it. TLS–TLV mark a poke with a 0.5 s LED flash
at 100 % (normal brightness is `led_brightness` in test.json). Probe and yoked trials log
Bump/OutcomeDelta = 0; `ProbeStart` / `ProbeEnd` rows mark the probe.

Every task has a 60 min task-time limit. TLC0–TLC2 also end when the target stays at
40 °C for 20 min continuously, or when the measured rolling mean is at or below 30 °C.
The rolling window is 30 min for TLC0 and TLC1 and 45 min for TLC2.

Other tasks also end when measured temperature stays at or below 10.5 °C for 20 min
continuously, or when the rolling mean reaches 24 °C. Their mean window is 30 min, except
for TL2, which uses 45 min. Rolling means are evaluated after a full window is available.

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
