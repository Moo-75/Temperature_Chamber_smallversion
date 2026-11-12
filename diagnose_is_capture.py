#!/usr/bin/env python3
"""Capture IBT-2 current-sense (IS) traces at ~1 kHz and classify them.

Wiring (two Dupont wires, logic side only):
    IBT-2 R_IS -> Arduino A1
    IBT-2 L_IS -> Arduino A2

The BTS7960 IS pin sources a current proportional to the HIGH-side load
current. Heat (RPWM, D9) conducts through the R high side, so R_IS is the
active channel. Cool (LPWM, D10) uses L_IS. On a fault (over-temperature,
over-current, under-voltage) IS sources a fixed large current instead, which
reads near the top of the ADC.

Each capture is: pre_ms of STOP baseline, then a fixed signed duty. The
firmware also records the real D9/D10 pin levels and A0 (thermistor), so
every PWM pulse can be checked for current, and the A0 on/off difference
shows ground shift from the return current.

Verdicts per capture:
    CONTINUOUS        every pulse conducts. Power reaches the bridge output.
    HICCUP            pulses conduct only in bursts every ~1-6 s.
    CUT_IN_PULSE      current starts in each pulse and collapses inside it.
    NO_CURRENT        no pulse conducts at all.
    FAULT             IS at the BTS7960 fault level.
    IRREGULAR         none of the above.

Run the same arguments on a working chamber too and compare the on-levels.

    ./upload_arduino.sh --kill
    python3 diagnose_is_capture.py                      # 15%, 30%, heat and cool
    python3 diagnose_is_capture.py --duties 0.15 --ms 9000
    python3 diagnose_is_capture.py --ris 1000           # also print rough amps
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import statistics
import struct
import sys
import time
from dataclasses import dataclass, field

ADC_MAX = 1023
SAMPLE_BYTES = 4
PWM_PERIOD_S = 0.2          # Timer1 TOP 3124, prescaler 1024 -> 5 Hz
FAULT_COUNTS = 900          # ~4.4 V at 5 V ref. BTS7960 fault current saturates IS.
FLOAT_MEAN_COUNTS = 60      # unwired analog input drifts toward the previous channel
FLOAT_STD_COUNTS = 8
MIN_STEP_COUNTS = 10        # ~50 mV above baseline counts as current
FULL_PULSE_FRAC = 0.8
CONTINUOUS_FRAC = 0.9
HICCUP_MIN_S = 1.0
HICCUP_MAX_S = 6.0
HICCUP_MAX_ACTIVE_FRAC = 0.6
K_ILIS = 8500.0             # BTS7960 typical load/sense current ratio
A0_C_PER_COUNT = 0.088      # 10 k NTC, 10 k series, ~24 C


class CaptureError(Exception):
    pass


@dataclass
class Capture:
    u: float
    period_us: int
    pre_n: int
    samples: list = field(default_factory=list)  # (a1, a2, a0, ph9, ph10)
    overruns: int = 0
    aborted: bool = False
    elapsed_us: int = 0

    @property
    def dt(self):
        return self.period_us / 1e6


def unpack_samples(raw):
    out = []
    for (word,) in struct.iter_unpack("<I", raw):
        out.append((
            word & 0x3FF,
            (word >> 10) & 0x3FF,
            (word >> 20) & 0x3FF,
            (word >> 30) & 1,
            (word >> 31) & 1,
        ))
    return out


def pack_samples(samples):
    """Inverse of unpack_samples. Used by the self-test."""
    raw = bytearray()
    for a1, a2, a0, ph9, ph10 in samples:
        word = a1 | (a2 << 10) | (a0 << 20) | (ph9 << 30) | (ph10 << 31)
        raw += struct.pack("<I", word)
    return bytes(raw)


def mean_sd(values):
    if not values:
        return None, None
    if len(values) == 1:
        return float(values[0]), 0.0
    return statistics.fmean(values), statistics.pstdev(values)


def counts_to_v(counts, vref):
    return counts * vref / ADC_MAX


# --------------------------------------------------------------------------
# Analysis (no serial needed)
# --------------------------------------------------------------------------

def check_baseline(cap):
    """STOP capture: decide whether each IS input looks wired."""
    result = {}
    for name, idx in (("R_IS(A1)", 0), ("L_IS(A2)", 1)):
        mu, sd = mean_sd([s[idx] for s in cap.samples])
        floating = mu is None or mu > FLOAT_MEAN_COUNTS or sd > FLOAT_STD_COUNTS
        result[name] = {"mean": mu, "sd": sd, "floating": floating}
    mu0, sd0 = mean_sd([s[2] for s in cap.samples])
    result["A0"] = {"mean": mu0, "sd": sd0}
    return result


def analyze(cap):
    heat = cap.u > 0
    act, idle = (0, 1) if heat else (1, 0)
    ph = 3 if heat else 4
    pre = cap.samples[:cap.pre_n]
    drive = cap.samples[cap.pre_n:]
    res = {
        "u": cap.u,
        "direction": "heat" if heat else "cool",
        "active": "R_IS(A1)" if heat else "L_IS(A2)",
        "overruns": cap.overruns,
        "aborted": cap.aborted,
    }
    if len(pre) < 20 or len(drive) < 20:
        res["verdict"] = "TOO_SHORT"
        return res

    mu0, sd0 = mean_sd([s[act] for s in pre])
    mu_idle0, _ = mean_sd([s[idle] for s in pre])
    thr = max(5.0 * sd0, MIN_STEP_COUNTS)
    res.update({"base_mean": mu0, "base_sd": sd0, "threshold": mu0 + thr})

    phase = [s[ph] for s in drive]
    on_frac_pin = sum(phase) / len(phase)
    res["pin_on_frac"] = on_frac_pin

    def near_edge(i):
        return (i > 0 and phase[i - 1] != phase[i]) or (i + 1 < len(phase) and phase[i + 1] != phase[i])

    fault_any = any(s[0] >= FAULT_COUNTS or s[1] >= FAULT_COUNTS for s in drive)
    fault_off = any(
        (s[act] >= FAULT_COUNTS) for i, s in enumerate(drive) if not phase[i] and not near_edge(i)
    )
    res["fault_any"] = fault_any
    res["fault_in_off_phase"] = fault_off

    if on_frac_pin == 0:
        res["verdict"] = "NO_PWM_ON_PIN"
        return res

    # Fixed 0.2 s windows aligned to the first on-sample. Timer1 and micros()
    # share the 16 MHz clock, so the windows stay aligned with the pulses even
    # at 100% duty, where there are no rising edges to segment on.
    first_on = phase.index(1)
    win = max(1, round(PWM_PERIOD_S / cap.dt))
    windows = []
    for start in range(first_on, len(drive), win):
        idxs = range(start, min(start + win, len(drive)))
        on_vals = [drive[i][act] for i in idxs if phase[i] and not near_edge(i)]
        if not on_vals:
            continue
        above = [v for v in on_vals if v > mu0 + thr]
        frac = len(above) / len(on_vals)
        if frac >= FULL_PULSE_FRAC:
            state = "full"
        elif len(above) >= 2:
            state = "partial"
        else:
            state = "none"
        windows.append({
            "t": (start) * cap.dt,
            "state": state,
            "frac": frac,
            "peak": max(on_vals),
            "above": above,
        })
    res["windows"] = len(windows)
    if not windows:
        res["verdict"] = "TOO_SHORT"
        return res

    n_full = sum(w["state"] == "full" for w in windows)
    n_partial = sum(w["state"] == "partial" for w in windows)
    n_active = n_full + n_partial
    res.update({"n_full": n_full, "n_partial": n_partial, "n_none": len(windows) - n_active})

    all_above = [v for w in windows for v in w["above"]]
    res["on_level_median"] = statistics.median(all_above) - mu0 if all_above else 0.0
    res["on_level_peak"] = max(w["peak"] for w in windows) - mu0

    # Bursts: runs of active windows.
    bursts = []
    prev_active = False
    for w in windows:
        active = w["state"] != "none"
        if active and not prev_active:
            bursts.append(w["t"])
        prev_active = active
    intervals = [b - a for a, b in zip(bursts, bursts[1:])]
    res["bursts"] = len(bursts)
    res["burst_interval_s"] = statistics.median(intervals) if intervals else None
    active_frac = n_active / len(windows)
    res["active_frac"] = active_frac

    # Idle channel during on-phase: should stay at its baseline.
    idle_on = [drive[i][idle] for i in range(len(drive)) if phase[i] and not near_edge(i)]
    res["idle_shift"] = (statistics.fmean(idle_on) - mu_idle0) if idle_on else None

    # A0 ground shift: on-phase minus off-phase.
    a0_on = [drive[i][2] for i in range(len(drive)) if phase[i] and not near_edge(i)]
    a0_off = [drive[i][2] for i in range(len(drive)) if not phase[i] and not near_edge(i)]
    if a0_on and a0_off:
        res["a0_shift"] = statistics.fmean(a0_on) - statistics.fmean(a0_off)
    else:
        res["a0_shift"] = None

    if fault_any:
        verdict = "FAULT"
    elif n_active == 0:
        verdict = "NO_CURRENT"
    elif n_full / len(windows) >= CONTINUOUS_FRAC:
        verdict = "CONTINUOUS"
    elif (
        len(bursts) >= 2
        and res["burst_interval_s"] is not None
        and HICCUP_MIN_S <= res["burst_interval_s"] <= HICCUP_MAX_S
        and active_frac <= HICCUP_MAX_ACTIVE_FRAC
    ):
        verdict = "HICCUP"
    elif n_partial >= n_full and active_frac >= 0.5:
        verdict = "CUT_IN_PULSE"
    else:
        verdict = "IRREGULAR"
    res["verdict"] = verdict
    return res


VERDICT_TEXT = {
    "CONTINUOUS": "모든 펄스에서 전류가 흐릅니다. 전력이 브리지 출력까지 나갑니다.",
    "HICCUP": "전류가 몇 초 간격으로 묶음으로만 흐릅니다. 전원 보호 재시작(히컵)과 맞습니다.",
    "CUT_IN_PULSE": "펄스마다 전류가 시작했다가 펄스 안에서 끊깁니다. 피크 과전류 보호와 맞습니다.",
    "NO_CURRENT": "어떤 펄스에서도 하이사이드 전류가 보이지 않습니다.",
    "FAULT": "IS가 BTS7960 고장 레벨입니다. 드라이버 자체 보호(과열, 과전류, 저전압)입니다.",
    "IRREGULAR": "규칙이 없는 통전입니다. CSV로 파형을 직접 보세요.",
    "NO_PWM_ON_PIN": "D9/D10 핀에서 PWM이 보이지 않습니다. 펌웨어나 EN_MODE를 확인하세요.",
    "TOO_SHORT": "분석할 샘플이 부족합니다. --ms 를 늘리세요.",
}


def fmt_counts(counts, vref, ris):
    if counts is None:
        return "-"
    text = f"{counts:6.1f} cnt ({counts_to_v(counts, vref):5.3f} V"
    if ris:
        amps = counts_to_v(counts, vref) / ris * K_ILIS
        text += f", 약 {amps:5.1f} A"
    return text + ")"


def print_result(res, vref, ris):
    print(f"  판정: {res['verdict']}  {VERDICT_TEXT.get(res['verdict'], '')}")
    if "windows" not in res:
        return
    interval = res.get("burst_interval_s")
    print(
        f"  PWM 주기 {res['windows']}개 중 전체통전 {res['n_full']}, 부분통전 {res['n_partial']}, "
        f"무전류 {res['n_none']}.  묶음 {res['bursts']}개"
        + (f", 간격 중앙값 {interval:.2f} s" if interval else "")
    )
    print(f"  {res['active']} 기준선 {res['base_mean']:.1f} ± {res['base_sd']:.1f} cnt")
    print(f"  통전 레벨 중앙값 {fmt_counts(res['on_level_median'], vref, ris)}")
    print(f"  통전 레벨 최대   {fmt_counts(res['on_level_peak'], vref, ris)}")
    if res.get("idle_shift") is not None:
        print(f"  반대쪽 IS 이동 {res['idle_shift']:+.1f} cnt (0 근처가 정상)")
    if res.get("a0_shift") is not None:
        print(
            f"  A0 켜짐-꺼짐 차이 {res['a0_shift']:+.2f} cnt "
            f"(겉보기 {res['a0_shift'] * -A0_C_PER_COUNT:+.3f} °C, 공통 GND 귀로 전류 흔적)"
        )
    if res["fault_in_off_phase"]:
        print("  꺼진 구간에도 고장 레벨이 남습니다. 고장이 래치된 상태입니다.")
    if res["overruns"]:
        print(f"  [주의] 샘플 지연 {res['overruns']}회. 시간축이 조금 어긋날 수 있습니다.")
    if res["aborted"]:
        print("  [주의] 캡처가 중간에 중단되었습니다.")


def interpret(baseline, results):
    print("\n" + "=" * 72)
    print("  종합")
    print("=" * 72)
    heat = [r for r in results if r["direction"] == "heat"]
    cool = [r for r in results if r["direction"] == "cool"]
    verdicts = {r["verdict"] for r in results}

    swapped = [
        r for r in results
        if r.get("verdict") == "NO_CURRENT" and (r.get("idle_shift") or 0) > MIN_STEP_COUNTS
    ]
    if swapped:
        print("  능동 채널은 조용한데 반대쪽 IS가 올라갑니다. R_IS/L_IS 선이 바뀌었을 수 있습니다.")

    if "FAULT" in verdicts:
        print("  BTS7960이 스스로 보호 동작에 들어갑니다.")
        print("  드라이버가 뜨거운 증상과 맞으면 과열 보호, 아니면 과전류나 12 V 저전압입니다.")
        print("  드라이버 쪽(모듈 자체, 출력 단락, B+ 전압)을 먼저 보세요.")
    elif verdicts <= {"NO_CURRENT"}:
        print("  가열·냉각 모두 하이사이드로 전류가 흐르지 않습니다.")
        print("  그런데 SMPS LED가 깜빡인다면, 과전류는 드라이버 하이사이드를 지나지 않는 곳에서 납니다.")
        print("  (B+ 배선, SMPS 출력 쪽 단락, 또는 SMPS 자체) 12 V가 B+에 아예 안 와도 이렇게 보입니다.")
    elif verdicts & {"HICCUP", "CUT_IN_PULSE"}:
        print("  전류는 흐르지만 전원이 버티지 못하고 끊깁니다. 문서의 히컵 가설이 IS로 확인됩니다.")
        print("  남은 질문은 '부하가 비정상으로 큰가(단락), 전원이 약한가'입니다.")
        print("  같은 인자로 정상 챔버를 캡처해 통전 레벨 중앙값을 비교하세요.")
        print("    - 이 챔버가 훨씬 크다: 출력 단락 또는 펠티어 과다 병렬")
        print("    - 비슷한데 정상 챔버는 CONTINUOUS: 이 챔버의 SMPS 용량이나 상태 문제")
    elif verdicts <= {"CONTINUOUS"}:
        print("  모든 펄스에서 전류가 흐릅니다. 전력은 브리지 출력까지 나갑니다.")
        print("  이 상태에서 온도가 안 움직이면 펠티어 열 경로나 센서 부착을 봐야 합니다.")
        print("  이때 SMPS LED가 계속 깜빡인다면 그 원인은 드라이버 바깥입니다.")
    else:
        print("  결과가 섞여 있습니다. 방향별 판정과 CSV 파형을 같이 보세요.")

    hv = {r["verdict"] for r in heat}
    cv = {r["verdict"] for r in cool}
    if heat and cool and hv != cv:
        print(f"  가열({', '.join(sorted(hv))})과 냉각({', '.join(sorted(cv))})이 다릅니다.")
        print("  한쪽 반브리지나 한쪽 배선만의 문제일 수 있습니다.")
    print("=" * 72)


def write_csv(path, cap, vref):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([f"# u={cap.u} period_us={cap.period_us} pre_n={cap.pre_n} vref={vref}"])
        writer.writerow(["t_s", "drive", "d9", "d10", "is_r", "is_l", "a0"])
        for i, (a1, a2, a0, ph9, ph10) in enumerate(cap.samples):
            writer.writerow([f"{i * cap.dt:.4f}", int(i >= cap.pre_n), ph9, ph10, a1, a2, a0])


# --------------------------------------------------------------------------
# Serial
# --------------------------------------------------------------------------

def run_capture(ser, write_line, u, total_ms, pre_ms, period_us):
    ser.reset_input_buffer()
    write_line(ser, f"IS_CAP,{u:.3f},{total_ms},{pre_ms},{period_us}")
    header = None
    deadline = time.time() + 3.0
    while time.time() < deadline:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if line.startswith("ISCAP,BEGIN"):
            header = line
            break
        if line == "ERR":
            raise CaptureError("펌웨어가 IS_CAP 인자를 거부했습니다.")
    if header is None:
        raise CaptureError("IS_CAP 응답이 없습니다. 새 펌웨어를 올렸는지 확인하세요 (./upload_arduino.sh --kill).")
    _, _, n_txt, period_txt, pre_txt = header.split(",")
    n, period, pre_n = int(n_txt), int(period_txt), int(pre_txt)

    need = n * SAMPLE_BYTES
    buf = bytearray()
    deadline = time.time() + total_ms / 1000.0 + 3.0
    while len(buf) < need and time.time() < deadline:
        chunk = ser.read(min(4096, need - len(buf)))
        if chunk:
            buf.extend(chunk)

    end = None
    if len(buf) < need:
        # Aborted early: the END line arrived inside the binary read.
        # Records are 4-byte aligned, so only a marker at a record boundary counts.
        marker = b"\nISCAP,END,"
        pos = buf.find(marker)
        while pos >= 0 and pos % SAMPLE_BYTES:
            pos = buf.find(marker, pos + 1)
        if pos >= 0:
            tail = bytes(buf[pos + 1:]).split(b"\n")[0]
            end = tail.decode("utf-8", errors="replace").strip()
            del buf[pos:]
    end_deadline = time.time() + 2.0
    while end is None and time.time() < end_deadline:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if line.startswith("ISCAP,END"):
            end = line
            break
    cap = Capture(u=u, period_us=period, pre_n=pre_n)
    usable = len(buf) - len(buf) % SAMPLE_BYTES
    cap.samples = unpack_samples(bytes(buf[:usable]))
    if end:
        _, _, sent, overruns, elapsed, aborted = end.split(",")
        cap.overruns = int(overruns)
        cap.elapsed_us = int(elapsed)
        cap.aborted = aborted == "1" or int(sent) < n
    else:
        cap.aborted = True
    if len(buf) < need:
        cap.aborted = True
    return cap


def parse_duties(text):
    duties = []
    for token in text.split(","):
        token = token.strip()
        if not token:
            continue
        value = float(token)
        if value > 1.0:
            value /= 100.0
        if not 0.02 < value <= 1.0:
            raise ValueError(f"듀티 {token} 는 2% 초과 100% 이하여야 합니다.")
        duties.append(value)
    if not duties:
        raise ValueError("듀티가 비었습니다.")
    return duties


def main():
    parser = argparse.ArgumentParser(description="IBT-2 IS current-sense capture")
    parser.add_argument("--json", default="test.json")
    parser.add_argument("--duties", default="0.15,0.30", help="쉼표 구분. 0~1 또는 퍼센트")
    parser.add_argument("--direction", choices=("both", "heat", "cool"), default="both")
    parser.add_argument("--ms", type=int, default=7000, help="캡처 길이(ms). 히컵 2주기 이상, 최대 12000")
    parser.add_argument("--pre", type=int, default=500, help="캡처 앞 STOP 기준선(ms)")
    parser.add_argument("--period-us", type=int, default=1000, help="샘플 간격(us). 500 이상")
    parser.add_argument("--rest", type=float, default=5.0, help="단계 사이 STOP 휴식(초)")
    parser.add_argument("--en-mode", choices=("BOTH", "ONE"), default="BOTH")
    parser.add_argument("--vref", type=float, default=5.0, help="ADC 기준 전압(V)")
    parser.add_argument("--ris", type=float, default=None, help="IBT-2 IS 저항(Ω). 주면 대략 A 환산")
    parser.add_argument("--out", default=None, help="CSV 저장 폴더")
    args = parser.parse_args()

    try:
        duties = parse_duties(args.duties)
    except ValueError as exc:
        print(f"[오류] {exc}")
        return 2
    if not 1000 <= args.ms <= 12000:
        print("[오류] --ms 는 1000~12000 사이여야 합니다.")
        return 2
    if not 0 < args.pre < args.ms:
        print("[오류] --pre 는 0보다 크고 --ms 보다 작아야 합니다.")
        return 2

    from diagnose_driver import SerialException, cmd, find_serial_port, open_serial, write_line

    signs = {"both": ((1.0, "가열"), (-1.0, "냉각")), "heat": ((1.0, "가열"),), "cool": ((-1.0, "냉각"),)}
    steps = [(duty, sign, name) for duty in duties for sign, name in signs[args.direction]]
    out_dir = args.out or time.strftime("is_capture_%Y%m%d_%H%M%S")
    os.makedirs(out_dir, exist_ok=True)

    print("=" * 72)
    print("  IBT-2 IS 고속 캡처")
    print("=" * 72)
    print("  준비: IBT-2 R_IS -> A1, L_IS -> A2. maintemp.py 와 task 는 먼저 종료.")
    print(f"  단계 {len(steps)}개, 각 {args.ms / 1000:.1f}초 (앞 {args.pre} ms는 STOP 기준선).")
    print(f"  EN_MODE={args.en_mode}. 샘플 간격 {args.period_us} us. CSV: {out_dir}/")
    print("  각 단계에서 SMPS LED 상태를 같이 봐 두세요. Ctrl+C 는 즉시 STOP 합니다.")
    print()

    candidates, baudrate = find_serial_port(args.json)
    ser = None
    for port in candidates:
        try:
            print(f"연결 {port} ...", end=" ", flush=True)
            ser, _ = open_serial(port, baudrate)
            print("OK")
            break
        except SerialException as exc:
            print(f"실패 ({exc})")
            ser = None
    if ser is None:
        print("[실패] 시리얼을 열 수 없습니다.")
        return 2

    summary = {"args": vars(args), "baseline": None, "results": []}
    try:
        cmd(ser, "STOP", expect_reply=False)
        time.sleep(0.2)
        reply = cmd(ser, f"EN_MODE,{args.en_mode}")
        if reply != args.en_mode:
            print(f"  EN_MODE 응답 {reply!r}. 새 펌웨어를 올린 뒤 다시 실행하세요.")
            return 2

        print("\n--- [0] STOP 기준선 1초 (IS 선 연결 확인) ---")
        base_cap = run_capture(ser, write_line, 0.0, 1000, 1000, args.period_us)
        write_csv(os.path.join(out_dir, "00_stop.csv"), base_cap, args.vref)
        baseline = check_baseline(base_cap)
        summary["baseline"] = baseline
        floating = []
        for name in ("R_IS(A1)", "L_IS(A2)"):
            b = baseline[name]
            flag = "  [떠 있음: 미연결 의심]" if b["floating"] else ""
            if b["floating"]:
                floating.append(name)
            print(f"  {name}: {b['mean']:.1f} ± {b['sd']:.1f} cnt{flag}")
        print(f"  A0: {baseline['A0']['mean']:.1f} ± {baseline['A0']['sd']:.1f} cnt")
        if floating:
            print("\n  IS 입력이 떠 있는 것으로 보입니다. R_IS→A1, L_IS→A2 선을 확인하세요.")
            print("  이 상태의 캡처는 판정할 수 없어 여기서 멈춥니다.")
            return 2

        results = []
        summary["results"] = results
        for index, (duty, sign, name) in enumerate(steps, start=1):
            print(f"\n--- [{index}/{len(steps)}] {name} {duty:.0%}  {args.ms / 1000:.1f}s ---")
            cap = run_capture(ser, write_line, sign * duty, args.ms, args.pre, args.period_us)
            fname = f"{index:02d}_{'heat' if sign > 0 else 'cool'}_{round(duty * 100):03d}.csv"
            write_csv(os.path.join(out_dir, fname), cap, args.vref)
            res = analyze(cap)
            res["duty"] = duty
            res["csv"] = fname
            results.append(res)
            print_result(res, args.vref, args.ris)
            if index != len(steps):
                cmd(ser, "STOP", expect_reply=False)
                time.sleep(args.rest)

        print("\n" + "=" * 72)
        print("  기록")
        print("=" * 72)
        print(f"  {'방향':>4}  {'duty':>5}  {'판정':<13} {'전체/부분/무':>11}  {'통전레벨':>9}  {'묶음간격':>8}")
        for r in results:
            interval = r.get("burst_interval_s")
            counts = f"{r.get('n_full', '-')}/{r.get('n_partial', '-')}/{r.get('n_none', '-')}"
            level = r.get("on_level_median")
            print(
                f"  {'가열' if r['direction'] == 'heat' else '냉각':>4}  {r['duty']:5.0%}  {r['verdict']:<13} "
                f"{counts:>11}  {(f'{level:.1f}' if level is not None else '-'):>9}  "
                f"{(f'{interval:.2f}s' if interval else '-'):>8}"
            )
        interpret(baseline, results)
        return 0
    finally:
        try:
            write_line(ser, "STOP")
        except Exception:
            pass
        try:
            write_line(ser, "EN_MODE,BOTH")
        except Exception:
            pass
        try:
            with open(os.path.join(out_dir, "summary.json"), "w") as f:
                json.dump(summary, f, ensure_ascii=False, indent=2, default=str)
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        print(f"\n종료 (STOP / EN_MODE,BOTH). 파형: {out_dir}/")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n중단됨.")
        sys.exit(130)
