import json
import math
import time

import RPi.GPIO as GPIO
import serial

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


class Sensor:
    def __init__(self, dir):
        with open(dir, "r") as config:
            data = json.load(config)
            gpio = data["GPIO"]
            poke_cfg = data.get("poke", {})
            self.pin = gpio["nose_poke"]
            self.on_level = poke_cfg.get("on", 1)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def poked(self):
        return GPIO.input(self.pin) == self.on_level

    def get(self):
        return 1 if self.poked() else 0


class LED:
    def __init__(self, dir):
        with open(dir, "r") as config:
            data = json.load(config)
            self.pin = data["GPIO"]["led"]
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
        self._on = False

    def on(self):
        GPIO.output(self.pin, GPIO.HIGH)
        self._on = True

    def off(self):
        GPIO.output(self.pin, GPIO.LOW)
        self._on = False

    def is_on(self):
        return self._on


class TTL:
    def __init__(self, dir):
        with open(dir, "r") as config:
            data = json.load(config)
            gpio = data["GPIO"]
            self.out_pin = gpio["ttl_out"]
            self.in_pin = gpio["ttl_in"]
        GPIO.setup(self.out_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.in_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def out_high(self):
        GPIO.output(self.out_pin, GPIO.HIGH)

    def out_low(self):
        GPIO.output(self.out_pin, GPIO.LOW)

    def pulse(self, duration_sec=0.01):
        self.out_high()
        time.sleep(duration_sec)
        self.out_low()

    def read_in(self):
        return GPIO.input(self.in_pin)


class Peltier_module:
    """Arduino USB serial: KY-013 read + BTS7960 PWM. Pi only sends commands."""

    SERIAL_PORT = "/dev/arduino"
    SERIAL_BAUD = 115200
    SERIAL_TIMEOUT = 1
    RECONNECT_DELAY_SEC = 1.0

    def __init__(self, dir="test.json"):
        self.target_temp = 25.0
        self.attenuation = 0.0
        self.att_min = 10.0
        self.att_max = 45.0
        self._last_set_temp_cmd_monotonic = 0.0
        self.set_temp_min_interval_sec = 0.05
        self._is_controlling = False
        self.ser = None
        self.heat_duty = 0.0
        self.cool_duty = 0.0

        try:
            with open(dir, "r") as config:
                data = json.load(config)
                arduino = data.get("arduino", {})
                self.SERIAL_PORT = arduino.get("port", self.SERIAL_PORT)
                self.SERIAL_BAUD = int(arduino.get("baudrate", self.SERIAL_BAUD))
        except (OSError, json.JSONDecodeError):
            pass

        self._open_serial_port()
        self._wait_for_ready()

    def _open_serial_port(self):
        ser = serial.Serial()
        ser.port = self.SERIAL_PORT
        ser.baudrate = self.SERIAL_BAUD
        ser.timeout = self.SERIAL_TIMEOUT
        ser.dtr = False
        ser.rts = False
        ser.open()
        self.ser = ser

    def _wait_for_ready(self):
        time.sleep(2.0)
        initial_message = self.ser.readline().decode("utf-8", errors="replace").strip()
        print(f"Arduino says: {initial_message}")
        if "Ready" not in initial_message:
            print("Warning: Arduino might not be ready.")

    def _write_line(self, command):
        self.ser.write(f"{command}\n".encode("utf-8"))

    def reconnect(self):
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        time.sleep(self.RECONNECT_DELAY_SEC)
        try:
            self._open_serial_port()
            self._wait_for_ready()
            if self._is_controlling:
                self._write_line("START")
            self._write_line(f"SET_TEMP,{self.target_temp}")
        except (OSError, serial.SerialException) as e:
            print(f"[Peltier] reconnect failed (device not back yet?): {e}")
            return False
        print(f"[Peltier] serial reconnected; restored target={self.target_temp} C")
        return True

    def send_command(self, command, no_response=True):
        for attempt in range(2):
            try:
                self._write_line(command)
                if no_response:
                    try:
                        self.ser.reset_input_buffer()
                    except Exception:
                        pass
                    return None
                timeout_start = time.time()
                while self.ser.in_waiting == 0:
                    if time.time() - timeout_start > 2.0:
                        print(f"Warning: Arduino response timeout for command: {command}")
                        return None
                    time.sleep(0.05)
                return self.ser.readline().decode("utf-8", errors="replace").strip()
            except (OSError, serial.SerialException) as e:
                print(f"[Peltier] serial I/O error on '{command}': {e}")
                if attempt == 0 and self.reconnect():
                    continue
                return None
        return None

    def get_ctrl(self):
        """Arduino GET_CTRL: t,t_pred,rate,u,f_hat,i,target,adc,ohm,pwm."""
        response = self.send_command("GET_CTRL", no_response=False)
        if not response:
            return None
        try:
            parts = [p.strip() for p in response.split(",") if p.strip() != ""]
            if len(parts) < 10:
                return None
            vals = [float(p) for p in parts[:10]]
            if not math.isfinite(vals[0]):
                return None
            u = vals[3]
            pwm = vals[9]
            self.heat_duty = max(0.0, u) * 100.0
            self.cool_duty = max(0.0, -u) * 100.0
            return {
                "sensor_temp": vals[0],
                "t_pred": vals[1],
                "rate": vals[2],
                "u": u,
                "d_term": vals[4],
                "f_hat": vals[4],
                "i_term": vals[5],
                "target_temp": vals[6],
                "adc": vals[7],
                "ohm": vals[8],
                "pwm": pwm,
                "heat_duty": self.heat_duty,
                "cool_duty": self.cool_duty,
            }
        except ValueError:
            return None

    def get_temperature(self):
        ctrl = self.get_ctrl()
        if ctrl is not None:
            return ctrl["sensor_temp"]
        response = self.send_command("GET_TEMP", no_response=False)
        if not response:
            return None
        try:
            parts = [float(x) for x in response.split(",") if x.strip()]
            temps = [t for t in parts if math.isfinite(t)]
            if not temps:
                return None
            return sum(temps) / len(temps)
        except ValueError:
            return None

    def get_temperatures(self):
        temp = self.get_temperature()
        return temp, temp

    def temperature_seton(self, temp, tolerance=0.5, timeout_sec=295, shared_data=None, dict_lock=None):
        def _sync_shared(curr):
            if shared_data is None or dict_lock is None:
                return
            with dict_lock:
                if curr is not None:
                    shared_data["sensor_temp"] = curr
                    shared_data["average_temp"] = curr
                shared_data["target_temp"] = self.target_temp

        temp = self.set_target_temperature(temp=temp)
        self.start_control()
        print("waiting for temperature set on ...")
        deadline = time.time() + timeout_sec
        curr_temp = None
        while True:
            curr_temp = self.get_temperature()
            _sync_shared(curr_temp)
            if curr_temp is not None and abs(curr_temp - temp) <= tolerance:
                break
            if time.time() >= deadline:
                print(f"temperature set on timeout at target {temp} C (current: {curr_temp} C)")
                return False
            print(curr_temp, "/", temp)
            time.sleep(0.2)
        print(f"temperature set on {curr_temp} C")
        return True

    def set_target_temperature(self, temp):
        temp = float(temp)
        now = time.time()
        elapsed = now - self._last_set_temp_cmd_monotonic
        if elapsed < self.set_temp_min_interval_sec:
            time.sleep(max(0.001, self.set_temp_min_interval_sec - elapsed))
        self.send_command(f"SET_TEMP,{temp}")
        self.target_temp = temp
        self._last_set_temp_cmd_monotonic = time.time()
        return temp

    def set_temperature_attenuation(self, attenuation):
        self.attenuation = attenuation

    def temp_updown(self, temp_updown):
        self.set_target_temperature(self.target_temp + temp_updown)

    def start_control(self):
        self._is_controlling = True
        self.send_command("START")
        self.send_command(f"SET_TEMP,{self.target_temp}")

    def stop_control(self):
        self._is_controlling = False
        self.send_command("STOP")

    def close(self):
        try:
            self.stop_control()
        except Exception:
            pass
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        print("Serial connection closed.")


if __name__ == "__main__":
    json_dir = input("json file [test.json]: ").strip() or "test.json"
    while True:
        b = input(
            "Enter number you want to test\n"
            "[0] LED  [1] Sensor  [2] TTL  [3] Temperature  [4] Exit\n"
        ).strip()
        if b == "0":
            led = LED(json_dir)
            while True:
                on = input("LED 1=on 0=off e=exit: ").strip()
                if on == "1":
                    led.on()
                elif on == "0":
                    led.off()
                elif on == "e":
                    led.off()
                    break
        elif b == "1":
            sensor = Sensor(json_dir)
            while True:
                c = input("1=read  2=exit: ").strip()
                if c == "1":
                    print("poke=", sensor.get())
                elif c == "2":
                    break
        elif b == "2":
            ttl = TTL(json_dir)
            while True:
                c = input("h=out high  l=out low  p=pulse  r=read in  e=exit: ").strip()
                if c == "h":
                    ttl.out_high()
                elif c == "l":
                    ttl.out_low()
                elif c == "p":
                    ttl.pulse(0.05)
                elif c == "r":
                    print("ttl_in=", ttl.read_in())
                elif c == "e":
                    ttl.out_low()
                    break
        elif b == "3":
            peltier = Peltier_module(json_dir)
            try:
                peltier.start_control()
                for i in range(10):
                    t = peltier.get_temperature()
                    print(f"[{i+1}/10] {t}")
                    time.sleep(0.5)
                peltier.stop_control()
            finally:
                peltier.close()
        elif b == "4":
            break
        else:
            print("Wrong input")
