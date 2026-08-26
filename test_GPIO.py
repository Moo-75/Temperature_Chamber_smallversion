import time
import maze


def main():
    json_dir = input("json file [test.json]: ").strip() or "test.json"
    print("=== LED ===")
    led = maze.LED(json_dir)
    led.on()
    time.sleep(0.5)
    led.off()
    print("LED on/off ok")

    print("=== poke (1s sample) ===")
    sensor = maze.Sensor(json_dir)
    for _ in range(10):
        print("poke=", sensor.get())
        time.sleep(0.1)

    print("=== TTL ===")
    ttl = maze.TTL(json_dir)
    ttl.pulse(0.05)
    print("ttl_in=", ttl.read_in())

    print("=== Arduino KY-013 + driver ===")
    peltier = maze.Peltier_module(json_dir)
    try:
        peltier.start_control()
        peltier.set_target_temperature(25.0)
        for i in range(8):
            ctrl = peltier.get_ctrl()
            print(f"[{i+1}/8] {ctrl}")
            time.sleep(0.5)
        peltier.stop_control()
    finally:
        peltier.close()
    print("done")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print("Hardware test failed:", e)
        print("Check /dev/arduino (./link_arduino.sh) and peltier_operating_system.ino firmware")
