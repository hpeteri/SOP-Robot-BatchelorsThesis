"""
i2head_tester.py

Interactive CLI tool for testing i2Head PCA9685 servos via Arduino.
Works with the data-driven firmware (CFG protocol).

Usage:
  python3 client/i2head_tester.py                    # default /dev/ttyACM0
  python3 client/i2head_tester.py /dev/ttyACM1        # custom port
  python3 client/i2head_tester.py /dev/ttyACM0 115200 # port and baud
"""

import sys
import serial
import time
import yaml

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200


def load_config(path):
    """Load YAML and return list of (joint_name, cfg) tuples sorted by channel."""
    with open(path) as f:
        config = yaml.safe_load(f)
    servos = []
    if "servos" in config:
        for name, cfg in config["servos"].items():
            servos.append((name, cfg))
    servos.sort(key=lambda x: x[1].get("channel", 0))
    return servos


def send_cfg(ser, servos):
    """Send full config to Arduino firmware."""
    for name, cfg in servos:
        ch = cfg.get("channel", 0)
        cmd = f"CFG:{ch}:{cfg.get('pulse_min',500)}:{cfg.get('pulse_max',2500)}:{cfg.get('angle_min',0)}:{cfg.get('angle_max',180)}:{cfg.get('home',90)}:{1 if cfg.get('reversed',False) else 0}\n"
        ser.write(cmd.encode())
        resp = ser.readline().decode().strip()
        print(f"  {cmd.strip()} -> {resp}")

    ser.write(b"CFG_DONE\n")
    print(f"  CFG_DONE -> {ser.readline().decode().strip()}")

    ser.write(b"HOME\n")
    print(f"  HOME -> {ser.readline().decode().strip()}")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD
    config_path = sys.argv[3] if len(sys.argv) > 3 else None

    try:
        ser = serial.Serial(port, baud, timeout=2)
        print(f"Connected to {port} at {baud} baud")
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)

    time.sleep(2)

    # Read startup messages
    while ser.in_waiting:
        print(f"Arduino: {ser.readline().decode().strip()}")

    # Load and send config if YAML provided
    if config_path:
        print(f"\nLoading config from {config_path}...")
        servos = load_config(config_path)
        if servos:
            send_cfg(ser, servos)
            print(f"Configured {len(servos)} servos\n")

    print("Commands:")
    print("  ch:ang,ch:ang,...  Set servo channels to angles")
    print("  home               Move all to home")
    print("  sweep              Test all configured channels")
    print("  config <yaml>      Load and send YAML config")
    print("  quit               Exit\n")

    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not cmd:
            continue

        if cmd == "quit":
            break

        if cmd == "home":
            ser.write(b"HOME\n")
            print(ser.readline().decode().strip())
            continue

        if cmd.startswith("config "):
            path = cmd[7:].strip()
            servos = load_config(path)
            if servos:
                send_cfg(ser, servos)
            continue

        if cmd == "sweep":
            print("Sweeping channels 0-15...")
            for ch in range(16):
                for ang in range(0, 181, 30):
                    c = f"{ch}:{ang}"
                    ser.write((c + "\n").encode())
                    time.sleep(0.05)
                for ang in range(180, -1, -30):
                    c = f"{ch}:{ang}"
                    ser.write((c + "\n").encode())
                    time.sleep(0.05)
            print("Sweep done")
            continue

        ser.write((cmd + "\n").encode())
        feedback = ser.readline().decode().strip()
        print(feedback)

    ser.close()
    print("Disconnected.")


if __name__ == "__main__":
    main()
