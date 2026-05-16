"""
i2head_tester.py

Interactive CLI tool for testing i2Head PCA9685 servos via Arduino.
Sends direct channel:angle commands over serial.

Usage:
  python3 client/i2head_tester.py                    # default /dev/ttyACM0
  python3 client/i2head_tester.py /dev/ttyACM1        # custom port
  python3 client/i2head_tester.py /dev/ttyACM0 115200 # port and baud
"""

import sys
import serial
import time

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Connected to {port} at {baud} baud")
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)

    time.sleep(2)  # Wait for Arduino reset

    # Read startup message
    startup = ser.readline().decode().strip()
    if startup:
        print(f"Arduino: {startup}")

    print("\nCommands:")
    print("  ch:ang,ch:ang,...  Set servo channels to angles")
    print("  home                Move all to 90 degrees")
    print("  sweep              Test all channels 0-15")
    print("  quit                Exit\n")

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
            cmd = ",".join(f"{ch}:90" for ch in range(16))

        if cmd == "sweep":
            print("Sweeping channels 0-15 from 0 to 180...")
            for ch in range(16):
                for ang in range(0, 181, 30):
                    c = f"{ch}:{ang}"
                    ser.write((c + "\n").encode())
                    time.sleep(0.1)
                    fb = ser.readline().decode().strip()
                    print(f"  {c} -> {fb}")
                for ang in range(180, -1, -30):
                    c = f"{ch}:{ang}"
                    ser.write((c + "\n").encode())
                    time.sleep(0.1)
                    fb = ser.readline().decode().strip()
                    print(f"  {c} -> {fb}")
            continue

        ser.write((cmd + "\n").encode())
        feedback = ser.readline().decode().strip()
        print(feedback)

    ser.close()
    print("Disconnected.")


if __name__ == "__main__":
    main()
