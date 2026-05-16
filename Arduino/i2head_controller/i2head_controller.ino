/**
 * i2head_controller.ino
 *
 * Controls PCA9685-based servo driver for InMoov i2Head facial expressions.
 * Receives commands over serial: channel:angle,channel:angle,...\n
 *
 * Requires Adafruit_PWMServoDriver library:
 *   https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
 *
 * Wiring:
 *   PCA9685 SCL -> Arduino SCL (A5 on Uno, 21 on Mega)
 *   PCA9685 SDA -> Arduino SDA (A4 on Uno, 20 on Mega)
 *   PCA9685 VCC -> 5V
 *   PCA9685 GND -> GND
 *   Servos on PCA9685 channels 0-15 per config
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int NUM_CHANNELS = 16;
const int SERIAL_BAUD = 115200;

// Default servo pulse range (microseconds)
const int SERVO_MIN_PULSE = 500;   // 0 degrees
const int SERVO_MAX_PULSE = 2500;  // 180 degrees

// Per-channel pulse limits for finer control, indexed by PCA9685 channel
const int MIN_PULSE[NUM_CHANNELS] = {
  500, 500, 500, 500, 500, 500, 500, 500,
  500, 500, 500, 500, 500, 500, 500, 500
};
const int MAX_PULSE[NUM_CHANNELS] = {
  2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500,
  2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500
};

// Safe angle limits per channel (degrees)
const int ANGLE_MIN[NUM_CHANNELS] = {
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0
};
const int ANGLE_MAX[NUM_CHANNELS] = {
  180, 180, 180, 180, 180, 180, 180, 180,
  180, 180, 180, 180, 180, 180, 180, 180
};

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(500);
  }

  pwm.begin();
  pwm.setPWMFreq(50); // 50 Hz for standard servos
  delay(10);

  // Set all channels to neutral (90 degrees) on startup
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    setServoAngle(ch, 90);
  }

  Serial.println("i2head_controller ready");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    parseAndExecute(command);
  }
}

void parseAndExecute(String command) {
  command.trim();
  if (command.length() == 0) {
    return;
  }

  // Format: channel:angle,channel:angle,...
  int start = 0;
  while (start < command.length()) {
    int colon = command.indexOf(':', start);
    int comma = command.indexOf(',', start);

    if (colon == -1) {
      break;
    }

    int channel = command.substring(start, colon).toInt();

    String angleStr;
    if (comma == -1) {
      angleStr = command.substring(colon + 1);
      start = command.length();
    } else {
      angleStr = command.substring(colon + 1, comma);
      start = comma + 1;
    }

    int angle = angleStr.toInt();

    if (channel >= 0 && channel < NUM_CHANNELS) {
      int clamped = constrain(angle, ANGLE_MIN[channel], ANGLE_MAX[channel]);
      setServoAngle(channel, clamped);
    }
  }

  Serial.print("OK:");
  Serial.println(command);
}

void setServoAngle(int channel, int angle) {
  int pulse = map(angle, 0, 180, MIN_PULSE[channel], MAX_PULSE[channel]);
  pwm.setPWM(channel, 0, pulseToTicks(pulse));
}

int pulseToTicks(int pulseUs) {
  // PCA9685: 12-bit resolution, 4096 ticks per cycle
  // At 50 Hz, cycle = 20000 us
  // ticks = pulse_us * 4096 / 20000
  return pulseUs * 4096 / 20000;
}
