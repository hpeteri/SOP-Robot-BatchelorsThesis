/**
 * i2head_controller.ino
 *
 * Data-driven PCA9685 servo controller for InMoov i2Head.
 *
 * Protocol (newline-terminated commands):
 *   CFG:ch:minPulse:maxPulse:minAngle:maxAngle:home:reversed
 *       Configure a single channel. All values from YAML.
 *   CFG_DONE
 *       End of configuration. Moves all to home positions.
 *   ch:angle,ch:angle,...
 *       Set channels to angles (after CFG_DONE).
 *   HOME
 *       Move all channels to configured home positions.
 *
 * All servo-specific data comes from the ROS2 bridge node
 * which reads it from a YAML file. No hardcoded limits.
 *
 * Hardware:
 *   - Arduino (Uno/Mega/Leonardo)
 *   - Adafruit PCA9685 PWM driver (I2C address 0x40)
 *
 * Dependencies: Adafruit_PWMServoDriver, Wire
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int MAX_CHANNELS = 32;  // Support up to 2 daisy-chained PCA9685s
const int SERIAL_BAUD = 115200;
const int PWM_FREQ = 50;      // 50 Hz for standard servos

// Runtime servo configuration (populated via CFG commands from bridge)
int numChannels = 0;
int minPulse[MAX_CHANNELS];
int maxPulse[MAX_CHANNELS];
int minAngle[MAX_CHANNELS];
int maxAngle[MAX_CHANNELS];
int homeAngle[MAX_CHANNELS];
bool reversed[MAX_CHANNELS];
bool configured[MAX_CHANNELS] = {false};

bool configComplete = false;

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(500);
  }

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);
  delay(10);

  Serial.println("i2head_controller ready");
  Serial.println("Awaiting configuration (CFG commands)...");
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    if (line.startsWith("CFG:")) {
      handleConfig(line);
    } else if (line == "CFG_DONE") {
      finalizeConfig();
    } else if (line == "HOME") {
      moveToHome();
      Serial.println("HOME_OK");
    } else if (configComplete) {
      executeCommand(line);
    } else {
      Serial.println("ERR:Config not complete");
    }
  }
}

void handleConfig(String cmd) {
  // Format: CFG:ch:minPulse:maxPulse:minAngle:maxAngle:home:reversed
  int parts[8];
  int idx = 0;
  int start = 4; // after "CFG:"

  for (int i = 0; i < 8; i++) {
    int colon = cmd.indexOf(':', start);
    String val = (colon == -1) ? cmd.substring(start) : cmd.substring(start, colon);
    parts[i] = val.toInt();
    start = colon + 1;
    if (colon == -1) break;
  }

  int ch = parts[0];
  if (ch < 0 || ch >= MAX_CHANNELS) {
    Serial.print("ERR:Invalid channel ");
    Serial.println(ch);
    return;
  }

  minPulse[ch]    = parts[1];
  maxPulse[ch]    = parts[2];
  minAngle[ch]    = parts[3];
  maxAngle[ch]    = parts[4];
  homeAngle[ch]   = parts[5];
  reversed[ch]    = (parts[6] == 1);
  configured[ch]  = true;

  if (ch >= numChannels) numChannels = ch + 1;

  Serial.print("CFG_OK:");
  Serial.println(ch);
}

void finalizeConfig() {
  configComplete = true;
  Serial.print("CFG_DONE_OK:");
  Serial.print(numChannels);
  Serial.println(" channels configured");
  moveToHome();
}

void moveToHome() {
  for (int ch = 0; ch < numChannels; ch++) {
    if (configured[ch]) {
      setServoAngle(ch, homeAngle[ch]);
    }
  }
  Serial.println("HOME_OK");
}

void executeCommand(String cmd) {
  // Format: ch:angle,ch:angle,...
  int start = 0;
  int count = 0;

  while (start < cmd.length()) {
    int colon = cmd.indexOf(':', start);
    int comma = cmd.indexOf(',', start);

    if (colon == -1) break;

    int ch = cmd.substring(start, colon).toInt();

    String angleStr;
    if (comma == -1) {
      angleStr = cmd.substring(colon + 1);
      start = cmd.length();
    } else {
      angleStr = cmd.substring(colon + 1, comma);
      start = comma + 1;
    }

    int angle = angleStr.toInt();

    if (ch >= 0 && ch < numChannels && configured[ch]) {
      int clamped = constrain(angle, minAngle[ch], maxAngle[ch]);
      setServoAngle(ch, clamped);
      count++;
    }
  }

  Serial.print("OK:");
  Serial.print(count);
  Serial.println(" servos");
}

void setServoAngle(int channel, int angleDeg) {
  int effectiveAngle = reversed[channel] ? (180 - angleDeg) : angleDeg;
  int pulse = map(effectiveAngle, 0, 180, minPulse[channel], maxPulse[channel]);
  int ticks = pulse * 4096 / 20000;
  pwm.setPWM(channel, 0, ticks);
}
