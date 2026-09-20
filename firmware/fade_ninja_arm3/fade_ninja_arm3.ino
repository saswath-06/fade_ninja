/*
 * Fade Ninja — 3-DOF arm servo board.
 *
 * The arm that actually got built: base rotation, arm pitch, razor tilt.
 * This is the firmware side of fade_ninja/servo_link.py's Servo3Driver.
 * FakeServo3Board is the same behaviour in Python and is what the tests
 * check against, so match the protocol rather than this file.
 *
 * Serial 115200, newline-terminated, one reply per command:
 *   HELLO             -> OK fade-ninja-servo v1
 *   J <q1> <q2> <q3>  -> OK          joint degrees
 *   HOME              -> OK          ease to the safe pose
 *   RELAX             -> OK          detach (THE ARM WILL DROP)
 *   P                 -> POS <q1> <q2> <q3>
 *   US <i> <us>       -> OK          raw pulse, for tools/servo_calibrate.py
 *
 * Three things are duplicated here on purpose, because the laptop can crash
 * mid-command and the horn is the last line of defence:
 *   - joint limits are clamped here as well as on the laptop
 *   - motion eases toward the target instead of snapping to it
 *   - if commands stop arriving, the servos HOLD; they are never detached
 *     automatically, because a limp arm falls under its own weight
 *
 * The e-stop is physical, in series with servo power. Not in this file.
 *
 * Servos need their own supply with a common ground to the Arduino. Three
 * servos stalling on USB power will brown out the board mid-cut.
 */

#include <Servo.h>

const int N = 3;

// ------------------------------------------------------------------ pins
// As wired on the build: D2 base rotation, D3 shaft up/down, D4 forward/back.
// Any PWM-capable pin works; these are the ones the harness plugs into.
const int PINS[N] = { 2, 3, 4 };

// ---------------------------------------------- joint limits (degrees)
// these mirror ArmSpec in fade_ninja/arm3dof.py; change both together
const float Q_MIN[N] = { -90.0, -20.0, -60.0 };
const float Q_MAX[N] = {  90.0,  80.0,  60.0 };
const float SAFE[N]  = {   0.0,  30.0,   0.0 };

// ------------------------------------------- per-servo calibration
// MEASURE THESE with tools/servo_calibrate.py, then copy the numbers in.
// The defaults assume a generic 500-2500us servo mounted with no offset and
// no reversal, which is true of approximately no real build.
const int   US_MIN[N] = { 500, 500, 500 };
const int   US_MAX[N] = { 2500, 2500, 2500 };
const bool  INVERT[N] = { false, false, false };
const float OFFSET[N] = { 0.0, 0.0, 0.0 };

const float MAX_STEP_DEG = 4.0;      // per update; at 50 Hz that is 200 deg/s
const unsigned long STEP_MS = 20;

Servo servos[N];
float current[N];
float target[N];
int   override_us[N];                // >0 while US is holding a raw pulse
bool attached = false;
unsigned long lastStep = 0;

char line[64];
byte lineLen = 0;

float clampJoint(int i, float v) {
  if (v < Q_MIN[i]) return Q_MIN[i];
  if (v > Q_MAX[i]) return Q_MAX[i];
  return v;
}

int pulseFor(int i, float deg) {
  float d = deg + OFFSET[i];
  float t = (d - Q_MIN[i]) / (Q_MAX[i] - Q_MIN[i]);
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  if (INVERT[i]) t = 1.0 - t;
  return (int)(US_MIN[i] + t * (US_MAX[i] - US_MIN[i]) + 0.5);
}

void attachAll() {
  if (attached) return;
  for (int i = 0; i < N; i++) servos[i].attach(PINS[i]);
  attached = true;
}

void writeAll() {
  if (!attached) return;
  for (int i = 0; i < N; i++) {
    servos[i].writeMicroseconds(
        override_us[i] > 0 ? override_us[i] : pulseFor(i, current[i]));
  }
}

void setup() {
  for (int i = 0; i < N; i++) {
    current[i] = SAFE[i];
    target[i] = SAFE[i];
    override_us[i] = 0;
  }
  attachAll();
  writeAll();
  Serial.begin(115200);
}

void handle(char *cmd) {
  if (strcmp(cmd, "HELLO") == 0) {
    Serial.println(F("OK fade-ninja-servo v1"));
  } else if (strcmp(cmd, "HOME") == 0) {
    for (int i = 0; i < N; i++) { target[i] = SAFE[i]; override_us[i] = 0; }
    attachAll();
    Serial.println(F("OK"));
  } else if (strcmp(cmd, "RELAX") == 0) {
    for (int i = 0; i < N; i++) servos[i].detach();
    attached = false;
    Serial.println(F("OK"));
  } else if (strcmp(cmd, "P") == 0) {
    Serial.print(F("POS "));
    for (int i = 0; i < N; i++) {
      Serial.print(current[i], 2);
      Serial.print(i < N - 1 ? ' ' : '\n');
    }
  } else if (strncmp(cmd, "J ", 2) == 0) {
    float v[N];
    if (sscanf(cmd + 2, "%f %f %f", &v[0], &v[1], &v[2]) == N) {
      for (int i = 0; i < N; i++) {
        target[i] = clampJoint(i, v[i]);
        override_us[i] = 0;          // a real joint command ends calibration
      }
      attachAll();
      Serial.println(F("OK"));
    } else {
      Serial.println(F("ERR J takes three angles"));
    }
  } else if (strncmp(cmd, "US ", 3) == 0) {
    // raw pulse, for calibration only: bypasses the angle mapping so the
    // horn can be lined up with the angle the laptop thinks it is commanding
    int idx, us;
    if (sscanf(cmd + 3, "%d %d", &idx, &us) == 2 && idx >= 0 && idx < N) {
      if (us < 400) us = 400;
      if (us > 2600) us = 2600;
      override_us[idx] = us;
      attachAll();
      Serial.println(F("OK"));
    } else {
      Serial.println(F("ERR US <joint 0-2> <microseconds>"));
    }
  } else {
    Serial.println(F("ERR unknown"));
  }
}

void loop() {
  // ease toward the target; never jump, whatever the laptop asked for
  unsigned long now = millis();
  if (now - lastStep >= STEP_MS) {
    lastStep = now;
    for (int i = 0; i < N; i++) {
      float d = target[i] - current[i];
      if (d > MAX_STEP_DEG) d = MAX_STEP_DEG;
      if (d < -MAX_STEP_DEG) d = -MAX_STEP_DEG;
      current[i] += d;
    }
    writeAll();
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) { line[lineLen] = 0; handle(line); lineLen = 0; }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }
}
