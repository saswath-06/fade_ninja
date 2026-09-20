/*
 * Fade Ninja — servo board (REFERENCE).
 *
 * This is the firmware side of the protocol fade_ninja/servo_link.py speaks.
 * If you are writing your own sketch, match the protocol rather than this
 * file; the Python side has FakeServoBoard, which is the same behaviour in
 * Python and is what the tests check against.
 *
 * Serial 115200, newline-terminated, one reply per command:
 *   HELLO                  -> OK fade-ninja-servo v1
 *   J <q1> <q2> <q3> <q4>  -> OK        joint degrees
 *   HOME                   -> OK        ease to the safe pose
 *   RELAX                  -> OK        detach (THE ARM WILL DROP)
 *   P                      -> POS <q1> <q2> <q3> <q4>
 *
 * Three things are duplicated here on purpose, because the laptop can crash
 * mid-command and the horn is the last line:
 *   - joint limits are clamped here as well as on the laptop
 *   - motion eases toward the target instead of snapping to it
 *   - if commands stop arriving, the servos HOLD; they are never detached
 *     automatically, because a limp arm falls under its own weight
 *
 * The e-stop is physical, in series with servo power. Not in this file.
 *
 * Servos need their own supply with a common ground to the Arduino. Four
 * servos stalling on USB power will brown out the board mid-cut.
 */

#include <Servo.h>

// ------------------------------------------------------------------ pins
const int PIN_Q1 = 9;    // base yaw
const int PIN_Q2 = 10;   // shoulder
const int PIN_Q3 = 11;   // elbow
const int PIN_Q4 = 6;    // wrist / cut angle

// ---------------------------------------------- joint limits (degrees)
// these mirror fade_ninja/eezy_ik.py; change both together
const float Q_MIN[4] = { -90.0, 0.0, -135.0, -45.0 };
const float Q_MAX[4] = {  90.0, 90.0,   0.0,  45.0 };
const float SAFE[4]  = {   0.0, 45.0, -90.0,   0.0 };

// ------------------------------------------- per-servo calibration
// MEASURE THESE with tools/servo_calibrate.py, then copy the numbers in.
// The defaults assume a generic 500-2500us servo mounted with no offset and
// no reversal, which is true of approximately no real build.
const int   US_MIN[4] = { 500, 500, 500, 500 };
const int   US_MAX[4] = { 2500, 2500, 2500, 2500 };
const bool  INVERT[4] = { false, false, false, false };
const float OFFSET[4] = { 0.0, 0.0, 0.0, 0.0 };

const float MAX_STEP_DEG = 4.0;      // per update; at 50 Hz that is 200 deg/s
const unsigned long STEP_MS = 20;

Servo servos[4];
const int PINS[4] = { PIN_Q1, PIN_Q2, PIN_Q3, PIN_Q4 };
float current[4];
float target[4];
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
  for (int i = 0; i < 4; i++) servos[i].attach(PINS[i]);
  attached = true;
}

void writeAll() {
  if (!attached) return;
  for (int i = 0; i < 4; i++) servos[i].writeMicroseconds(pulseFor(i, current[i]));
}

void setup() {
  for (int i = 0; i < 4; i++) { current[i] = SAFE[i]; target[i] = SAFE[i]; }
  attachAll();
  writeAll();
  Serial.begin(115200);
}

void handle(char *cmd) {
  if (strcmp(cmd, "HELLO") == 0) {
    Serial.println(F("OK fade-ninja-servo v1"));
  } else if (strcmp(cmd, "HOME") == 0) {
    for (int i = 0; i < 4; i++) target[i] = SAFE[i];
    attachAll();
    Serial.println(F("OK"));
  } else if (strcmp(cmd, "RELAX") == 0) {
    for (int i = 0; i < 4; i++) servos[i].detach();
    attached = false;
    Serial.println(F("OK"));
  } else if (strcmp(cmd, "P") == 0) {
    Serial.print(F("POS "));
    for (int i = 0; i < 4; i++) {
      Serial.print(current[i], 2);
      Serial.print(i < 3 ? ' ' : '\n');
    }
  } else if (strncmp(cmd, "J ", 2) == 0) {
    float v[4];
    if (sscanf(cmd + 2, "%f %f %f %f", &v[0], &v[1], &v[2], &v[3]) == 4) {
      for (int i = 0; i < 4; i++) target[i] = clampJoint(i, v[i]);
      attachAll();
      Serial.println(F("OK"));
    } else {
      Serial.println(F("ERR J takes four angles"));
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
    for (int i = 0; i < 4; i++) {
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
