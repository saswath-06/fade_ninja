/*
 * Robot barber — low-level motor board.
 *
 * The Arduino does exactly one job the Pi can't: hardware-timed step pulses.
 * All motion planning (MOVE profiles, JOG, replay tracking, joint limits in
 * degrees) lives on the Pi in Python. The Pi sends velocity commands at
 * 50 Hz; this board generates pulses, integrates step counters, slews the
 * tilt servo, and reports position + switches back.
 *
 * Works with NOTHING attached: AccelStepper counts the steps it generates,
 * so the whole serial link and control chain is testable motor-free.
 *
 * Library: AccelStepper (Library Manager). Board: Uno/Nano (ATmega328).
 *
 * Serial 115200, newline-terminated lines:
 *   HELLO                          -> OK robot-barber-arm v1
 *   V <phi_sps> <psi_sps> <theta_dps>
 *       steppers in steps/sec, servo in deg/sec        -> OK
 *   P                              -> POS <phi_steps> <psi_steps>
 *                                     <theta_cdeg> <phi_es> <psi_es> <contact>
 *   Z                              -> zero step counters, OK
 *   HOME                           -> seek endstops, back off, zero
 *                                     (OK / ERR homing timeout)
 *   STOP                           -> all velocities zero, OK
 *
 * Safety on this board (independent of the Pi):
 *   - velocity watchdog: no V for 250 ms -> stop (a hung Pi can't run away)
 *   - endstop pressed  -> motion toward it is blocked
 *   - the E-STOP IS PHYSICAL, in series with motor power. Not represented
 *     here on purpose; software is never the last line.
 */

#include <AccelStepper.h>
#include <Servo.h>

// ------------------------------------------------------------------ pins
const int PIN_PHI_STEP = 2, PIN_PHI_DIR = 3;
const int PIN_PSI_STEP = 4, PIN_PSI_DIR = 5;
const int PIN_ES_PHI = 6;      // endstop at phi min, NC to GND, INPUT_PULLUP
const int PIN_ES_PSI = 7;      // endstop at psi min
const int PIN_CONTACT = 8;     // spring-slide contact switch
const int PIN_SERVO = 9;       // theta tilt servo (Timer1)
const int PIN_ENABLE = 10;     // stepper driver EN (active LOW), optional

// ------------------------------------------------------------ servo map
// theta joint range 0..60 deg -> pulse range. Calibrate at gate A5.
const float THETA_MIN_DEG = 0.0, THETA_MAX_DEG = 60.0;
const int SERVO_US_AT_MIN = 1000, SERVO_US_AT_MAX = 2000;

const float MAX_SPS = 2000.0;          // step-rate ceiling this board honors
const unsigned long WATCHDOG_MS = 250;
const float HOME_SPS = 300.0;
const long HOME_BACKOFF_STEPS = 160;   // ~5 mm equivalent; tune at B1.2
const unsigned long HOME_TIMEOUT_MS = 20000;

AccelStepper phi(AccelStepper::DRIVER, PIN_PHI_STEP, PIN_PHI_DIR);
AccelStepper psi(AccelStepper::DRIVER, PIN_PSI_STEP, PIN_PSI_DIR);
Servo tilt;

float thetaDeg = 0.0, thetaVel = 0.0;  // servo has no feedback: setpoint IS position
unsigned long lastV = 0, lastServo = 0;
char line[64];
byte lineLen = 0;

bool pressed(int pin) { return digitalRead(pin) == HIGH; }  // NC switch opens

void applyServo() {
  float f = (thetaDeg - THETA_MIN_DEG) / (THETA_MAX_DEG - THETA_MIN_DEG);
  tilt.writeMicroseconds(SERVO_US_AT_MIN +
                         (int)(f * (SERVO_US_AT_MAX - SERVO_US_AT_MIN)));
}

void stopAll() {
  phi.setSpeed(0); psi.setSpeed(0); thetaVel = 0.0;
}

void setup() {
  pinMode(PIN_ES_PHI, INPUT_PULLUP);
  pinMode(PIN_ES_PSI, INPUT_PULLUP);
  pinMode(PIN_CONTACT, INPUT_PULLUP);
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW);       // drivers enabled
  phi.setMaxSpeed(MAX_SPS); psi.setMaxSpeed(MAX_SPS);
  tilt.attach(PIN_SERVO);
  applyServo();
  Serial.begin(115200);
}

void report() {
  Serial.print(F("POS "));
  Serial.print(phi.currentPosition()); Serial.print(' ');
  Serial.print(psi.currentPosition()); Serial.print(' ');
  Serial.print((long)(thetaDeg * 100.0)); Serial.print(' ');
  Serial.print(pressed(PIN_ES_PHI) ? 1 : 0); Serial.print(' ');
  Serial.print(pressed(PIN_ES_PSI) ? 1 : 0); Serial.print(' ');
  Serial.println(pressed(PIN_CONTACT) ? 1 : 0);
}

bool homeAxis(AccelStepper &m, int esPin) {
  unsigned long t0 = millis();
  m.setSpeed(-HOME_SPS);
  while (!pressed(esPin)) {            // seek the endstop
    m.runSpeed();
    if (millis() - t0 > HOME_TIMEOUT_MS) { m.setSpeed(0); return false; }
  }
  m.setSpeed(HOME_SPS);                // back off until it releases, then more
  long releasePos = -1;
  while (millis() - t0 < HOME_TIMEOUT_MS) {
    m.runSpeed();
    if (releasePos < 0 && !pressed(esPin))
      releasePos = m.currentPosition();
    if (releasePos >= 0 &&
        m.currentPosition() >= releasePos + HOME_BACKOFF_STEPS) {
      m.setSpeed(0);
      m.setCurrentPosition(0);
      return true;
    }
  }
  m.setSpeed(0);
  return false;
}

void handle(char *cmd) {
  if (strncmp(cmd, "V ", 2) == 0) {
    float vphi, vpsi, vtheta;
    if (sscanf(cmd + 2, "%f %f %f", &vphi, &vpsi, &vtheta) == 3) {
      phi.setSpeed(constrain(vphi, -MAX_SPS, MAX_SPS));
      psi.setSpeed(constrain(vpsi, -MAX_SPS, MAX_SPS));
      thetaVel = vtheta;
      lastV = millis();
      Serial.println(F("OK"));
    } else Serial.println(F("ERR bad V"));
  } else if (strcmp(cmd, "P") == 0) {
    report();
  } else if (strcmp(cmd, "HELLO") == 0) {
    Serial.println(F("OK robot-barber-arm v1"));
  } else if (strcmp(cmd, "Z") == 0) {
    phi.setCurrentPosition(0); psi.setCurrentPosition(0); thetaDeg = 0.0;
    applyServo();
    Serial.println(F("OK"));
  } else if (strcmp(cmd, "STOP") == 0) {
    stopAll();
    Serial.println(F("OK"));
  } else if (strcmp(cmd, "HOME") == 0) {
    stopAll();
    bool ok = homeAxis(phi, PIN_ES_PHI) && homeAxis(psi, PIN_ES_PSI);
    thetaDeg = 0.0; applyServo();
    Serial.println(ok ? F("OK") : F("ERR homing timeout"));
  } else {
    Serial.println(F("ERR unknown"));
  }
}

void loop() {
  // velocity watchdog: a silent Pi means stop, not coast
  if (millis() - lastV > WATCHDOG_MS) stopAll();

  // block motion into a pressed endstop
  if (pressed(PIN_ES_PHI) && phi.speed() < 0) phi.setSpeed(0);
  if (pressed(PIN_ES_PSI) && psi.speed() < 0) psi.setSpeed(0);

  phi.runSpeed();
  psi.runSpeed();

  // integrate the servo setpoint at ~200 Hz
  unsigned long now = millis();
  if (now - lastServo >= 5) {
    thetaDeg = constrain(thetaDeg + thetaVel * (now - lastServo) / 1000.0,
                         THETA_MIN_DEG, THETA_MAX_DEG);
    lastServo = now;
    applyServo();
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
