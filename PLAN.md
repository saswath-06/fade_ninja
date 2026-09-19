# Robot Barber: Execution Plan (final)

Hardware, software and glove tracks build and test separately, then meet in Phase D.

Rules:
- Every step has a TEST. Do not pass a gate until it passes.
- **HARD GATE** means stop everything if it fails.
- Primary path is **record and replay**. Profile export is secondary.

---

# PHASE 0: INTERFACES (everyone, 20 min, do first)

**0.1 Teach log line** (the primary artifact)
```
t_ms, phi, psi, theta, contact
```
50 Hz, read from the arm's encoders. Glove values are never saved.

**0.2 Commands** (same over serial and BLE)
```
HOME
MOVE <phi> <psi> <theta>        degrees
JOG  <dphi> <dpsi> <dtheta>     deg/sec, teach mode
REC START | REC STOP            returns the log
REPLAY <log>
STOP | STATUS
```

**0.3 Profile** (secondary, for portability across head sizes)
```json
{"name":"low_fade","points":[{"u":0.00,"mm":0.5},{"u":0.45,"mm":1.6},{"u":1.00,"mm":6.0}]}
```

**0.4 Calibration table** (needed for profiles, not for replay)
```json
{"L_mm":25.0,"table":[[0,0.6],[15,1.5],[30,3.1],[45,5.4],[60,8.2]]}
```

**0.5 Head geometry**
```json
{"phi_neckline_deg":8.0,"phi_top_deg":62.0}
```

> **TEST:** all five blocks in a shared file both tracks have open.

---

# PHASE A: HARDWARE

**A1. Geometry lock** **HARD GATE, blocks printing**
Sit a teammate in the chair, clipper against their head in cutting position. Measure head centre out past your hand to the carriage mount, add 20 mm. Measure heel-to-teeth distance, that is `L`.
> **TEST 1:** cardboard at that radius swings ear to back to ear, 20 mm clearance at ears and shoulders.
> **TEST 2:** `L` written down and sent to software. Expect 20 to 30 mm.

**A2. Start printing.** Cradle, rail sections, carriage, clipper mount.
> **TEST:** printer running. Longest item on the critical path.

**A3. Rail and carriage.**
> **TEST:** push by hand through full travel, no binding, no felt slop.

**A4. Spring slide and contact switch.** Minimum 25 mm travel, a head is an oval.
> **TEST:** press 10 times, triggers at the same depth every time.

**A5. Clipper mount.** Pivot axis through the **heel**, not the centre of mass.
> **TEST:** tilt by hand full range, heel stays planted, teeth lift. Lift at max tilt matches `L*sin(theta)` within 15%.

**A6. Cradle on chair.**
> **TEST:** refit the head 5 times, marked reference point repeats within 2 mm.

**A7. CONTACT SWEEP** **HARD GATE**
Carriage powered, clipper off, spring engaged, full arc against a stand-in head.
> **TEST:** heel stays in contact the whole sweep, switch never releases. Watch it.
> **Fix:** more spring travel, softer spring, compliant pad. Nothing downstream matters until this passes.

**A8. Contact across rotation** **HARD GATE**
> **TEST:** holds ear to back to ear.

**A9. E-stop** in series with motor power.
> **TEST:** hit it mid-motion, everything stops.

---

# PHASE B: SOFTWARE (bench, no mechanics)

**B1.1** Steppers and servo move from a test sketch.
> **TEST:** each moves independently.

**B1.2** Homing, back off 5 mm, zero.
> **TEST:** power cycle and home 5 times, same taped mark every time.

**B1.3** `MOVE phi psi theta`.
> **TEST:** `MOVE 30 90 15`, protractor, all within 2 degrees.

**B1.4** Repeatability.
> **TEST:** away and back 10 times, within 1 mm. Drift means lost steps, lower acceleration.

**B1.5** `JOG` rate control.
> **TEST:** `JOG 5 0 0` moves the carriage at a steady 5 deg/sec, `JOG 0 0 0` stops it.

**B1.6** `REC START/STOP`, 50 Hz logging from encoders.
> **TEST:** record 10 seconds of jogging, dump the log, 500 samples with sane values and correct timestamps.

**B1.7** `REPLAY`. Smooth with savgol first, validate every sample against joint limits, refuse rather than clip.
> **TEST 1:** record a jog, replay it with no clipper, watch it retrace the same path.
> **TEST 2:** record, replay, record the replay, compare the two logs. RMS difference under 1 degree on each axis.
> **TEST 3:** feed a log with an out-of-range value, it refuses to run.

**B1.8** BLE for all of the above.
> **TEST:** send a log from a phone, ESP32 echoes it correctly.
> **FALLBACK:** USB serial to a laptop.

**B2. iOS app** (stub the arm)
**B2.1** Recording list: name, duration, date.
> **TEST:** list renders, selection works.
**B2.2** Record and replay buttons over BLE.
> **TEST:** against a laptop pretending to be the ESP32, commands arrive intact.
**B2.3** Trajectory view: plot theta against phi for a recording.
> **TEST:** a recorded ramp shows as a ramp.
**B2.4** Save and load recordings.
> **TEST:** close and reopen, recordings persist.
**B2.5 (secondary)** Profile export and curve editor.
> **TEST:** export a recording to a profile, the curve matches the plot.
**B2.6 (optional)** Photo to profile via a vision model.
> Do not start until D4 passes.

**B3. Log to profile converter** (pure function, no hardware)
> **TEST:** feed a synthetic log with a known linear ramp, the fitted profile matches within 0.2 mm.

---

# PHASE C: TEACH GLOVE (optional, one person)

**C1. Hardware.** IMU on the back of the hand, flex sensor on the index finger, one button, ESP32 on the wrist.
> **TEST:** raw IMU and flex stream at 50 Hz.

**C2. Mapping to JOG.** Rate control with a deadzone. Hand pitch to tilt rate, hand roll to rotation rate, index flex to carriage rate.
> **TEST:** hold the hand still for 10 seconds, all rates read zero. Drift here is what ruins teleop.

**C3. Glove drives the arm.**
> **TEST:** someone who has never used it can drive the carriage to a target and hold it within 2 degrees.

**C4. Teach and replay round trip.**
> **TEST:** glove-teach a deliberate ramp, stop recording, replay it, the arm retraces it.

**CUT LINE:** if C1 to C4 are not done by hour 20, drop the glove. Hand-jog the arm with keyboard commands to author the cut instead. The demo does not depend on the glove.

---

# PHASE D: INTEGRATION

**D1. MANNEQUIN AND WIGS** **HARD GATE**
Someone leaves at store opening. Multiple identical wigs. Pin or glue the caps so they cannot shift under the clipper.
> **TEST:** wig does not move when you drag a clipper across it with the motor off.

**D2. SEAMS FIRST** **HARD GATE**
Jog the arm across three adjacent stripes at constant tilt. Clipper on.
> **TEST:** uniform patch, no visible vertical seams. Reduce the rotation step until they vanish.
> Do not attempt a fade until this passes.

**D3. Teach one cut.** Glove or keyboard jog, clipper on, record the whole thing.
> **TEST:** a recognisable fade on one side, and a saved log.

**D4. Replay on a fresh wig.**
> **TEST:** photograph the taught cut and the replayed cut side by side. They match.
> **This is the product thesis proven rather than asserted.** It is the single most important result in the project.

**D5. Second cut, visibly different.** Teach a high fade against the low fade.
> **TEST:** photograph under demo lighting, the difference is obvious in the photo.

**D6 (secondary). Calibration and profile export.**
Five servo angles, five 30 mm patches, constant speed and preload. Grid laid out before cutting. Comb upright and measure in a ruler photo.
> **TEST 1:** table is monotonic. If it flattens at high tilt, the heel is lifting, go fix A5.
> **TEST 2:** export a recording to a profile and run it on a different-sized head. It scales sensibly.

**D7. Rehearsal.**
> **TEST:** end to end under 4 minutes, works 2 out of 3. Backup video of a successful teach and replay recorded.

---

# CRITICAL PATH

```
0.x interfaces
   |
A1 geometry --> A2 print (long pole, runs all night)
   |                |
   |             A3..A6 assembly
   |                |
   |             A7/A8 CONTACT GATE   <-- hard stop
   |                     \
B1 firmware ------------> D2 seams --> D3 teach --> D4 REPLAY --> D5 --> D7
B2 app     ------------/                                  \
C1..C4 glove (optional) -/                                 D6 profile export (secondary)
```

# THE DEMO

1. Barber teaches a cut with the glove on wig 1. Robot records.
2. Fresh wig goes on. Hit replay. Same cut appears.
3. Load the other recording, visibly different cut.
4. Teammate sits in the chair, motors off, to show it fits a person.

Say the honest parts yourself: mannequin only because human cutting needs force-limited actuators and certification, and replay assumes a fixed head, which a live person would need tracking for.

# WHAT KILLS YOU

1. No wigs, or wigs that shift under the clipper.
2. A7 contact gate fails.
3. Skipping D2. Seams at constant tilt mean a fade looks worse, not better.
4. Spending the night on the glove before D3 works.
