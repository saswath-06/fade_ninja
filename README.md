# Robot Barber

A barber teaches the robot once by driving it with a glove. The robot records what it did, frame by frame, and replays that cut exactly, forever.

Today a barber keeps your cut in their memory, so "same as last time" is a guess and moving cities loses it. Here it is a file.

---

## 1. How it cuts: the head is the ruler

The machine never measures hair and never holds a gap above the scalp.

**The heel of the clipper always touches the scalp**, pushed in by a spring. That contact point is a pivot. Tilt the clipper and the teeth lift away, and the gap under the teeth is the hair length left.

```
FLAT (short)                 TILTED (longer)

   ====[clipper]====            ====[clipper]
   ~~~~~ scalp ~~~~~            ~~~~~~~~~~~~~~\~~~~
   heel + teeth down            heel down, teeth lifted
   -> 0.5 mm                    -> 6 mm
```

```
h = L * sin(theta)
```

`L` is heel-to-teeth distance, about 25 mm. For 2.4 mm of hair, theta = arcsin(2.4/25) = 5.5 degrees.

**Why contact and not position.** Sensitivity to servo angle error is `L*cos(theta)*dtheta`, which is 0.43 mm per degree. Holding the clipper at a set distance instead makes every millimetre of positioning error a millimetre of length error, with a realistic budget around 5 mm. A fade's whole range is 0.5 to 8 mm, so position control has an error bar wider than the haircut.

This is also what barbers do by hand. They rock the clipper out rather than hovering it.

The spring does this automatically, so during teaching the barber never has to think about pressing in.

---

## 2. Three axes

- **phi**, carriage along the arc rail = height on the head, neckline to top of the fade zone.
- **psi**, rotation of the whole rail = around the head, ear to back to ear, about 180 degrees.
- **theta**, tilt servo = hair length left.

A clipper head is about 45 mm wide, so passes should advance 25 to 30 mm at the scalp, not 45, or you get visible vertical seams.

---

## 3. Teaching and replay

### The glove is a controller, not a sensor

The glove does not measure where the barber's hand is in space. It is a joystick. It sends rate commands to the arm, the barber watches the real clipper on the real head and corrects by eye.

**So the glove's accuracy does not matter.** All precision comes from the arm's encoders. A cheap IMU and a flex sensor are enough.

| glove input | arm axis |
|---|---|
| hand pitch (IMU) | tilt rate |
| hand roll (IMU) | rotation rate |
| index flex sensor | carriage rate |
| thumb button | record on / off |

Rate control, not position control. Tilting your hand sets a speed, not a target. A cheap IMU drifts within seconds, and rate control with a deadzone is forgiving of that.

### What gets saved

At 50 Hz, read off **the arm**, not the glove:

```
t_ms, phi, psi, theta, contact
```

The glove's values are discarded the instant they become a jog command. Saving glove readings would mean saving a drifting estimate of where a hand was, instead of exactly where the clipper went.

### Replay

Step through the log and command each sample at its timestamp. That is the whole mechanism.

**Replay needs no calibration table.** You recorded theta, you replay theta. Millimetre conversion is only needed for displaying real units or for the profile export below.

**Before replay:** run a savgol filter over phi, psi and theta to take out hand jitter, and validate every sample against joint limits. Refuse to run rather than clipping.

### Profile export (secondary)

A raw log replays one cut on one head. Converting it to a profile makes it transfer to a different head size:

```
for samples with contact == true:
    u  = (phi - phi_neckline) / (phi_top - phi_neckline)
    mm = calibration_lookup(theta)
fit a smooth monotonic curve, sample 5 control points
```

```json
{"name":"low_fade","points":[{"u":0.00,"mm":0.5},{"u":0.45,"mm":1.6},{"u":1.00,"mm":6.0}]}
```

About ten lines of code, and it is the difference between a macro recorder and "your haircut is a file."

---

## 4. Where machine learning sits

**Curve fitting** from a teach log to a portable profile.

**Reference photo to profile.** A vision model reads a picture of a haircut and returns structured parameters: guideline height, blend span, bottom and top lengths.

**The critic.** After a pass, a fixed tripod camera photographs the region and a vision model flags uneven areas, scheduling a corrective pass.

**Not the cutting.** Cutting is replay plus contact control, deliberately deterministic. A machine operating near a head should be predictable, and that is the right answer when a judge asks.

---

## 5. Calibration (needed for profiles, not for replay)

Five servo angles, five 30 mm test patches on a spare wig, constant speed and spring preload. Lay the grid out first, you only get one cut per patch.

Measuring: comb upright, side-on macro photo with a steel ruler against the scalp in frame, measure in the photo. Calipers compress hair and lie.

```json
{"L_mm":25.0,"table":[[0,0.6],[15,1.5],[30,3.1],[45,5.4],[60,8.2]]}
```

Must be monotonic. If it flattens at high tilt, the heel is lifting off and the mount geometry is wrong.

---

## 6. Safety

- Mannequin and wigs only. No human cutting.
- Physical e-stop in series with motor power.
- The spring slide limits how hard the clipper can press regardless of software.
- Head is cradled, never clamped. On a human the head would rest, not be restrained, because the ability to pull away is the most reliable safety layer in the system.
- Test with the clipper motor off first. Vibration is the enemy of everything.
- Replay validates against joint limits before moving.

---

## 7. Honest limits

- Replay assumes the head is in the same place it was during teaching. A fixed mannequin and identical wigs make that true here. It would not hold on a live person without tracking.
- Reproduces the fade, not line-ups, neckline edges, or scissor work on top. It is a fade machine, not a general barber.
- Tilt range caps maximum length. Beyond that needs guard changes, out of scope.
- Accuracy numbers here are estimates until calibration and dry runs give real ones.

---

## 8. Software (simulation-first)

The `fadegpt/` package implements the whole software side against a
physics-lite simulator, so every Phase B gate in PLAN.md is testable before
any hardware exists, and the same suite becomes the spec for the ESP32 port.

```
uv sync                     # install (Python 3.11+, numpy/scipy/matplotlib)
uv run pytest               # every PLAN.md software gate, plus A7/D2/D4/D6 analogs
uv run python demo.py       # teach -> replay -> profile transfer; figures in out/
uv run python server.py     # virtual ESP32 on TCP :8462 (the B2.2 app test target)
```

Layout: `interfaces.py` (the five Phase 0 blocks), `protocol.py` (0.2
commands), `arm.py` + `rig.py` (virtual arm and bench), `head.py` +
`cutting.py` (ellipsoid scalp, spring contact, hair field), `recorder.py`,
`replay.py` (savgol -> validate-and-refuse -> track), `teach.py` (scripted
barber with 8 Hz hand tremor), `calibration.py` + `profile.py` (log ->
portable profile and back), `viz.py`.

Simulated results (deterministic, seeded): replay retraces the taught path
with < 0.12 deg RMS per axis; the replayed fade matches the taught fade
within 0.03 mm; seams vanish once pass advance drops below ~43 mm for the
45 mm blade; a profile exported from one head lands within 0.17 mm on a
head 10% larger.

### The teach glove is an iPhone (`phone/`, `pendant.py`)

The glove of section 3 is a phone. It streams gravity-referenced pitch and
roll over UDP at 50 Hz; `pendant.py` maps those to `JOG` rates (tilt ->
clipper tilt, twist -> around the head, thumb slider -> height) and the
recorder logs **the arm's encoders**, never the phone's numbers. That is the
section 3 rule kept intact: the phone is a controller, so its drift is
absorbed by rate control and the barber's eye, and the saved log is exactly
where the clipper went.

No camera and no ARKit **on this rate pendant** — an IMU cannot integrate to
position without drifting metres in seconds, and rate control does not need
position. Absolute pose teleop (ARKit → IK → EEZYbotARM) is a separate path;
see `PHONE_POSE_TDD.md` and `ios/FadePose/`.

```
uv run python teach_phone.py                        # teach on the sim head
uv run python teach_phone.py --replay out/phone_teach_1.csv
uv run python teach_phone.py --hardware /dev/ttyACM0
```

Build instructions for the rate pendant are in `phone/README.md`. Two independent
watchdogs stop the arm if the phone or the laptop goes quiet (250 ms); the
e-stop stays physical.

### ARKit pose teleop → EEZYbotARM (`pose_server.py`, `ios/FadePose/`)

Absolute phone position/tilt over UDP `:8463` → IK → joints. Live canvas at
HTTP `:8464`. Rate pendant on `:8470` is unchanged.

```
uv run python pose_server.py --host 0.0.0.0
uv run python tools/fake_phone.py --circle 0.05   # no phone needed
# open http://<laptop-ip>:8464/
```

Build the ARKit app from `ios/FadePose/README.md`. Gates: `PHONE_POSE_TDD.md`.

### ARKit pose teleop + 3D viewer (`pose_teleop.py`, `teleop_sim.py`)

The FadePose app's 6DoF pose drives the arm so that **the clipper goes where
your hand goes**: move the phone 10 cm and the tip travels 10 cm across the
head, in the same direction. Hand movement is resolved into the two
directions the machine can move and converted by arc length (phi on the rail
arc, psi along the latitude circle), so it is 1:1 rather than one phone axis
wired to one joint. Movement toward or away from the head is discarded — the
rail sets stand-off and the spring slide absorbs the rest.

On engage the hand frame is auto-aligned so "push forward" means "push at the
head" regardless of which way you are standing or how the phone was held when
ARKit started.

```
uv run python teleop_sim.py        # then open the printed http://127.0.0.1:8465/
```

The app's Start/Stop is a **clutch**. Engaging anchors the phone pose to the
arm's current joints, so no axis jumps; releasing lets you reposition your
hand without the arm following. That bounds ARKit drift to one stroke rather
than a session — the reason absolute position is usable here at all. Losing
tracking holds position rather than extrapolating, and a quiet link stops the
arm after 250 ms. Live teleop clamps at the joint limits (replay still
refuses, since a bad log should never run).

`tools/arm_sim.html` is the viewer: the rail, carriage and clipper in 3D with
joint bars, limit flags, a clipper-tip trail and link/tracking/clutch status.
Each engaged stroke is recorded from the arm's encoders, so a session leaves
ordinary teach logs that replay with `teach_phone.py --replay`.

### Hardware path (Pi 4 + Arduino)

The Pi runs this Python stack unchanged; the Arduino
(`firmware/robot_barber_arm/`, needs the AccelStepper library) only
generates step pulses and reports counters + switches over USB serial.
`fadegpt/hardware_arm.py` is a drop-in for the simulated arm using the same
tested motion planner; `tests/test_hardware_arm.py` runs the Phase B gates
against an emulated board, and works motor-free on the real one because
step counters advance whether or not motors are wired.

```
uv sync --extra hw
uv run python -m fadegpt.hwcheck /dev/ttyACM0    # bench link check, no motors
uv run python server.py --hardware /dev/ttyACM0  # real arm behind the same TCP protocol
```
