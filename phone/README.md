# Fade Pendant — iPhone teach glove

The phone replaces the teach glove. It streams orientation over UDP at 50 Hz;
the laptop turns that into `JOG` rates and records where the **arm** went.

No camera and no ARKit: gravity-referenced pitch and roll from CoreMotion are
drift-free, work in any lighting, and need no permissions beyond local
network. Position tracking would require the camera (an IMU alone cannot
integrate to position without drifting metres in seconds) and the project
does not need it — README section 3 specifies rate control.

## Build it (15 minutes)

1. Xcode → **File ▸ New ▸ Project ▸ iOS ▸ App**. Product Name `FadePendant`,
   Interface **SwiftUI**, Language **Swift**.
2. Replace the generated `ContentView.swift` with this folder's
   `ContentView.swift`, and **delete the generated `FadePendantApp.swift`**
   (this file declares its own `@main`).
3. Target ▸ **Info** ▸ add key **`NSLocalNetworkUsageDescription`** with a
   value like `Sends clipper motion to the barber robot.` iOS 14+ refuses
   LAN traffic without it, silently.
4. Target ▸ **Signing & Capabilities** → pick your Apple ID team. A free
   account works; the build lasts 7 days.
5. Plug the phone in, select it as the run destination, press ▶.
   First run: Settings ▸ General ▸ VPN & Device Management ▸ trust your cert.

## Network

Campus and hotel WiFi usually block device-to-device traffic (client
isolation, 802.1X). The reliable answer is to make the **phone a hotspot**
and join the laptop to it — then both ends are on the same small network.

Start the laptop side first; it prints the address to type into the app:

```
uv run python teach_phone.py
  Listening on udp://172.20.10.2:8470
```

## Using it

| control | axis |
|---|---|
| tilt nose up/down | theta — clipper tilt, sets hair length |
| twist wrist | psi — around the head |
| thumb slider (springs back) | phi — up and down the head |
| hold CUT | clipper motor runs |
| RECORD | start / stop a take |

**Recenter** first: hold the phone the way you'd hold clippers, tap it, and
that pose becomes neutral. Grip and handedness stop mattering.

The slider springs back to centre because these are *rates*, not positions —
tilt sets a speed, and you correct by watching the clipper, exactly as the
glove was specified to work.

## Safety

Two independent watchdogs, both outside the phone:

- `pendant.py` zeroes all rates if no packet arrives for 250 ms, so a locked
  or crashed phone stops the arm rather than leaving it coasting.
- The Arduino firmware does the same if the laptop goes quiet.

The e-stop stays physical, in series with motor power. Software is never the
last line.

## Tuning

Mapping lives in `RateMap` in `fadegpt/pendant.py` — deadzone, full-scale
tilt, per-axis speeds and expo curve. Tune there and re-run; no app rebuild.
