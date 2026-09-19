# FadePose — ARKit pose streamer

Minimal iOS app that streams ARKit world pose to `pose_server.py` over UDP.

## Build (XcodeGen — preferred)

```bash
brew install xcodegen   # once
cd ios/FadePose
xcodegen generate       # writes FadePose.xcodeproj
open FadePose.xcodeproj
```

Signing ▸ pick your team. Run on a **physical iPhone** (ARKit needs a device).

## Build (by hand)

1. **File ▸ New ▸ Project ▸ iOS ▸ App**. Product Name `FadePose`, Interface **SwiftUI**, Language **Swift**.
2. Replace the generated `ContentView.swift` with this folder's file, and **delete** the generated `FadePoseApp.swift` (ours declares `@main`).
3. Target ▸ **Info** ▸ add keys from `Info-keys.plist.txt`.
4. Signing ▸ pick your team. Run on device.

## Laptop side

```bash
uv run python pose_server.py --host 0.0.0.0 --port 8463
# open http://<laptop-ip>:8464/ for the live sim
```

Put the laptop on the phone's hotspot. Open the app (AR warms up on appear). Type the printed IP, **Connect UDP**, wait until tracking shows **normal**, then **Start**.

## Wire format

```
START <track>          # track 0=normal required
POSE <t_ms> <x> <y> <z> <qw> <qx> <qy> <qz> <track>
STOP
```

See `PHONE_POSE_TDD.md` and `tests/golden_fadepose_lines.txt`.
