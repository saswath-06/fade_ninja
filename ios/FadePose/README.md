# FadePose — ARKit pose streamer

Minimal iOS app that streams ARKit world pose to `pose_server.py` over UDP.

## Build (Xcode)

1. **File ▸ New ▸ Project ▸ iOS ▸ App**. Product Name `FadePose`, Interface **SwiftUI**, Language **Swift**.
2. Replace the generated `ContentView.swift` with this folder's `ContentView.swift`, and **delete** the generated `FadePoseApp.swift` (ours declares `@main`).
3. Target ▸ **Info** ▸ add keys from `Info-keys.plist.txt`:
   - `NSCameraUsageDescription`
   - `NSLocalNetworkUsageDescription`
4. Signing ▸ pick your team. Run on a physical iPhone (ARKit needs a device).

## Laptop side

```bash
uv run python pose_server.py --host 0.0.0.0 --port 8463
# open http://<laptop-ip>:8464/ for the live sim
```

Put the laptop on the phone's hotspot, type the printed IP into the app, Connect, wait for tracking normal, Start.

## Wire format

See `PHONE_POSE_TDD.md` and golden strings in `tests/test_fadepose_golden.py`.
