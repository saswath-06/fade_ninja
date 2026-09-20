#!/usr/bin/env bash
# Install Fade Ninja onto whichever iPhone is plugged into this Mac.
#
# The fastest way to get the app onto a friend's phone: plug it in, unlock it,
# run this. Xcode registers the device against the free personal team, builds
# for it, and installs. About two minutes per phone.
#
# The build lasts 7 days, then iOS refuses to launch it until it is signed
# again — free provisioning has no way around that.
set -euo pipefail

cd "$(dirname "$0")/.."

echo "Looking for a connected iPhone..."
LINE=$(xcrun devicectl list devices 2>/dev/null | grep -iE "iphone|ipad" | grep -i "connected" | head -1 || true)
if [ -z "$LINE" ]; then
  echo
  echo "  No unlocked iPhone found."
  echo "  - plug it in with a DATA cable (charge-only cables will not work)"
  echo "  - unlock the screen and tap Trust if asked"
  echo "  - on the phone: Settings > Privacy & Security > Developer Mode > On"
  exit 1
fi

UDID=$(echo "$LINE" | grep -oE "[0-9A-F]{8}-[0-9A-F]{16}" | head -1)
NAME=$(echo "$LINE" | sed -E 's/ +[0-9A-F]{8}-.*//')
echo "Found: $NAME"

echo "Building for this device..."
xcodebuild -project ios/FadePose.xcodeproj -scheme FadePose \
  -configuration Debug -destination "id=$UDID" build \
  -quiet -allowProvisioningUpdates

APP="$HOME/Library/Developer/Xcode/DerivedData/FadePose-ganujpcvenwwnicubvqythbrzmdm/Build/Products/Debug-iphoneos/FadePose.app"
[ -d "$APP" ] || { echo "Build produced no app bundle at $APP"; exit 1; }

echo "Installing..."
xcrun devicectl device install app --device "$UDID" "$APP" >/dev/null

echo
echo "Done. On $NAME:"
echo "  1. Settings > General > VPN & Device Management > trust the developer"
echo "  2. Open Fade Ninja, sign in with a handle"
echo "  3. Robot address: the IP printed by pose_server.py"
echo
echo "This build stops working in 7 days; re-run this script to renew it."
