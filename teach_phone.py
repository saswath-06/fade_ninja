"""Teach a cut with an iPhone, then replay it.

    uv run python teach_phone.py              # teach on the simulated head
    uv run python teach_phone.py --replay out/phone_teach.csv
    uv run python teach_phone.py --hardware /dev/ttyACM0

The phone drives the arm through JOG rates. What gets SAVED is where the arm
actually went, sampled from its encoders at 50 Hz — never the phone's own
numbers. That is why replay reproduces the cut even though the phone drifts:
the recording is the arm's truth, not the hand's estimate.

Press the RECORD button in the app to start and stop a take.
Ctrl-C here to quit.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from fadegpt.head import Head
from fadegpt.interfaces import TICK_S, TeachLog
from fadegpt.pendant import PendantReceiver, RateMap, local_ip
from fadegpt.replay import replay, rms_per_axis, smooth
from fadegpt.rig import Rig

OUT = Path(__file__).parent / "out"


def build_rig(hardware: str | None) -> Rig:
    if hardware:
        from fadegpt.hardware_arm import ArmConfig, HardwareArm
        rig = Rig(head=None)
        rig.arm = HardwareArm.open(ArmConfig(port=hardware))
        return rig
    return Rig(head=Head(), seed=0)


def teach(rig: Rig, port: int, rate_map: RateMap, web: bool = False) -> list[Path]:
    if web:
        from fadegpt.webpendant import WebPendantReceiver
        rx = WebPendantReceiver(port=port)
        print(f"\n  Open this on the iPhone (Safari):\n")
        print(f"      https://{local_ip()}:{port}/\n")
        print("  Safari will warn about the certificate — tap Show Details,")
        print("  then 'visit this website'. That warning is expected: the cert")
        print("  is self-signed, and HTTPS is what unlocks the motion sensors.\n")
    else:
        rx = PendantReceiver(port=port)
        print(f"\n  Listening on udp://{local_ip()}:{port}")
        print("  Enter that address in the Fade Pendant app, then Connect.\n")
    print("  tilt nose up/down  -> clipper tilt (hair length)")
    print("  twist wrist        -> around the head")
    print("  thumb slider       -> up/down the head")
    print("  hold CUT           -> clipper runs")
    print("  RECORD             -> start/stop a take\n")

    if not rig.arm.homed:
        rig.arm.home()
    rig.goto(20.0, 90.0, 20.0)     # a sane starting pose

    saved: list[Path] = []
    was_recording = False
    was_live = False
    next_tick = time.monotonic()
    try:
        while True:
            pkt = rx.latest()

            if pkt is None:                      # dead-man: link quiet
                rig.arm.jog(0, 0, 0)
                rig.clipper_on = False
                if was_live:
                    print("  ! link lost — rates zeroed")
                    was_live = False
            else:
                if not was_live:
                    print("  phone connected")
                    was_live = True
                rig.arm.jog(*rate_map.rates(pkt))
                rig.clipper_on = pkt.cut

                if pkt.rec and not was_recording:
                    rig.recorder.start()
                    print("  ● recording…")
                elif not pkt.rec and was_recording:
                    log = rig.recorder.stop()
                    OUT.mkdir(exist_ok=True)
                    path = OUT / f"phone_teach_{len(saved) + 1}.csv"
                    path.write_text(log.to_csv())
                    saved.append(path)
                    print(f"  ■ saved {len(log)} samples "
                          f"({log.samples[-1].t_ms / 1000:.1f}s) -> {path}")
                was_recording = pkt.rec

            rig.tick()

            if rig.recorder.recording and rig.recorder._t_ms % 1000 == 0:
                enc = rig.encoders
                sys.stdout.write(
                    f"\r  phi {enc[0]:6.1f}  psi {enc[1]:6.1f}  "
                    f"theta {enc[2]:5.1f}  {'CUT' if rig.clipper_on else '   '}  ")
                sys.stdout.flush()

            next_tick += TICK_S
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        print("\n  stopping")
    finally:
        rig.arm.jog(0, 0, 0)
        rig.clipper_on = False
        rig.arm.stop()
        rx.close()
        if rx.received:
            print(f"  link stats: {rx.received} packets, {rx.dropped} dropped")
    return saved


def do_replay(rig: Rig, path: Path) -> None:
    log = TeachLog.from_csv(path.read_text())
    print(f"  replaying {len(log)} samples from {path}")
    rerec = replay(log, rig, record=True)
    rms = rms_per_axis(smooth(log), rerec)
    print("  tracking RMS (deg): " +
          ", ".join(f"{k}={v:.3f}" for k, v in rms.items()))
    if rig.head is not None:
        u, mm = rig.head.length_vs_u()
        valid = ~np.isnan(mm)
        if valid.any():
            print(f"  cut on the head: {mm[valid].min():.1f}–"
                  f"{mm[valid].max():.1f} mm across the fade zone")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=0,
                    help="default 8470 native, 8443 with --web")
    ap.add_argument("--web", action="store_true",
                    help="serve the browser pendant over HTTPS — no Xcode, "
                         "no app install; open the URL in Safari")
    ap.add_argument("--replay", metavar="LOG.csv",
                    help="replay a saved take instead of teaching")
    ap.add_argument("--hardware", metavar="SERIAL_PORT",
                    help="drive the real arm instead of the simulator")
    args = ap.parse_args()

    rig = build_rig(args.hardware)
    if args.replay:
        do_replay(rig, Path(args.replay))
        return 0

    port = args.port or (8443 if args.web else 8470)
    saved = teach(rig, port, RateMap(), web=args.web)
    if saved:
        print(f"\n  {len(saved)} take(s) saved. Replay one with:")
        print(f"    uv run python teach_phone.py --replay {saved[-1]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
