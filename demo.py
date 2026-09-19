"""End-to-end demo in simulation, mirroring the live demo script:

1. "Barber" teaches a low fade on wig 1 (with hand tremor). Robot records.
2. Fresh wig: replay the log. Same cut appears (D4 — the product thesis).
3. A second, visibly different cut (high fade, D5).
4. Export the taught cut to a portable profile and run it on a 10% larger
   head (D6).

Writes figures and artifacts to out/ and prints the numbers the gates ask for.

    uv run python demo.py
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from fadegpt.head import Head
from fadegpt.interfaces import HeadGeometry
from fadegpt.profile import log_to_profile, profile_curve, profile_to_log
from fadegpt.replay import replay, rms_per_axis, smooth
from fadegpt.rig import Rig
from fadegpt.teach import default_fade_mm, high_fade_mm, teach_fade
from fadegpt.viz import (plot_fade_curves, plot_side_by_side, plot_trajectory)

OUT = Path(__file__).parent / "out"


def main():
    OUT.mkdir(exist_ok=True)

    # ---- 1. teach (D3)
    print("1. Teaching a low fade on wig 1 (scripted barber, 8 Hz hand tremor)")
    rig_a = Rig(head=Head(), seed=0)
    log = teach_fade(rig_a, seed=1)
    (OUT / "low_fade.log.csv").write_text(log.to_csv())
    print(f"   recorded {len(log)} samples "
          f"({log.samples[-1].t_ms / 1000:.1f} s) -> out/low_fade.log.csv")
    plot_trajectory(log, smooth(log), str(OUT / "trajectory.png"))

    # ---- 2. replay on a fresh wig (D4)
    print("2. Fresh wig on. REPLAY.")
    rig_b = Rig(head=Head(), seed=99)
    rerec = replay(log, rig_b, record=True)
    rms = rms_per_axis(smooth(log), rerec)
    print("   replay tracking RMS (deg): " +
          ", ".join(f"{k}={v:.3f}" for k, v in rms.items()) +
          "   (gate B1.7: < 1.0)")

    u, mm_a = rig_a.head.length_vs_u()
    _, mm_b = rig_b.head.length_vs_u()
    valid = ~np.isnan(mm_a) & ~np.isnan(mm_b)
    print(f"   taught vs replayed fade, max difference: "
          f"{np.abs(mm_a[valid] - mm_b[valid]).max():.3f} mm  (gate D4: < 0.3)")
    plot_side_by_side(rig_a.head, rig_b.head, str(OUT / "d4_side_by_side.png"))

    # ---- 3. a visibly different second cut (D5)
    print("3. Teaching a high fade — the other recording for the demo.")
    rig_h = Rig(head=Head(), seed=3)
    teach_fade(rig_h, fade_mm=high_fade_mm, seed=4)
    _, mm_h = rig_h.head.length_vs_u()
    plot_fade_curves({"low fade (taught)": (u, mm_a),
                      "high fade": (u, mm_h)},
                     str(OUT / "d5_two_cuts.png"))

    # ---- 4. profile export and transfer (D6)
    print("4. Exporting the low fade to a portable profile...")
    prof = log_to_profile(log, rig_a.cal, HeadGeometry(), name="low_fade")
    (OUT / "low_fade.profile.json").write_text(prof.to_json())
    print(f"   {prof.to_json()}")

    big = Head(scale=1.1)
    rig_c = Rig(head=big, seed=5)
    replay(profile_to_log(prof, rig_c.cal, big), rig_c)
    _, mm_c = rig_c.head.length_vs_u()
    curve = profile_curve(prof)
    v = ~np.isnan(mm_c)
    print(f"   run on a 10% larger head, max error vs profile: "
          f"{np.abs(mm_c[v] - curve(u[v])).max():.3f} mm  (gate D6: < 0.4)")
    plot_fade_curves({"profile target": (u, np.asarray(curve(u))),
                      "cut on larger head": (u, mm_c)},
                     str(OUT / "d6_transfer.png"))

    print(f"\nFigures in {OUT}/: trajectory.png, d4_side_by_side.png, "
          f"d5_two_cuts.png, d6_transfer.png")


if __name__ == "__main__":
    main()
