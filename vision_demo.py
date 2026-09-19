"""Vision-controlled cutting, proven in sim with a sabotaged rig:

1. Vision calibrates itself (ruler patches + a rehearsal cut on a spare wig).
2. It reads the TARGET from a reference photo — never from code.
3. It cuts on a rig with BOTH faults at once: a blade that leaves 18% too
   much hair, and a clipper that silently fails over one stripe during the
   first pass set.
4. The critic photographs the head, measures, and schedules corrections
   until the cut matches the photo.

Figures land in out/. Ground truth is only used for the final judgement.

    uv run python vision_demo.py
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from fadegpt import camera
from fadegpt.autopilot import VisionAutopilot
from fadegpt.head import Head
from fadegpt.interfaces import HeadGeometry
from fadegpt.profile import profile_curve
from fadegpt.teach import default_fade_mm
from fadegpt.viz import INK, INK_2, SEQ, SERIES_1, SERIES_2, SURFACE, _style
from tests.test_vision import SabotagedRig, biased_cal, true_error_vs_target

OUT = Path(__file__).parent / "out"


def photo_panel(ax, img, title):
    ax.imshow(img, origin="lower", cmap="gray", vmin=0, vmax=1, aspect="auto")
    _style(ax, title, "psi (px)", "phi (px)")
    ax.grid(False)


def main():
    OUT.mkdir(exist_ok=True)
    print("1. Vision self-calibration (ruler patches + rehearsal cut)...")
    ap = VisionAutopilot.calibrate()
    print(f"   execution baseline gain: {ap.baseline_gain:.3f}")

    print("2. Reading the target fade from a reference photo...")
    ref_img = camera.render(camera.reference_head(default_fade_mm), seed=12)
    target = ap.target_from_photo(ref_img, HeadGeometry())
    print(f"   extracted: {target.to_json()}")

    print("3. Cutting on a DOUBLY sabotaged rig "
          "(blade +18% long, one stripe silently missed)...")
    rig = SabotagedRig(head=Head(), seed=0, physical_cal=biased_cal(1.18))
    first_cut_img = {}
    shots = []
    orig_capture = ap._capture

    def capture_and_keep(r, k):
        img = orig_capture(r, k)
        shots.append(img)
        return img

    ap._capture = capture_and_keep
    rep = ap.cut(rig, target, max_iters=5)
    final_img = camera.render(rig.head, seed=999)

    print("   critic's log:")
    for n in rep.notes:
        print(f"     - {n}")
    err = true_error_vs_target(rig.head, profile_curve(target))
    print(f"4. Ground-truth verdict: worst too-long error {err:.2f} mm "
          f"({'PASS' if err < 0.5 else 'FAIL'}, threshold 0.5)")
    print(f"   converged={rep.converged} after {rep.iterations} critiques, "
          f"{rep.corrective_passes} corrective passes")

    # ---- figure: the story in four photographs + the curves
    fig, axes = plt.subplots(1, 4, figsize=(16, 3.6), facecolor=SURFACE,
                             layout="constrained")
    photo_panel(axes[0], ref_img, "Reference photo (the order)")
    photo_panel(axes[1], shots[0] if shots else final_img,
                "After 1st pass set — faults visible")
    photo_panel(axes[2], final_img, "After vision corrections")

    u = np.linspace(0.05, 0.95, 30)
    curve = profile_curve(target)
    uu, mm = rig.head.length_vs_u()
    valid = ~np.isnan(mm)
    axes[3].plot(u, curve(u), color=INK_2, linestyle="--", linewidth=2,
                 label="target (from photo)")
    axes[3].plot(uu[valid], mm[valid], color=SERIES_1, linewidth=2,
                 label="achieved (ground truth)")
    _style(axes[3], "Verdict", "u — height", "hair length (mm)")
    axes[3].legend(frameon=False, fontsize=8, labelcolor=INK_2)
    fig.savefig(OUT / "vision_loop.png", dpi=150)
    print(f"\nFigure: {OUT}/vision_loop.png")


if __name__ == "__main__":
    main()
