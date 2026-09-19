"""Vision autopilot: computer vision controls the arm + trimmer.

The loop, per README section 4 (vision plans and verifies; motion execution
stays deterministic):

  reference photo ──vision──> target profile
        │
        v
  plan passes -> validate -> execute (the same replay pipeline as always)
        ^                                        │
        │                                        v
  corrective passes <──the critic── photograph the cut

The critic closes the loop: it measures the actual cut from photographs,
estimates a gain error (miscalibrated blade) and finds regions that are
still too long (a missed or failed pass), then plans corrective passes for
exactly those. Hair only gets shorter, so vision can fix "too long"
everywhere but can only report "too short".
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import camera, vision
from .head import Head
from .interfaces import Profile
from .profile import pass_centers_deg, passes_log, profile_curve
from .replay import replay
from .rig import Rig


@dataclass
class Report:
    converged: bool
    iterations: int
    gain_history: list = field(default_factory=list)
    max_over_history: list = field(default_factory=list)
    corrective_passes: int = 0
    notes: list = field(default_factory=list)


class VisionAutopilot:
    def __init__(self, length_model: vision.LengthModel,
                 flat_img: np.ndarray, capture=None,
                 baseline_gain: float = 1.0):
        """capture: rig -> image. Defaults to the simulated tripod camera;
        on hardware it becomes the real camera grab.

        baseline_gain: what a CLEAN execution measures as gain. The blade
        footprint leaves sloped regions slightly shorter than commanded, so
        even a perfect cut doesn't read 1.0 — corrections are judged against
        this rehearsal baseline, never against the naive 1.0."""
        self.model = length_model
        self.flat = flat_img
        self.baseline_gain = baseline_gain
        self._capture = capture or (lambda rig, k: camera.render(rig.head,
                                                                 seed=100 + k))
        self._shot = 0

    @classmethod
    def calibrate(cls, patch_lengths=(0.6, 1.5, 3.1, 5.4, 8.2)):
        """Build the intensity->mm model from a ruler-measured patch head and
        a flat photo of an uncut head (both photographed, never read), then
        rehearse one clean cut on a spare wig to learn the execution baseline.
        """
        flat_img = camera.render(camera.ruler_calibration_head([])[0], seed=1)
        cal_head, labels = camera.ruler_calibration_head(patch_lengths)
        cal_img = camera.render(cal_head, seed=2)
        model = vision.fit_length_model(cal_img, flat_img, labels)
        ap = cls(model, flat_img)

        # rehearsal: cut a known ramp on a spare wig, measure how a healthy
        # rig reads — the sim analog of dry-running before the demo (D7)
        ramp = lambda u: 0.6 + 5.4 * np.clip(u, 0, 1)  # noqa: E731
        rig = Rig(head=Head(), seed=7)
        replay(passes_log(ramp, rig.cal, rig.head, pass_centers_deg(rig.head)),
               rig)
        crit = vision.critique(ap.photo(rig), flat_img, model, ramp,
                               rig.head.geometry)
        ap.baseline_gain = crit.gain
        return ap

    def photo(self, rig: Rig) -> np.ndarray:
        self._shot += 1
        return self._capture(rig, self._shot)

    def target_from_photo(self, ref_img: np.ndarray, geometry) -> Profile:
        return vision.curve_from_reference(ref_img, self.flat, self.model,
                                           geometry)

    # ------------------------------------------------------------- the loop

    def cut(self, rig: Rig, target: Profile, max_iters: int = 4,
            over_thresh_mm: float = 0.5) -> Report:
        curve = profile_curve(target)
        rep = Report(converged=False, iterations=0)
        g_est = 1.0  # learned blade gain: measured mm per commanded mm

        def cmd_curve(u):
            lo, hi = rig.cal.mm_range
            return np.clip(np.asarray(curve(u)) / g_est, lo, hi)

        # first cut: full coverage
        plan = passes_log(cmd_curve, rig.cal, rig.head,
                          pass_centers_deg(rig.head))
        replay(plan, rig)

        for it in range(1, max_iters + 1):
            rep.iterations = it
            crit = vision.critique(self.photo(rig), self.flat, self.model,
                                   curve, rig.head.geometry, over_thresh_mm)
            rep.gain_history.append(crit.gain)
            rep.max_over_history.append(crit.max_over_mm)

            # judge against the rehearsal baseline, and only ever correct in
            # the too-long direction — cutting is one-way
            rel = crit.gain / self.baseline_gain
            biased = rel > 1.06
            if rel < 0.94:
                rep.notes.append(f"iter {it}: cut reads {rel:.2f} of baseline "
                                 "— too short, uncorrectable")
            if not biased and not crit.defect_psi:
                rep.converged = True
                break

            if biased:
                g_est *= rel  # e.g. reads 15% long -> command 15% less
                centers = pass_centers_deg(rig.head)
                rep.notes.append(f"iter {it}: gain {crit.gain:.3f} "
                                 f"(baseline {self.baseline_gain:.3f}), "
                                 f"recut all at 1/{g_est:.3f}")
            else:
                centers = np.array(crit.defect_psi)
                rep.notes.append(f"iter {it}: corrective passes at "
                                 f"psi={[round(c) for c in crit.defect_psi]}")
            rep.corrective_passes += len(centers)
            replay(passes_log(cmd_curve, rig.cal, rig.head, centers), rig)

        return rep
