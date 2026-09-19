"""Clipper model: footprint size and the contact + tilt cutting rule.

h = L * sin(theta) in principle; in practice the measured calibration table is
the ground truth, so the sim cuts with Calibration.theta_to_mm.
"""
from __future__ import annotations

from .calibration import Calibration
from .head import Head

CLIPPER_WIDTH_MM = 45.0   # blade width across the direction of travel
# hair is severed at the teeth line, so the effective footprint along the
# direction of travel is short — a wide value would smear a sloped fade
BLADE_DEPTH_MM = 4.0

# A clipper head is ~45 mm wide; passes should advance 25-30 mm at the scalp
# or you get visible vertical seams (README section 2).
PASS_ADVANCE_MM = 27.0


def apply_cut(head: Head, phi: float, psi: float, theta: float,
              cal: Calibration, clipper_on: bool) -> bool:
    """One tick of cutting. Returns whether the heel is in contact."""
    in_contact = head.contact(phi, psi)
    if clipper_on and in_contact:
        head.cut(phi, psi, float(cal.theta_to_mm(theta)),
                 CLIPPER_WIDTH_MM, BLADE_DEPTH_MM)
    return in_contact
