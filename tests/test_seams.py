"""D2 analog: adjacent constant-tilt stripes must merge into a uniform patch.

A clipper head is ~45 mm wide; passes advancing more than that leave visible
ridges of uncut hair. The sim finds the largest seam-free advance.
"""
import numpy as np

from fadegpt.head import HAIR_START_MM, Head
from fadegpt.interfaces import TICK_S
from fadegpt.rig import Rig

THETA = 30.0  # constant tilt -> 3.1 mm everywhere by the default table
SWEEP_RATE = 25.0


def cut_stripes(advance_mm: float, n_stripes: int = 3) -> tuple[Rig, list]:
    rig = Rig(head=Head())
    g = rig.head.geometry
    rho = float(rig.head.horizontal_radius(
        0.5 * (g.phi_neckline_deg + g.phi_top_deg), 90.0))
    step_deg = np.degrees(advance_mm / rho)
    centers = [90.0 + (k - (n_stripes - 1) / 2) * step_deg
               for k in range(n_stripes)]

    rig.arm.home()
    for psi_c in centers:
        rig.goto(g.phi_neckline_deg, psi_c, THETA)
        rig.clipper_on = True
        n = int(round((g.phi_top_deg - g.phi_neckline_deg) / SWEEP_RATE / TICK_S))
        for i in range(n + 1):
            phi = g.phi_neckline_deg + (g.phi_top_deg - g.phi_neckline_deg) * i / n
            rig.arm.track(phi, psi_c, THETA)
            rig.tick()
        rig.clipper_on = False
    return rig, centers


def interior_band(rig: Rig, centers: list) -> np.ndarray:
    """Hair in the region the stripes were supposed to cover, mid 80% of phi."""
    g = rig.head.geometry
    span = g.phi_top_deg - g.phi_neckline_deg
    pm = (rig.head.phi_grid >= g.phi_neckline_deg + 0.1 * span) & \
         (rig.head.phi_grid <= g.phi_top_deg - 0.1 * span)
    sm = (rig.head.psi_grid >= centers[0]) & (rig.head.psi_grid <= centers[-1])
    return rig.head.hair[np.ix_(pm, sm)]


def test_d2_27mm_advance_is_uniform():
    rig, centers = cut_stripes(27.0)
    band = interior_band(rig, centers)
    target = float(rig.cal.theta_to_mm(THETA))
    assert np.all(band < HAIR_START_MM), "uncut ridge between stripes"
    assert np.allclose(band, target, atol=0.1), \
        f"patch not uniform: {band.min():.2f}..{band.max():.2f} mm"


def test_d2_55mm_advance_leaves_seams():
    rig, centers = cut_stripes(55.0)
    band = interior_band(rig, centers)
    assert np.any(band == HAIR_START_MM), \
        "advancing 55 mm with a 45 mm blade should leave visible seams"


def max_seam_free_advance() -> float:
    """The experiment D2 asks for: reduce the step until seams vanish."""
    best = 0.0
    for advance in np.arange(25.0, 60.0, 2.5):
        rig, centers = cut_stripes(float(advance))
        if np.all(interior_band(rig, centers) < HAIR_START_MM):
            best = float(advance)
        else:
            break
    return best


def test_d2_max_advance_close_to_blade_width():
    best = max_seam_free_advance()
    assert 35.0 <= best <= 47.5, \
        f"max seam-free advance {best} mm should be just under the 45 mm blade"
