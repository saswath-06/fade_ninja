"""The fixed tripod camera, simulated.

Renders the head's hair field into an IMAGE: pixel brightness rises with hair
length in a nonlinear way the vision code is never told, plus lens vignetting,
blur and sensor noise. Everything downstream of here (vision.py, autopilot.py)
sees only these images — never head.hair — so the control loop is proven
against what a real camera would deliver, not against ground truth.
"""
from __future__ import annotations

import numpy as np
from scipy.ndimage import gaussian_filter

from .head import HAIR_START_MM, Head
from .interfaces import HeadGeometry

# hidden optics: vision must calibrate these away, not import them
_SKIN = 0.22          # brightness of a shaved patch
_GAIN = 0.58
_GAMMA = 0.55         # perceived brightness compresses with length
_VIGNETTE = 0.14
_BLUR_PX = 0.8
_NOISE = 0.012


def render(head: Head, seed: int = 0) -> np.ndarray:
    """One photograph: float image in [0, 1], shape = the (phi, psi) grid."""
    rng = np.random.default_rng(seed)
    length = np.clip(head.hair, 0.0, HAIR_START_MM)
    img = _SKIN + _GAIN * (length / HAIR_START_MM) ** _GAMMA

    h, w = img.shape
    yy, xx = np.mgrid[0:h, 0:w]
    r2 = (((yy - h / 2) / (h / 2)) ** 2 + ((xx - w / 2) / (w / 2)) ** 2) / 2
    img = img * (1.0 - _VIGNETTE * r2)

    img = gaussian_filter(img, _BLUR_PX)
    img = img + rng.normal(0.0, _NOISE, img.shape)
    return np.clip(img, 0.0, 1.0)


# ------------------------------------------------- calibration + reference

def ruler_calibration_head(patch_lengths_mm, geometry: HeadGeometry | None = None,
                           patch_rows: int = 10) -> tuple[Head, list]:
    """The sim analog of the wig whose patches you cut and measure with a
    ruler photo (PLAN.md D6): a head bearing horizontal bands of KNOWN
    lengths. Returns the head and, per patch, (mm, row_slice) — the labels
    come from 'ruler measurement', never from the actuator."""
    head = Head(geometry=geometry)
    labels = []
    r0 = 5
    for mm in patch_lengths_mm:
        rows = slice(r0, r0 + patch_rows)
        head.hair[rows, :] = mm
        labels.append((float(mm), rows))
        r0 += patch_rows + 2
    return head, labels


def reference_head(mm_of_u, geometry: HeadGeometry | None = None) -> Head:
    """A head wearing the TARGET haircut — rendering it produces the
    'reference photo' a client would hand the barber."""
    head = Head(geometry=geometry)
    g = head.geometry
    u = g.u(head.phi_grid)
    in_zone = (u >= 0) & (u <= 1)
    below = head.phi_grid < g.phi_neckline_deg
    head.hair[in_zone, :] = np.asarray(mm_of_u(u[in_zone]))[:, None]
    head.hair[below, :] = float(mm_of_u(0.0))
    return head
