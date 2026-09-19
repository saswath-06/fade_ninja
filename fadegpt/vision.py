"""Computer vision for the robot barber: images in, lengths and defects out.

Pipeline (classical CV, no ground truth access):
1. flat-field: divide out lens vignetting using a photo of the UNCUT head
   (uniform hair = the flat reference).
2. intensity -> mm: a monotonic curve fitted to ruler-measured calibration
   patches (the sim analog of cutting test patches on a wig and measuring
   them in a ruler photo).
3. curve extraction: reference photo -> target fade profile.
4. the critic: estimated length map vs target -> defect regions + gain error.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.ndimage import gaussian_filter

from .head import HAIR_START_MM
from .interfaces import HeadGeometry, Profile, ProfilePoint


@dataclass
class LengthModel:
    """Monotonic map from flat-fielded intensity to hair length."""
    intensities: np.ndarray
    lengths: np.ndarray

    def __call__(self, img: np.ndarray) -> np.ndarray:
        i = np.clip(img, self.intensities[0], self.intensities[-1])
        return PchipInterpolator(self.intensities, self.lengths)(i)


def flat_field(img: np.ndarray, flat_img: np.ndarray) -> np.ndarray:
    """Remove vignetting using a photo of a uniformly-covered head."""
    flat = gaussian_filter(flat_img, 6.0)
    return img * (flat.mean() / np.maximum(flat, 1e-3))


def fit_length_model(cal_img: np.ndarray, flat_img: np.ndarray,
                     labels) -> LengthModel:
    """labels: [(mm, row_slice), ...] from the ruler-measured patch head.
    Adds the uncut length as a free anchor (the flat photo is all uncut)."""
    corrected = flat_field(cal_img, flat_img)
    pts = [(float(np.median(corrected[rows, 30:-30])), mm)
           for mm, rows in labels]
    flatc = flat_field(flat_img, flat_img)
    pts.append((float(np.median(flatc[20:-20, 30:-30])), HAIR_START_MM))
    pts.sort()
    ii = np.array([p[0] for p in pts])
    mm = np.array([p[1] for p in pts])
    if np.any(np.diff(ii) <= 0) or np.any(np.diff(mm) <= 0):
        raise ValueError("calibration patches are not monotonic in intensity")
    return LengthModel(ii, mm)


def length_map(img: np.ndarray, flat_img: np.ndarray,
               model: LengthModel) -> np.ndarray:
    return model(flat_field(img, flat_img))


# ---------------------------------------------------------- target extraction

def curve_from_reference(ref_img: np.ndarray, flat_img: np.ndarray,
                         model: LengthModel, geometry: HeadGeometry,
                         n_points: int = 5, name: str = "from_photo") -> Profile:
    """Reference photo of a finished cut -> target fade profile."""
    est = length_map(ref_img, flat_img, model)
    phi_idx = np.arange(est.shape[0])
    u = geometry.u(phi_idx)  # grid rows are 1 deg apart, row i = phi i
    # stop short of u=1: the long, uncut hair above the fade zone blurs into
    # the top rows of the photo and would inflate the target there
    band = (u >= 0.0) & (u <= 0.92)
    per_row = np.median(est[band, 20:-20], axis=1)
    uu = u[band]
    # a fade never shortens going up: enforce monotonicity
    per_row = np.maximum.accumulate(per_row)
    us = np.linspace(0.0, 1.0, n_points)
    mms = np.interp(us, uu, per_row)
    # extend the trimmed tail linearly from the clean upper section
    tail = (uu >= 0.6)
    slope, icept = np.polyfit(uu[tail], per_row[tail], 1)
    high = us > uu[-1]
    mms[high] = np.maximum(mms[high], slope * us[high] + icept)
    return Profile(name, [ProfilePoint(float(a), float(b))
                          for a, b in zip(us, mms)])


# ------------------------------------------------------------------ the critic

@dataclass
class Critique:
    gain: float                 # median measured/target (1.0 = calibrated)
    max_over_mm: float          # worst too-long error in the band
    defect_psi: list            # psi column centers needing another pass
    err_map: np.ndarray         # measured - target, band rows only


def critique(img: np.ndarray, flat_img: np.ndarray, model: LengthModel,
             mm_of_u, geometry: HeadGeometry,
             over_thresh_mm: float = 0.5) -> Critique:
    """Compare a photo of the cut against the target curve."""
    est = length_map(img, flat_img, model)
    phi_idx = np.arange(est.shape[0])
    u = geometry.u(phi_idx)
    band = (u >= 0.05) & (u <= 0.95)
    target = np.asarray(mm_of_u(u[band]))[:, None]
    err = est[band, :] - target

    meaningful = target[:, 0] >= 1.2  # relative gain needs non-tiny targets
    gain = float(np.median(est[band, :][meaningful, 20:-20]
                           / target[meaningful])) if meaningful.any() else 1.0

    col_err = np.median(err, axis=0)
    smooth_col = gaussian_filter(col_err, 2.0)
    over_cols = np.flatnonzero(smooth_col > over_thresh_mm)

    centers = []
    if over_cols.size:
        splits = np.split(over_cols, np.flatnonzero(np.diff(over_cols) > 3) + 1)
        for grp in splits:
            if len(grp) >= 4:  # ignore single-column noise
                lo, hi = grp[0], grp[-1]
                step = 15  # deg; pass footprint is ~30 deg wide
                centers.extend(np.arange(lo + step / 2, hi + 1, step).tolist()
                               or [float((lo + hi) / 2)])
    return Critique(gain=gain,
                    max_over_mm=float(np.percentile(err, 99.5)),
                    defect_psi=[float(c) for c in centers],
                    err_map=err)
