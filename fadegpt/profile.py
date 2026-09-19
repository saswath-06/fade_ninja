"""B3: teach log -> portable profile, and profile -> trajectory for any head.

A raw log replays one cut on one head. A profile — hair length as a function
of normalized height u — transfers to a different head size.
"""
from __future__ import annotations

import numpy as np
from scipy.interpolate import PchipInterpolator

from .calibration import Calibration
from .cutting import PASS_ADVANCE_MM
from .head import Head
from .interfaces import (TICK_MS, TICK_S, HeadGeometry, JOINT_LIMITS, Profile,
                         ProfilePoint, TeachLog)
from .teach import PHI_LEAD_DEG, SWEEP_RATE_DEG_S, pass_centers_deg


class ProfileError(ValueError):
    pass


def log_to_profile(log: TeachLog, cal: Calibration, geom: HeadGeometry,
                   name: str = "profile", n_points: int = 5,
                   n_bins: int = 24) -> Profile:
    """For contact samples in the fade zone: u from phi, mm from theta via the
    calibration table, then a smooth monotonic curve sampled at n control
    points."""
    a = log.arrays()
    u_all = np.asarray(geom.u(a["phi"]))
    m = a["contact"] & (u_all >= -0.02) & (u_all <= 1.02)
    if m.sum() < n_bins:
        raise ProfileError("not enough in-zone contact samples to fit a profile")
    u = np.clip(u_all[m], 0.0, 1.0)
    mm = np.asarray(cal.theta_to_mm(a["theta"][m]))

    edges = np.linspace(0.0, 1.0, n_bins + 1)
    idx = np.clip(np.digitize(u, edges) - 1, 0, n_bins - 1)
    centers, means = [], []
    for i in range(n_bins):
        sel = idx == i
        if sel.any():
            centers.append((edges[i] + edges[i + 1]) / 2)
            # what remains on the head at this height is the SHORTEST cut
            # made there — transit passes at longer settings cut nothing.
            # A low percentile is that minimum, robust to jitter spikes.
            means.append(np.percentile(mm[sel], 20))
    centers, means = np.array(centers), np.array(means)

    # a fade never gets shorter as you go up: enforce monotonic non-decreasing
    dense_u = np.linspace(0.0, 1.0, 101)
    dense = np.interp(dense_u, centers, means)
    dense = np.maximum.accumulate(dense)

    us = np.linspace(0.0, 1.0, n_points)
    mms = np.interp(us, dense_u, dense)
    return Profile(name, [ProfilePoint(float(uu), float(vv))
                          for uu, vv in zip(us, mms)])


def profile_curve(profile: Profile):
    """Smooth monotone u -> mm interpolator (clamped outside [0, 1])."""
    us = np.array([p.u for p in profile.points])
    mms = np.array([p.mm for p in profile.points])
    interp = PchipInterpolator(us, mms)

    def curve(u):
        return interp(np.clip(u, us[0], us[-1]))
    return curve


def profile_to_log(profile: Profile, cal: Calibration, head: Head,
                   sweep_rate: float = SWEEP_RATE_DEG_S,
                   pass_advance_mm: float = PASS_ADVANCE_MM) -> TeachLog:
    """Generate an ideal (jitter-free) teach log that cuts this profile on
    THIS head — head size only changes the psi pass spacing; u handles height.
    """
    g = head.geometry
    curve = profile_curve(profile)
    phi0 = max(JOINT_LIMITS["phi"].lo, g.phi_neckline_deg - PHI_LEAD_DEG)
    phi1 = min(JOINT_LIMITS["phi"].hi, g.phi_top_deg + PHI_LEAD_DEG)
    theta_hi = float(cal.mm_to_theta(curve(1.0)))

    def theta_for(phi):
        return cal.mm_to_theta(curve(g.u(phi)))

    rows_phi, rows_psi, rows_theta = [], [], []

    def seg(phi_a, phi_b, psi_c, theta_fn, rate):
        n = max(1, int(round(abs(phi_b - phi_a) / rate / TICK_S)))
        phis = np.linspace(phi_a, phi_b, n + 1)
        rows_phi.append(phis)
        rows_psi.append(np.full(n + 1, psi_c))
        rows_theta.append(np.asarray(theta_fn(phis), dtype=float)
                          * np.ones(n + 1))

    centers = pass_centers_deg(head, pass_advance_mm)
    psi_rate = JOINT_LIMITS["psi"].max_rate * 0.6
    for k, psi_c in enumerate(centers):
        if k > 0:
            prev = centers[k - 1]
            n = max(1, int(round((psi_c - prev) / psi_rate / TICK_S)))
            rows_phi.append(np.full(n, phi1))
            rows_psi.append(np.linspace(prev, psi_c, n))
            rows_theta.append(np.full(n, theta_hi))
            # come back down at the longest length: cuts nothing extra
            seg(phi1, phi0, psi_c, lambda p: np.full_like(p, theta_hi),
                sweep_rate * 1.2)
            # settle theta to the bottom value while below the neckline,
            # no faster than half the servo's rate limit
            theta_lo = float(theta_for(phi0))
            theta_rate = JOINT_LIMITS["theta"].max_rate * 0.5
            n = max(2, int(round(abs(theta_hi - theta_lo) / theta_rate / TICK_S)))
            rows_phi.append(np.full(n, phi0))
            rows_psi.append(np.full(n, psi_c))
            rows_theta.append(np.linspace(theta_hi, theta_lo, n))
        seg(phi0, phi1, psi_c, theta_for, sweep_rate)

    phi = np.concatenate(rows_phi)
    psi = np.concatenate(rows_psi)
    theta = np.concatenate(rows_theta)
    t_ms = np.arange(len(phi)) * TICK_MS
    return TeachLog.from_arrays(t_ms, phi, psi, theta,
                                np.ones(len(phi), dtype=bool))
