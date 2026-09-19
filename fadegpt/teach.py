"""Synthetic barber: scripted teleop with injected hand tremor.

Stands in for the glove (PLAN.md cut line: "hand-jog the arm to author the
cut instead"). Drives the arm through fade passes while the recorder logs
the encoders — jitter and all — which is exactly what savgol must clean up.
"""
from __future__ import annotations

import numpy as np

from .calibration import Calibration
from .cutting import PASS_ADVANCE_MM
from .interfaces import TICK_MS, TICK_S, HeadGeometry, JOINT_LIMITS, TeachLog
from .rig import Rig

SWEEP_RATE_DEG_S = 25.0
PHI_LEAD_DEG = 3.0  # start a pass slightly below the neckline, end above top


def default_fade_mm(u):
    """A low fade: 0.6 mm at the neckline blending to 6 mm at the top."""
    return 0.6 + 5.4 * np.clip(u, 0.0, 1.0) ** 1.6


def high_fade_mm(u):
    """A high fade: stays short much longer, then blends fast."""
    return 0.6 + 5.4 * np.clip(u, 0.0, 1.0) ** 4.0


class HandTremor:
    """Resonant AR(2) noise centred near 8 Hz — physiological hand tremor.

    That band is well above the ~2 Hz the savgol pass keeps, which is the
    whole reason smoothing before replay works.
    """

    def __init__(self, sigma_deg: float, seed: int,
                 freq_hz: float = 8.0, r: float = 0.85):
        self._rng = np.random.default_rng(seed)
        w = 2 * np.pi * freq_hz * (0.02)  # rad per 20 ms tick
        self._a1 = 2 * r * np.cos(w)
        self._a2 = -r * r
        self._sigma = sigma_deg
        self._prev = np.zeros(3)
        self._prev2 = np.zeros(3)
        # run in so the process starts stationary, then rescale to sigma
        burn = [self._step() for _ in range(200)]
        scale = np.std(np.array(burn)[:, 0]) or 1.0
        self._gain = sigma_deg / scale

    def _step(self) -> np.ndarray:
        x = self._a1 * self._prev + self._a2 * self._prev2 + \
            self._rng.normal(0.0, 1.0, 3)
        self._prev2, self._prev = self._prev, x
        return x

    def __call__(self) -> np.ndarray:
        return self._step() * self._gain


def pass_centers_deg(head, pass_advance_mm: float = PASS_ADVANCE_MM) -> np.ndarray:
    """psi centres so adjacent 45 mm-wide passes overlap at the scalp."""
    g = head.geometry
    phi_mid = 0.5 * (g.phi_neckline_deg + g.phi_top_deg)
    rho = float(head.horizontal_radius(phi_mid, 90.0))
    step = np.degrees(pass_advance_mm / rho)
    lim = JOINT_LIMITS["psi"]
    return np.arange(lim.lo + step / 2, lim.hi - step / 2 + 1e-9, step)


def teach_fade(rig: Rig, fade_mm=default_fade_mm,
               sweep_rate: float = SWEEP_RATE_DEG_S,
               pass_advance_mm: float = PASS_ADVANCE_MM,
               tremor_deg: float = 0.25, seed: int = 1) -> TeachLog:
    """Cut a full fade on the rig's head, recording the whole thing (D3)."""
    head = rig.head
    g = head.geometry
    phi0 = max(JOINT_LIMITS["phi"].lo, g.phi_neckline_deg - PHI_LEAD_DEG)
    phi1 = min(JOINT_LIMITS["phi"].hi, g.phi_top_deg + PHI_LEAD_DEG)
    tremor = HandTremor(tremor_deg, seed)
    theta_hi = float(rig.cal.mm_to_theta(fade_mm(1.0)))

    def theta_for(phi: float) -> float:
        return float(rig.cal.mm_to_theta(fade_mm(float(g.u(phi)))))

    if not rig.arm.homed:
        rig.arm.home()
    centers = pass_centers_deg(head, pass_advance_mm)
    rig.goto(phi0, float(centers[0]), theta_for(phi0))

    rig.recorder.start()
    rig.clipper_on = True
    try:
        n_ticks = int(round((phi1 - phi0) / sweep_rate / TICK_S))
        for k, psi_c in enumerate(centers):
            if k > 0:
                # transit down at the longest length so nothing gets over-cut
                rig.goto(phi0, float(psi_c), theta_hi)
                # then ease the tilt down — a hand never slams the servo, and
                # a gentle corner is what savgol can smooth without artifacts
                theta_lo = theta_for(phi0)
                m = max(2, int(round(abs(theta_hi - theta_lo) / 60.0 / TICK_S)))
                for v in np.linspace(theta_hi, theta_lo, m):
                    rig.arm.track(phi0, float(psi_c), float(v))
                    rig.tick()
            for i in range(n_ticks + 1):
                phi = phi0 + (phi1 - phi0) * i / n_ticks
                j = tremor()
                rig.arm.track(phi + j[0], float(psi_c) + 0.3 * j[1],
                              theta_for(phi) + j[2])
                rig.tick()
    finally:
        rig.clipper_on = False
        rig.arm.stop()
    return rig.recorder.stop()


# ------------------------------------------------- synthetic logs (no rig)

def ramp_log(cal: Calibration, geom: HeadGeometry | None = None,
             mm0: float = 0.6, mm1: float = 6.0, duration_s: float = 4.0,
             psi: float = 90.0) -> TeachLog:
    """A deliberate linear ramp: mm rises linearly with u across the fade
    zone. Ground truth for the B3 log-to-profile gate."""
    geom = geom or HeadGeometry()
    n = int(round(duration_s / TICK_S))
    u = np.linspace(0.0, 1.0, n)
    t_ms = np.arange(n) * TICK_MS
    phi = geom.phi_neckline_deg + u * (geom.phi_top_deg - geom.phi_neckline_deg)
    mm = mm0 + (mm1 - mm0) * u
    theta = cal.mm_to_theta(mm)
    return TeachLog.from_arrays(t_ms, phi, np.full(n, psi), theta,
                                np.ones(n, dtype=bool))


def jittered(log: TeachLog, sigma_deg: float = 0.25, seed: int = 7) -> TeachLog:
    """The same path as recorded through a trembling hand."""
    a = log.arrays()
    n = len(log)
    out = {}
    for ax, s in (("phi", sigma_deg), ("psi", 0.3 * sigma_deg),
                  ("theta", sigma_deg)):
        tremor = HandTremor(s, seed=seed + hash(ax) % 1000)
        noise = np.array([tremor()[0] for _ in range(n)])
        out[ax] = a[ax] + noise
    return TeachLog.from_arrays(a["t_ms"], out["phi"], out["psi"],
                                out["theta"], a["contact"])
