"""Virtual head: ellipsoid scalp, spring/contact model, hair-length field.

Geometry convention:
- The rail arc lives in a vertical plane; psi rotates that plane around the
  head's vertical axis (0 = left ear, 90 = back, 180 = right ear).
- phi (0..70 deg) is position along the arc. It maps to elevation
  el = phi + EL_OFFSET so phi=0 sits below the equator near the neckline.
- The clipper heel rides on a spring slide pointing at the head centre from
  a carriage at rail_radius_mm. Contact = the scalp is within spring reach.
"""
from __future__ import annotations

import numpy as np

from .interfaces import HeadGeometry, JOINT_LIMITS

EL_OFFSET_DEG = -20.0   # phi=0 -> 20 deg below the equator

# spring slide (A4): heel protrudes 5..30 mm inward from the carriage face
SPRING_MIN_MM = 5.0
SPRING_TRAVEL_MM = 25.0
SPRING_MAX_MM = SPRING_MIN_MM + SPRING_TRAVEL_MM

CLEARANCE_MM = 8.0      # nominal gap at the closest scalp point (A1)

HAIR_START_MM = 20.0


def _direction(el_rad, psi_rad):
    """Unit vector from head centre for elevation el and azimuth psi."""
    return np.stack([np.cos(el_rad) * np.cos(psi_rad),
                     np.cos(el_rad) * np.sin(psi_rad),
                     np.sin(el_rad)], axis=-1)


class Head:
    def __init__(self, scale: float = 1.0,
                 center_offset_mm: tuple[float, float, float] = (0, 0, 0),
                 geometry: HeadGeometry | None = None,
                 grid_step_deg: float = 1.0,
                 rail_radius_mm: float | None = None):
        # ellipsoid semi-axes (mm): x front-back, y ear-ear, z vertical
        self.axes = np.array([90.0, 78.0, 100.0]) * scale
        self.offset = np.array(center_offset_mm, dtype=float)
        self.geometry = geometry or HeadGeometry()

        phi_lim = JOINT_LIMITS["phi"]
        psi_lim = JOINT_LIMITS["psi"]
        self.phi_grid = np.arange(phi_lim.lo, phi_lim.hi + 1e-9, grid_step_deg)
        self.psi_grid = np.arange(psi_lim.lo, psi_lim.hi + 1e-9, grid_step_deg)
        self.hair = np.full((len(self.phi_grid), len(self.psi_grid)),
                            HAIR_START_MM)

        # A1 geometry lock: rail radius fits THIS head (nominal, no offset)
        if rail_radius_mm is None:
            r = self.radius(self.phi_grid[:, None], self.psi_grid[None, :],
                            nominal=True)
            rail_radius_mm = float(r.max()) + CLEARANCE_MM
        self.rail_radius = rail_radius_mm

    # ------------------------------------------------------------- geometry

    def radius(self, phi_deg, psi_deg, nominal: bool = False):
        """Distance from the head-axis origin to the scalp along (phi, psi).

        With a centre offset o, solves |o + t*d| on the ellipsoid for t.
        """
        el = np.radians(np.asarray(phi_deg, dtype=float) + EL_OFFSET_DEG)
        ps = np.radians(np.asarray(psi_deg, dtype=float))
        el, ps = np.broadcast_arrays(el, ps)
        d = _direction(el, ps)
        o = np.zeros(3) if nominal else self.offset
        # ((o + t d) / axes)^2 = 1  ->  a t^2 + 2 b t + c = 0
        dn = d / self.axes
        on = o / self.axes
        a = np.sum(dn * dn, axis=-1)
        b = np.sum(dn * on, axis=-1)
        c = np.sum(on * on, axis=-1) - 1.0
        disc = b * b - a * c
        disc = np.maximum(disc, 0.0)
        return (-b + np.sqrt(disc)) / a

    def horizontal_radius(self, phi_deg, psi_deg):
        """Distance from the vertical axis: converts psi degrees to mm of arc."""
        el = np.radians(np.asarray(phi_deg, dtype=float) + EL_OFFSET_DEG)
        return self.radius(phi_deg, psi_deg) * np.cos(el)

    # ------------------------------------------------------------- contact

    def gap(self, phi_deg, psi_deg):
        """Carriage-face to scalp distance along the spring axis."""
        return self.rail_radius - self.radius(phi_deg, psi_deg)

    def contact(self, phi_deg, psi_deg):
        """True while the spring keeps the heel on the scalp (A4/A7)."""
        g = self.gap(phi_deg, psi_deg)
        return bool(np.all(g <= SPRING_MAX_MM)) if np.ndim(g) == 0 \
            else g <= SPRING_MAX_MM

    # ------------------------------------------------------------- cutting

    def cut(self, phi_deg: float, psi_deg: float, leave_mm: float,
            width_mm: float, depth_mm: float) -> None:
        """Clip hair under the clipper footprint down to leave_mm."""
        r = float(self.radius(phi_deg, psi_deg))
        rho = float(self.horizontal_radius(phi_deg, psi_deg))
        rho = max(rho, 1.0)
        half_w_deg = np.degrees((width_mm / 2) / rho)   # across, in psi
        half_d_deg = np.degrees((depth_mm / 2) / r)     # along, in phi

        pm = np.abs(self.phi_grid - phi_deg) <= half_d_deg
        sm = np.abs(self.psi_grid - psi_deg) <= half_w_deg
        region = np.ix_(pm, sm)
        self.hair[region] = np.minimum(self.hair[region], leave_mm)

    # ------------------------------------------------------------- analysis

    def fade_zone_mask(self) -> np.ndarray:
        g = self.geometry
        return ((self.phi_grid >= g.phi_neckline_deg)
                & (self.phi_grid <= g.phi_top_deg))[:, None] \
            & np.ones(len(self.psi_grid), dtype=bool)[None, :]

    def cut_mask(self) -> np.ndarray:
        return self.hair < HAIR_START_MM - 1e-9

    def length_vs_u(self, n_bins: int = 20):
        """Mean remaining length of cut cells, binned by normalized height u.

        This is the measurable shape of the fade — the sim analog of
        photographing the head.
        """
        u = self.geometry.u(self.phi_grid)
        cut = self.cut_mask()
        centers = (np.arange(n_bins) + 0.5) / n_bins
        means = np.full(n_bins, np.nan)
        for i in range(n_bins):
            rows = (u >= i / n_bins) & (u < (i + 1) / n_bins)
            cells = self.hair[rows][cut[rows]]
            if cells.size:
                means[i] = cells.mean()
        return centers, means
