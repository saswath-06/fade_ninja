"""Build out/fadebench.html: run the sim, export the recordings and head
geometry as JSON, inject into the template.

    uv run python tools/build_visual.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fade_ninja.head import Head
from fade_ninja.replay import _snap_to_limits, smooth
from fade_ninja.rig import Rig
from fade_ninja.teach import high_fade_mm, teach_fade

ROOT = Path(__file__).resolve().parent.parent


def record(fade_mm=None, seed=0, teach_seed=1):
    rig = Rig(head=Head(), seed=seed)
    kwargs = {"fade_mm": fade_mm} if fade_mm else {}
    log = teach_fade(rig, seed=teach_seed, **kwargs)
    lg = _snap_to_limits(smooth(log))  # what REPLAY actually executes
    a = lg.arrays()
    samples = [[round(p, 2), round(s, 2), round(th, 2), int(c)]
               for p, s, th, c in zip(a["phi"], a["psi"], a["theta"],
                                      a["contact"])]
    u, mm = rig.head.length_vs_u(20)
    valid = ~np.isnan(mm)
    return {
        "samples": samples,
        "final_u": [round(float(v), 3) for v in u[valid]],
        "final_mm": [round(float(v), 3) for v in mm[valid]],
    }


def main():
    head = Head()
    data = {
        "tick_ms": 20,
        "hair_start_mm": 20.0,
        "clipper_width_mm": 45.0,
        "blade_depth_mm": 4.0,
        "geometry": {"neck": head.geometry.phi_neckline_deg,
                     "top": head.geometry.phi_top_deg},
        "phi_max": 70, "psi_max": 180,
        "calibration": [[0, 0.6], [15, 1.5], [30, 3.1], [45, 5.4], [60, 8.2]],
        "rho_per_phi": [round(float(v), 1) for v in
                        head.horizontal_radius(head.phi_grid, 90.0)],
        "r_per_phi": [round(float(v), 1) for v in
                      head.radius(head.phi_grid, 90.0)],
        "recordings": {
            "low_fade": record(seed=0, teach_seed=1),
            "high_fade": record(fade_mm=high_fade_mm, seed=3, teach_seed=4),
        },
    }
    blob = json.dumps(data, separators=(",", ":"))
    tpl = (ROOT / "tools" / "fadebench_template.html").read_text()
    out = ROOT / "out" / "fadebench.html"
    out.parent.mkdir(exist_ok=True)
    out.write_text(tpl.replace("/*__DATA__*/{}", blob))
    print(f"wrote {out} ({out.stat().st_size / 1024:.0f} KB)")


if __name__ == "__main__":
    main()
