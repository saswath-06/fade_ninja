"""Figures for the demo: trajectory (B2.3), hair maps, taught-vs-replayed.

Colors follow the dataviz method: two fixed categorical slots, a single-hue
sequential ramp for magnitude (hair length), a two-hue diverging ramp with a
neutral midpoint for the difference map. Text stays in ink colors.
"""
from __future__ import annotations

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap

from .head import HAIR_START_MM, Head
from .interfaces import TeachLog

SERIES_1 = "#2a78d6"   # blue
SERIES_2 = "#eb6834"   # orange
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3e0"
SURFACE = "#fcfcfb"

# sequential: one hue, light -> dark (short hair = light / skin, long = dark)
SEQ = LinearSegmentedColormap.from_list(
    "hair", ["#f3f7fc", "#c7dcf3", "#7fb0e4", "#2a78d6", "#123f78"])
# diverging: warm and cool poles around a neutral gray midpoint
DIV = LinearSegmentedColormap.from_list(
    "diff", ["#eb6834", "#f4b393", "#eceae6", "#93bce8", "#2a78d6"])


def _style(ax, title: str, xlabel: str, ylabel: str):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=INK, fontsize=11, loc="left")
    ax.set_xlabel(xlabel, color=INK_2, fontsize=9)
    ax.set_ylabel(ylabel, color=INK_2, fontsize=9)
    ax.tick_params(colors=INK_2, labelsize=8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.grid(color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)


def plot_trajectory(log: TeachLog, smoothed: TeachLog | None = None,
                    path: str = "trajectory.png"):
    """B2.3: theta against phi for a recording — a recorded ramp shows as a ramp."""
    fig, ax = plt.subplots(figsize=(7, 4.2), facecolor=SURFACE)
    a = log.arrays()
    ax.plot(a["phi"], a["theta"], color=SERIES_1, linewidth=0.8, alpha=0.45,
            label="recorded (with hand tremor)")
    if smoothed is not None:
        s = smoothed.arrays()
        ax.plot(s["phi"], s["theta"], color=SERIES_2, linewidth=2,
                label="smoothed for replay")
    _style(ax, "Taught trajectory: tilt vs height",
           "phi — carriage position (deg)", "theta — tilt (deg)")
    ax.legend(frameon=False, fontsize=8, labelcolor=INK_2)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_hair_map(head: Head, title: str, ax=None):
    standalone = ax is None
    if standalone:
        fig, ax = plt.subplots(figsize=(7, 4.2), facecolor=SURFACE)
    im = ax.imshow(head.hair, origin="lower", aspect="auto", cmap=SEQ,
                   vmin=0, vmax=HAIR_START_MM,
                   extent=[head.psi_grid[0], head.psi_grid[-1],
                           head.phi_grid[0], head.phi_grid[-1]])
    g = head.geometry
    for y in (g.phi_neckline_deg, g.phi_top_deg):
        ax.axhline(y, color=INK_2, linewidth=0.8, linestyle=":")
    _style(ax, title, "psi — around the head (deg)", "phi — height (deg)")
    ax.grid(False)
    if standalone:
        fig.colorbar(im, ax=ax, label="hair length (mm)")
        fig.tight_layout()
        return ax.figure
    return im


def plot_side_by_side(head_a: Head, head_b: Head, path: str = "d4.png"):
    """D4: the taught cut and the replayed cut, plus their difference."""
    fig, axes = plt.subplots(1, 3, figsize=(14.5, 4.2), facecolor=SURFACE,
                             layout="constrained")
    im0 = plot_hair_map(head_a, "Taught cut (wig 1)", axes[0])
    plot_hair_map(head_b, "Replayed cut (fresh wig)", axes[1])
    diff = head_b.hair - head_a.hair
    im2 = axes[2].imshow(diff, origin="lower", aspect="auto", cmap=DIV,
                         vmin=-2, vmax=2,
                         extent=[head_a.psi_grid[0], head_a.psi_grid[-1],
                                 head_a.phi_grid[0], head_a.phi_grid[-1]])
    _style(axes[2], "Difference (mm)", "psi (deg)", "phi (deg)")
    axes[2].grid(False)
    fig.colorbar(im0, ax=list(axes[:2]), label="hair length (mm)",
                 fraction=0.04, pad=0.02)
    fig.colorbar(im2, ax=axes[2], label="replayed − taught (mm)",
                 fraction=0.08, pad=0.02)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def plot_fade_curves(curves: dict[str, tuple[np.ndarray, np.ndarray]],
                     path: str = "fades.png"):
    """Mean remaining length vs normalized height u, one line per cut."""
    fig, ax = plt.subplots(figsize=(7, 4.2), facecolor=SURFACE)
    for (label, (u, mm)), color in zip(curves.items(), (SERIES_1, SERIES_2)):
        valid = ~np.isnan(mm)
        ax.plot(u[valid], mm[valid], color=color, linewidth=2, label=label)
        ax.annotate(label, (u[valid][-1], mm[valid][-1]),
                    textcoords="offset points", xytext=(6, 0),
                    color=INK_2, fontsize=8, va="center")
    _style(ax, "The fade as measured on the head",
           "u — normalized height (0 = neckline, 1 = top)",
           "hair length left (mm)")
    ax.set_xlim(0, 1.25)
    ax.legend(frameon=False, fontsize=8, labelcolor=INK_2, loc="upper left")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path
