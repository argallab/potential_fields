#!/usr/bin/env python3
r"""
Visualize Potential Field forces and moments as functions of their inputs.

Plots:
- Attractive force magnitude vs. Euclidean distance to goal
- Attractive moment magnitude vs. angular distance to goal
- Repulsive force magnitude vs. distance to obstacle (inside influence region)

Formulas (mirroring potential_fields/src/pfield/pfield.cpp):
- Attractive linear force:  F = -k_att * d * \hat{r}  (zero inside translationalTolerance)
  => |F| = k_att * d, for d > translationalTolerance; else 0
- Attractive moment:       M = -k_rot * (theta * \hat{u}) (zero inside rotationalThreshold)
  => |M| = k_rot * theta, for theta > rotationalThreshold; else 0
- Repulsive linear force:  F = k_rep * (1/d - 1/q) * (1/d^2) * \hat{r}, for d < q; else 0
  => |F| = k_rep * (1/d - 1/q) * (1/d^2), for d < q; else 0

where:
  d     = distance to goal (attractive) or obstacle center (repulsive)
  theta = angular distance in radians
  q     = influence distance (outer boundary of repulsion)

Note: In the C++ code, the obstacle "influence zone" is determined by geometry and an influence scale.
      Here, we visualize the core force law in terms of a single influence distance q for clarity.
"""
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from pathlib import Path
from pfield_3d_prototype import (
    attractive_force_magnitude,
    attractive_moment_magnitude,
    repulsive_force_magnitude,
)

# Defaults matched to PotentialField defaults
DEFAULT_ATTRACTIVE_GAIN = 1.0
DEFAULT_ROTATIONAL_ATTRACTIVE_GAIN = 0.7
DEFAULT_REPULSIVE_GAIN = 0.002
TRANSLATIONAL_TOLERANCE = 1e-3
ROTATIONAL_THRESHOLD = 0.02

GRAPH_LIMITS = {
    "min_x": 0.0,
    "max_x": 1.0,
    "min_y": 0.0,
    "max_y": 1.0,
}

# Save plots into the repository's data directory by default
THIS_DIR = Path(__file__).resolve().parent
DATA_DIR = THIS_DIR.parent / "data"
DATA_DIR.mkdir(parents=True, exist_ok=True)


def add_limits_to_ax(ax):
    ax.set_xlim(GRAPH_LIMITS["min_x"], GRAPH_LIMITS["max_x"])
    ax.set_ylim(GRAPH_LIMITS["min_y"], GRAPH_LIMITS["max_y"])


def plot_attractive_force(ax,
                          k_att_list=(DEFAULT_ATTRACTIVE_GAIN,),
                          d_max=2.0,
                          tol=TRANSLATIONAL_TOLERANCE):
    d = np.linspace(0.0, d_max, 1000)
    for k in k_att_list:
        mag = attractive_force_magnitude(d, k_att=k, tol=tol)
        ax.plot(d, mag, label=f"k_att={k}")
    ax.axvline(tol, color='k', linestyle='--', alpha=0.4, label=f"tol={tol}")
    ax.set_title("Attractive Force |F| vs Distance d")
    ax.set_xlabel("Distance to goal d [m]")
    ax.set_ylabel("|F| [N]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    add_limits_to_ax(ax)


def plot_attractive_moment(ax,
                           k_rot_list=(DEFAULT_ROTATIONAL_ATTRACTIVE_GAIN,),
                           th_max=math.pi,
                           threshold=ROTATIONAL_THRESHOLD):
    th = np.linspace(0.0, th_max, 1000)
    for k in k_rot_list:
        mag = attractive_moment_magnitude(th, k_rot=k, threshold=threshold)
        ax.plot(th, mag, label=f"k_rot={k}")
    ax.axvline(threshold, color='k', linestyle='--',
               alpha=0.4, label=f"threshold={threshold}")
    ax.set_title("Attractive Moment |M| vs Angular Distance θ")
    ax.set_xlabel("Angular distance θ [rad]")
    ax.set_ylabel("|M| [Nm]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    add_limits_to_ax(ax)


def plot_repulsive_force(ax,
                         k_rep_list=(DEFAULT_REPULSIVE_GAIN,),
                         influence_list=(0.25, 0.5, 1.0),
                         d_max=1.5):
    d = np.linspace(1e-4, d_max, 2000)
    for q in influence_list:
        for k in k_rep_list:
            mag = repulsive_force_magnitude(d, k_rep=k, influence_distance=q)
            ax.plot(d, mag, label=f"k_rep={k}, q={q}")
    # Annotate influence boundaries
    for q in influence_list:
        ax.axvline(q, color='k', linestyle='--', alpha=0.2)
    ax.set_ylim(bottom=0.0)
    ax.set_title("Repulsive Force |F| vs Distance d (inside influence)")
    ax.set_xlabel("Distance to obstacle center d [m]")
    ax.set_ylabel("|F| [N]")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=2)
    add_limits_to_ax(ax)


def main(save_path: str | None = None):
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))

    plot_attractive_force(axes[0], k_att_list=(0.5, 1.0, 2.0))
    plot_attractive_moment(axes[1], k_rot_list=(0.5, 0.7, 1.0))
    plot_repulsive_force(axes[2], k_rep_list=(
        0.5, 1.0, 2.0), influence_list=(0.25, 0.5, 1.0))

    fig.tight_layout()

    if save_path is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_path = DATA_DIR / f"pf_forces_{ts}.png"
    fig.savefig(save_path, dpi=160)
    print(f"Saved plot to: {save_path}")


if __name__ == "__main__":
    # Default save in the data directory; pass a full path to override.
    main()
