#!/usr/bin/env python3
import math
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from pfield_3d_prototype import (
    combine_forces_1d,
    apply_motion_constraints_1d,
)

# Potential Field constants (match pfield.hpp defaults / your chosen values)
# [Ns/m]  (force magnitude = gain * distance)
attractive_gain = 1.0
rotational_attractive_gain = 0.7      # (unused in these translational plots)
repulsive_gain = 20.0                 # [Ns/m]
max_linear_velocity = 1.0             # [m/s]
max_angular_velocity = 0.5            # [rad/s] (not plotted)
max_linear_acceleration = 1.0         # [m/s^2]
max_angular_acceleration = 0.25       # [rad/s^2] (not plotted)
influence_zone_scale = 1.25           # Q in the repulsive formula
min_distance = 1e-3                   # numerical floor to avoid blow-up

# Obstacle / layout configuration
# We model a 1D line toward the goal at x=0; obstacle sits between start and goal.
goal_x = 0.0
obstacle_center = 0.5                 # Distance of obstacle from goal along +x
# We scan distances from 0 (at goal) out past obstacle + influence zone
scan_max = obstacle_center + influence_zone_scale * 1.5
num_samples = 800

# Save results into the repository's data directory
THIS_DIR = Path(__file__).resolve().parent
DATA_DIR = THIS_DIR.parent / "data"
DATA_DIR.mkdir(parents=True, exist_ok=True)


def _combine_forces_1d(d: float) -> tuple[float, float, float]:
    return combine_forces_1d(
        distance_from_goal=d,
        obstacle_center=obstacle_center,
        k_att=attractive_gain,
        k_rep=repulsive_gain,
        influence_q=influence_zone_scale,
        min_distance=min_distance,
    )


def plot_force_curves(distances, F_attr, F_rep, F_total):
    plt.figure(figsize=(8, 5))
    plt.plot(distances, np.abs(F_attr), label='|Attractive Force|')
    plt.plot(distances, np.abs(F_rep), label='|Repulsive Force|')
    plt.plot(distances, np.abs(F_total),
             label='|Resultant Force|', linewidth=2)
    plt.axvline(obstacle_center, color='k', linestyle='--',
                alpha=0.4, label='Obstacle Center')
    plt.axvspan(obstacle_center - influence_zone_scale,
                obstacle_center + influence_zone_scale,
                color='orange', alpha=0.08, label='Influence Zone')
    plt.xlabel('Distance from Goal (m)')
    plt.ylabel('Force Magnitude (N)')
    plt.title('Potential Field Forces vs Distance (1D slice)')
    plt.legend()
    plt.grid(True)
    fn = DATA_DIR / 'pf_forces_vs_distance.png'
    plt.tight_layout()
    plt.savefig(fn, dpi=150)
    print(f"Saved {fn}")


def plot_velocity_curves(distances, v_uncon, v_constrained_dict):
    plt.figure(figsize=(8, 5))
    # plt.plot(distances, np.abs(v_uncon),
    #          label='|Unconstrained Velocity|', linewidth=2, color='C0')
    for dt, vc in v_constrained_dict.items():
        plt.plot(distances, np.abs(vc),
                 label=f'|Constrained Velocity| dt={dt}')
    plt.axhline(max_linear_velocity, color='k', linestyle='--',
                alpha=0.5, label='Velocity Cap')
    plt.xlabel('Distance from Goal (m)')
    plt.ylabel('Linear Velocity (m/s)')
    plt.title('Velocity vs Distance with Motion Constraints')
    plt.legend()
    plt.grid(True)
    fn = DATA_DIR / 'pf_velocity_vs_distance.png'
    plt.tight_layout()
    plt.savefig(fn, dpi=150)
    print(f"Saved {fn}")


def main():
    distances = np.linspace(0.0, scan_max, num_samples)
    F_attr_list = []
    F_rep_list = []
    F_tot_list = []

    # Compute forces
    for d in distances:
        Fa, Fr, Ft = _combine_forces_1d(float(d))
        F_attr_list.append(Fa)
        F_rep_list.append(Fr)
        F_tot_list.append(Ft)

    F_attr_arr = np.array(F_attr_list)
    F_rep_arr = np.array(F_rep_list)
    F_tot_arr = np.array(F_tot_list)

    # Unconstrained velocity (force -> velocity gain = 1)
    v_uncon = F_tot_arr.copy()

    # Constrained velocities for various dt, assuming starting from rest and walking inward (largest forces outside first)
    v_constrained_dict = {}
    for dt in (0.01, 0.05, 0.1, 0.2):
        v_prev = 0.0
        vc = []
        # Traverse from far to near (reverse distances) to emulate moving toward goal
        for F in reversed(F_tot_arr):
            v_des = F  # transform wrench->twist (gain=1)
            v_lim = apply_motion_constraints_1d(
                v_des=float(v_des),
                v_prev=float(v_prev),
                dt=float(dt),
                vmax=max_linear_velocity,
                amax=max_linear_acceleration,
            )
            vc.append(v_lim)
            v_prev = v_lim
        # reverse back to align with distances ascending
        v_constrained_dict[dt] = np.array(list(reversed(vc)))

    plot_force_curves(distances, F_attr_arr, F_rep_arr, F_tot_arr)
    plot_velocity_curves(distances, v_uncon, v_constrained_dict)

    print("Done. Inspect generated PNGs in data/.")


if __name__ == "__main__":
    main()
