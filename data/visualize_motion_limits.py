#!/usr/bin/env python3
import math
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from pathlib import Path

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

# Time steps to visualize acceleration limiting effect
dt_list = [0.01, 0.05, 0.1, 0.2]

out_dir = Path(__file__).parent
out_dir.mkdir(parents=True, exist_ok=True)


@dataclass
class Limits:
    vmax: float
    amax: float


limits = Limits(max_linear_velocity, max_linear_acceleration)


def attractive_force_mag(d: float) -> float:
    # F = -k * d -> magnitude k*d
    return attractive_gain * d


def repulsive_force_mag(d_obs: float) -> float:
    # From PotentialField::computeRepulsiveForceLinear
    # Only active if distance < influence_zone_scale
    if d_obs >= influence_zone_scale:
        return 0.0
    d = max(d_obs, min_distance)
    inv_d = 1.0 / d
    inv_inf = 1.0 / influence_zone_scale
    inv_d2 = inv_d * inv_d
    mag = repulsive_gain * (inv_d - inv_inf) * inv_d2
    return max(mag, 0.0)


def combine_forces(d: float) -> tuple[float, float, float]:
    """
    d: distance from goal along +x axis.
    Obstacle at obstacle_center. Distance to obstacle center = |d - obstacle_center|
    Attractive pulls toward goal (negative direction), repulsive pushes away from obstacle center.
    We treat everything 1D, so signs matter:
      Attractive force = -attractive_gain * d
      Repulsive force (if within influence) pushes away from obstacle: sign = +1 if d < obstacle_center else -1
    """
    F_attr = -attractive_gain * d
    dist_obs = abs(d - obstacle_center)
    F_rep_mag = repulsive_force_mag(dist_obs)
    if F_rep_mag > 0:
        # Direction: away from obstacle center
        sign = -1.0 if d > obstacle_center else 1.0
        F_rep = sign * F_rep_mag
    else:
        F_rep = 0.0
    F_total = F_attr + F_rep
    return F_attr, F_rep, F_total


def apply_motion_constraints(v_des: float, v_prev: float, dt: float, lim: Limits) -> float:
    """
    Mirror of C++ logic (1D simplification):
      1. Velocity cap
      2. Acceleration cap relative to prev
      3. Re-apply velocity cap
    """
    # Step 1 velocity cap
    v_limited = np.clip(v_des, -lim.vmax, lim.vmax)
    if dt <= 0.0:
        return v_limited
    dv = v_limited - v_prev
    dv_max = lim.amax * dt
    if abs(dv) > dv_max and dv_max > 0:
        v_limited = v_prev + np.sign(dv) * dv_max
    # Re-cap
    v_limited = np.clip(v_limited, -lim.vmax, lim.vmax)
    return v_limited


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
    fn = out_dir / 'pf_forces_vs_distance.png'
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
    fn = out_dir / 'pf_velocity_vs_distance.png'
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
        Fa, Fr, Ft = combine_forces(d)
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
    for dt in dt_list:
        v_prev = 0.0
        vc = []
        # Traverse from far to near (reverse distances) to emulate moving toward goal
        for F in reversed(F_tot_arr):
            v_des = F  # transform wrench->twist (gain=1)
            v_lim = apply_motion_constraints(v_des, v_prev, dt, limits)
            vc.append(v_lim)
            v_prev = v_lim
        # reverse back to align with distances ascending
        v_constrained_dict[dt] = np.array(list(reversed(vc)))

    plot_force_curves(distances, F_attr_arr, F_rep_arr, F_tot_arr)
    plot_velocity_curves(distances, v_uncon, v_constrained_dict)

    print("Done. Inspect generated PNGs.")


if __name__ == "__main__":
    main()
