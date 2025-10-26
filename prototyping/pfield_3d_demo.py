# Re-run the 3D PF testbed (the previous state was cleared).

from dataclasses import dataclass
import numpy as np
import math
from typing import List, Tuple, Dict, Any

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def _is_positive_finite(v: float) -> bool:
    return math.isfinite(v) and v > 1e-12


def soft_saturate_norm(v: np.ndarray, max_norm: float, beta: float = 1.0) -> np.ndarray:
    """
    Softly saturate vector 'v' by its L2 norm, matching the C++ formula:
        s = (maxNorm * tanh(beta * n / maxNorm)) / n
        v_sat = v * s
    """
    if not _is_positive_finite(max_norm):
        return v
    n = float(np.linalg.norm(v))
    if not _is_positive_finite(n):
        return v
    s = (max_norm * math.tanh(beta * n / max_norm)) / n
    return v * s


def rate_limit_step(prev: np.ndarray, curr: np.ndarray, dmax: float) -> np.ndarray:
    """
    Rate limit the step from 'prev' to 'curr' so ||curr - prev|| <= dmax.
    """
    if (not _is_positive_finite(dmax)) or dmax <= 0.0:
        return curr
    d = curr - prev
    dn = float(np.linalg.norm(d))
    if (not _is_positive_finite(dn)) or (dn <= dmax):
        return curr
    return prev + d * (dmax / dn)


def _clip_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    n = np.linalg.norm(v)
    if n <= max_norm or max_norm <= 0:
        return v
    return v * (max_norm / (n + 1e-12))


def _rotzxy(yaw: float, pitch: float, roll: float) -> np.ndarray:
    cz, sz = math.cos(yaw), math.sin(yaw)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cx, sx = math.cos(roll), math.sin(roll)
    Rz = np.array([[cz, -sz, 0],
                   [sz,  cz, 0],
                   [0,   0, 1]])
    Ry = np.array([[cy, 0, sy],
                   [0, 1,  0],
                   [-sy, 0, cy]])
    Rx = np.array([[1,  0,   0],
                   [0, cx, -sx],
                   [0, sx,  cx]])
    return Rz @ Ry @ Rx


@dataclass
class SphereObstacle:
    cx: float
    cy: float
    cz: float
    radius: float
    influence_distance: float = 1.0

    def distance_and_direction(self, p: np.ndarray) -> Tuple[float, np.ndarray]:
        c = np.array([self.cx, self.cy, self.cz], dtype=float)
        v = p - c
        dist_center = np.linalg.norm(v)
        if dist_center == 0.0:
            return self.radius, np.array([1.0, 0.0, 0.0])
        n_hat = v / dist_center
        signed = dist_center - self.radius
        return signed, n_hat


@dataclass
class OBBObstacle:
    cx: float
    cy: float
    cz: float
    half_x: float
    half_y: float
    half_z: float
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    influence_distance: float = 1.0

    def R(self) -> np.ndarray:
        return _rotzxy(self.yaw, self.pitch, self.roll)

    def _world_to_local(self, p: np.ndarray) -> np.ndarray:
        R = self.R().T
        return R @ (p - np.array([self.cx, self.cy, self.cz]))

    def _local_to_world_vec(self, v: np.ndarray) -> np.ndarray:
        return self.R() @ v

    def distance_and_direction(self, p: np.ndarray) -> Tuple[float, np.ndarray]:
        pl = self._world_to_local(p)
        h = np.array([self.half_x, self.half_y, self.half_z])
        q = np.clip(pl, -h, h)
        d = pl - q
        outside_dist = np.linalg.norm(d)

        if outside_dist > 0.0:
            n_local = d / (outside_dist + 1e-12)
            signed = outside_dist
        else:
            pen = h - np.abs(pl)
            axis = int(np.argmin(pen))
            sign = 1.0 if pl[axis] >= 0 else -1.0
            n_local = np.zeros(3)
            n_local[axis] = sign
            signed = -pen[axis]

        n_world = self._local_to_world_vec(n_local)
        n_world /= (np.linalg.norm(n_world) + 1e-12)
        return float(signed), n_world


class PotentialField3D:
    def __init__(self,
                 attractive_gain: float = 1.0,
                 repulsive_gain: float = 1.0,
                 linear_gain: float = 1.0,
                 max_speed: float = 1.0,
                 max_accel: float = 2.0,
                 damping: float = 0.0,
                 beta_velocity: float = 1.0):
        self.attractive_gain = float(attractive_gain)
        self.repulsive_gain = float(repulsive_gain)
        self.linear_gain = float(linear_gain)
        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)
        self.damping = float(damping)
        self.beta_velocity = float(beta_velocity)
        self._spheres: List[SphereObstacle] = []
        self._obbs: List[OBBObstacle] = []

    def add_sphere(self, cx, cy, cz, radius, influence_distance=1.0):
        self._spheres.append(SphereObstacle(
            cx, cy, cz, radius, influence_distance))

    def add_box(self, cx, cy, cz, sx, sy, sz, yaw=0.0, pitch=0.0, roll=0.0, influence_distance=1.0):
        self._obbs.append(OBBObstacle(cx, cy, cz, sx/2.0, sy /
                          2.0, sz/2.0, yaw, pitch, roll, influence_distance))

    def attractive_force(self, p: np.ndarray, g: np.ndarray) -> np.ndarray:
        return self.attractive_gain * (g - p)

    def repulsive_force(self, p: np.ndarray) -> np.ndarray:
        total = np.zeros(3, dtype=float)
        for s in self._spheres:
            d, n_hat = s.distance_and_direction(p)
            d_inf = s.influence_distance
            if d < d_inf:
                d_eff = np.sign(d) * max(abs(d), 1e-6)
                coeff = self.repulsive_gain * \
                    (1.0/d_eff - 1.0/d_inf) * (1.0/(d_eff*d_eff))
                total += coeff * n_hat
        for b in self._obbs:
            d, n_hat = b.distance_and_direction(p)
            d_inf = b.influence_distance
            if d < d_inf:
                d_eff = np.sign(d) * max(abs(d), 1e-6)
                coeff = self.repulsive_gain * \
                    (1.0/d_eff - 1.0/d_inf) * (1.0/(d_eff*d_eff))
                total += coeff * n_hat
        return total

    def total_force(self, p: np.ndarray, g: np.ndarray, v_curr: np.ndarray) -> np.ndarray:
        return self.attractive_force(p, g) + self.repulsive_force(p) - self.damping * v_curr

    def _apply_motion_limits(self, v_cmd: np.ndarray, v_prev: np.ndarray, dt: float, use_soft_saturate: bool = True) -> np.ndarray:
        """
          Mirror the C++ sequence:
            1) soft-saturate v_cmd by max speed
            2) rate-limit step by max accel * dt
            3) re-apply soft-saturation to ensure speed stays within the limit
          """
        if use_soft_saturate:
            # 1) soft-saturate instantaneous command
            v_soft = soft_saturate_norm(
                v_cmd, self.max_speed, self.beta_velocity)

            # 2) rate-limit step magnitude (acceleration limit)
            dV_max = self.max_accel * \
                (dt if _is_positive_finite(dt) else 0.0)
            v_rl = rate_limit_step(v_prev, v_soft, dV_max)

            # 3) re-apply soft saturation to keep just inside speed limit
            v_final = soft_saturate_norm(
                v_rl, self.max_speed, self.beta_velocity)

            return v_final
        else:
            v = _clip_norm(
                v_cmd, self.max_speed if self.max_speed > 0 else np.inf)
            a = (v - v_prev) / max(dt, 1e-6)
            a = _clip_norm(
                a, self.max_accel if self.max_accel > 0 else np.inf)
            v_limited = v_prev + a * dt
            v_limited = _clip_norm(
                v_limited, self.max_speed if self.max_speed > 0 else np.inf)
            return v_limited

    def plan_path(self,
                  start: Tuple[float, float, float],
                  goal: Tuple[float, float, float],
                  dt: float = 0.02,
                  max_steps: int = 6000,
                  goal_tolerance: float = 0.05,
                  stagnation_window: int = 120,
                  stagnation_eps: float = 1e-4,
                  use_soft_saturate: bool = True) -> Dict[str, Any]:
        p = np.array(start, dtype=float)
        g = np.array(goal, dtype=float)
        v_prev = np.zeros(3, dtype=float)

        xs, ys, zs = [p[0]], [p[1]], [p[2]]
        vxs, vys, vzs = [0.0], [0.0], [0.0]
        ts = [0.0]
        buf: List[float] = []

        for k in range(1, max_steps + 1):
            F = self.total_force(p, g, v_prev)
            v_cmd = self.linear_gain * F
            v = self._apply_motion_limits(v_cmd, v_prev, dt, use_soft_saturate)

            p_next = p + v * dt
            p = p_next
            v_prev = v

            xs.append(p[0])
            ys.append(p[1])
            zs.append(p[2])
            vxs.append(v[0])
            vys.append(v[1])
            vzs.append(v[2])
            ts.append(k * dt)

            if np.linalg.norm(p - g) <= goal_tolerance:
                break

            buf.append(np.linalg.norm(v))
            if len(buf) > stagnation_window:
                buf.pop(0)
                if np.mean(buf) < stagnation_eps:
                    break

        return {
            "x": np.array(xs), "y": np.array(ys), "z": np.array(zs),
            "vx": np.array(vxs), "vy": np.array(vys), "vz": np.array(vzs),
            "t": np.array(ts),
            "goal_reached": np.linalg.norm(p - g) <= goal_tolerance
        }

    def simulate(self,
                 start: Tuple[float, float, float],
                 goal: Tuple[float, float, float],
                 dt: float = 0.03,
                 max_steps: int = 6000,
                 goal_tolerance: float = 0.05,
                 use_soft_saturate: bool = True,
                 save_path: str = None,
                 fps: int = 30,
                 dpi: int = 100,
                 writer: str = "pillow"):
        p = np.array(start, dtype=float)
        g = np.array(goal, dtype=float)
        v_prev = np.zeros(3, dtype=float)

        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')

        xs = [p[0], g[0]] + [s.cx for s in self._spheres] + \
            [b.cx for b in self._obbs]
        ys = [p[1], g[1]] + [s.cy for s in self._spheres] + \
            [b.cy for b in self._obbs]
        zs = [p[2], g[2]] + [s.cz for s in self._spheres] + \
            [b.cz for b in self._obbs]
        ax.set_xlim(min(xs)-2, max(xs)+2)
        ax.set_ylim(min(ys)-2, max(ys)+2)
        ax.set_zlim(min(zs)-2, max(zs)+2)

        # spheres
        u = np.linspace(0, 2*np.pi, 20)
        vgrid = np.linspace(0, np.pi, 10)
        for s in self._spheres:
            xsph = s.radius*np.outer(np.cos(u), np.sin(vgrid)) + s.cx
            ysph = s.radius*np.outer(np.sin(u), np.sin(vgrid)) + s.cy
            zsph = s.radius*np.outer(np.ones_like(u), np.cos(vgrid)) + s.cz
            ax.plot_wireframe(xsph, ysph, zsph, linewidth=0.4)
            Rinf = s.radius + s.influence_distance
            xi = Rinf*np.outer(np.cos(u), np.sin(vgrid)) + s.cx
            yi = Rinf*np.outer(np.sin(u), np.sin(vgrid)) + s.cy
            zi = Rinf*np.outer(np.ones_like(u), np.cos(vgrid)) + s.cz
            ax.plot_wireframe(xi, yi, zi, linewidth=0.3)

        def draw_obb(ax, obb: OBBObstacle, inflate: float = 0.0, ls='-'):
            h = np.array([obb.half_x+inflate, obb.half_y +
                         inflate, obb.half_z+inflate])
            corners = np.array([[+h[0], +h[1], +h[2]],
                                [+h[0], +h[1], -h[2]],
                                [+h[0], -h[1], +h[2]],
                                [+h[0], -h[1], -h[2]],
                                [-h[0], +h[1], +h[2]],
                                [-h[0], +h[1], -h[2]],
                                [-h[0], -h[1], +h[2]],
                                [-h[0], -h[1], -h[2]]])
            R = obb.R()
            W = (R @ corners.T).T + np.array([obb.cx, obb.cy, obb.cz])
            edges = [(0, 1), (0, 2), (0, 4),
                     (7, 6), (7, 5), (7, 3),
                     (1, 3), (1, 5),
                     (2, 3), (2, 6),
                     (4, 5), (4, 6)]
            for i, j in edges:
                ax.plot([W[i, 0], W[j, 0]], [W[i, 1], W[j, 1]], [
                        W[i, 2], W[j, 2]], linestyle=ls, linewidth=0.8)

        for b in self._obbs:
            draw_obb(ax, b, inflate=0.0, ls='-')
            draw_obb(ax, b, inflate=b.influence_distance, ls='--')

        ax.scatter([start[0]], [start[1]], [start[2]])
        ax.scatter([goal[0]], [goal[1]], [goal[2]])

        traj_line, = ax.plot([], [], [], linewidth=1.5)
        agent, = ax.plot([], [], [], marker='o')
        hud_speed = ax.text2D(0.02, 0.96, "", transform=ax.transAxes)
        hud_rl = ax.text2D(0.02, 0.92, "", transform=ax.transAxes)
        path_x, path_y, path_z = [p[0]], [p[1]], [p[2]]

        def init():
            traj_line.set_data([], [])
            traj_line.set_3d_properties([])
            agent.set_data([p[0]], [p[1]])
            agent.set_3d_properties([p[2]])
            hud_speed.set_text("")
            hud_rl.set_text("")
            return traj_line, agent, hud_speed, hud_rl

        def update(frame):
            nonlocal p, v_prev
            F = self.total_force(p, g, v_prev)
            v_cmd = self.linear_gain * F
            v = self._apply_motion_limits(v_cmd, v_prev, dt, use_soft_saturate)
            p = p + v * dt
            # Save for HUD
            v_soft = soft_saturate_norm(
                v_cmd, self.max_speed, self.beta_velocity)
            dV_max = self.max_accel*(dt if _is_positive_finite(dt) else 0.0)
            will_rate_limit = (np.linalg.norm(v_soft-v_prev) > dV_max+1e-12)
            v_prev = v

            path_x.append(p[0])
            path_y.append(p[1])
            path_z.append(p[2])
            traj_line.set_data(path_x, path_y)
            traj_line.set_3d_properties(path_z)
            agent.set_data([p[0]], [p[1]])
            agent.set_3d_properties([p[2]])
            speed = np.linalg.norm(v)
            hud_speed.set_text(f"‖v‖ = {speed:.3f} m/s")
            hud_rl.set_text(
                f"Rate limiter: {'ON' if will_rate_limit else 'off'}")
            agent.set_markerfacecolor('none' if not will_rate_limit else None)
            agent.set_markeredgewidth(1.5 if not will_rate_limit else 2.5)

            if np.linalg.norm(p - g) <= goal_tolerance:
                anim.event_source.stop()
            return traj_line, agent, hud_speed, hud_rl

        anim = animation.FuncAnimation(
            fig, update, init_func=init, frames=max_steps, interval=dt*1000, blit=False)

        # Optional save to GIF (requires Pillow installed), or a supported writer
        if save_path:
            try:
                if writer == "pillow":
                    try:
                        from matplotlib.animation import PillowWriter
                        pw = PillowWriter(fps=fps)
                        anim.save(save_path, writer=pw, dpi=dpi)
                    except Exception:
                        # Fallback to string-based writer if PillowWriter import path differs
                        anim.save(save_path, writer="pillow", fps=fps, dpi=dpi)
                else:
                    # e.g., writer="imagemagick" (requires ImageMagick installed)
                    anim.save(save_path, writer=writer, fps=fps, dpi=dpi)
                print(f"Saved animation to {save_path}")
            except Exception as e:
                print(f"Failed to save animation to {save_path}: {e}")

        plt.show()


def _demo_3d():
    pf = PotentialField3D(
        attractive_gain=1.2,
        repulsive_gain=0.22,
        linear_gain=1.0,
        max_speed=1.5,
        max_accel=3.2,
        damping=0.0,
        beta_velocity=1.0
    )
    pf.add_sphere(cx=1.5, cy=1.0, cz=0.7, radius=0.7, influence_distance=1.0)
    pf.add_box(cx=3.0, cy=-0.5, cz=0.0, sx=1.2, sy=2.0, sz=1.0,
               yaw=np.deg2rad(25.0), pitch=np.deg2rad(10.0), roll=np.deg2rad(-5.0),
               influence_distance=0.9)

    start = (-2.0, -1.5, -0.5)
    goal = (4.0,  1.0,  0.6)

    dt = 0.04
    tol = 0.08
    steps = 4000

    res = pf.plan_path(start, goal, dt=dt, goal_tolerance=tol,
                       max_steps=steps, use_soft_saturate=True)
    res_clipped = pf.plan_path(start, goal, dt=dt, goal_tolerance=tol,
                               max_steps=steps, use_soft_saturate=False)
    print("Goal reached:", res["goal_reached"], "Duration:", float(res["t"][-1]), "s",
          "End:", (float(res["x"][-1]), float(res["y"][-1]), float(res["z"][-1])))
    # Plot kinematics
    plot_kinematics(
        "Kinematics vs Time (Soft-Saturated)", res, show=True, save_path="Demo3DKinematicsSoftSaturate"
    )
    plot_kinematics(
        "Kinematics vs Time (Clipped)", res_clipped, show=False, save_path="Demo3DKinematicsClipped"
    )
    pf.simulate(start, goal, dt=dt, goal_tolerance=tol,
                max_steps=steps, use_soft_saturate=True, save_path="Demo3DSimulation.gif")


def plot_kinematics(title: str, res: Dict[str, Any], show: bool = True, save_path: str = None):
    """
    Plot position, velocity, and acceleration vs time from a plan_path result dict.

    Expected keys in res:
      - "t": time array [N]
      - "x", "y", "z": position components [N]
      - "vx", "vy", "vz": velocity components [N]

    Acceleration is computed numerically from velocity using np.gradient over time.
    """

    t = np.asarray(res.get("t", []), dtype=float)
    x = np.asarray(res.get("x", []), dtype=float)
    y = np.asarray(res.get("y", []), dtype=float)
    z = np.asarray(res.get("z", []), dtype=float)
    vx = np.asarray(res.get("vx", []), dtype=float)
    vy = np.asarray(res.get("vy", []), dtype=float)
    vz = np.asarray(res.get("vz", []), dtype=float)

    if t.size == 0:
        raise ValueError("Result dictionary is missing time array 't'.")

    # Compute accelerations via gradient over time (handles non-uniform dt)
    ax = np.gradient(vx, t)
    ay = np.gradient(vy, t)
    az = np.gradient(vz, t)

    pos_mag = np.sqrt(x*x + y*y + z*z)
    vel_mag = np.sqrt(vx*vx + vy*vy + vz*vz)
    acc_mag = np.sqrt(ax*ax + ay*ay + az*az)

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 9))

    fig.suptitle(f'{title}', fontsize=16)

    # Position
    axes[0].plot(t, x, label='x')
    axes[0].plot(t, y, label='y')
    axes[0].plot(t, z, label='z')
    axes[0].plot(t, pos_mag, '--', color='gray', linewidth=1.0, label='|p|')
    axes[0].set_ylabel('Position [m]')
    axes[0].grid(True, linestyle=':')
    axes[0].legend(loc='best', fontsize=9)

    # Velocity
    axes[1].plot(t, vx, label='vx')
    axes[1].plot(t, vy, label='vy')
    axes[1].plot(t, vz, label='vz')
    axes[1].plot(t, vel_mag, '--', color='gray', linewidth=1.0, label='|v|')
    axes[1].set_ylabel('Velocity [m/s]')
    axes[1].grid(True, linestyle=':')
    axes[1].legend(loc='best', fontsize=9)

    # Acceleration
    axes[2].plot(t, ax, label='ax')
    axes[2].plot(t, ay, label='ay')
    axes[2].plot(t, az, label='az')
    axes[2].plot(t, acc_mag, '--', color='gray', linewidth=1.0, label='|a|')
    axes[2].set_ylabel('Acceleration [m/s^2]')
    axes[2].set_xlabel('Time [s]')
    axes[2].grid(True, linestyle=':')
    axes[2].legend(loc='best', fontsize=9)

    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
    if show:
        plt.show()
    return fig, axes


if __name__ == "__main__":
    _demo_3d()
