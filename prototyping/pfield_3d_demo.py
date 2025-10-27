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
class CylinderObstacle:
    cx: float
    cy: float
    cz: float
    radius: float
    half_h: float  # half height along local Z
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
        """
        Signed distance to the surface of a finite cylinder (axis along local Z).
        Positive outside, negative inside. Also returns an outward normal direction.
        """
        pl = self._world_to_local(p)
        x, y, z = float(pl[0]), float(pl[1]), float(pl[2])
        r = math.hypot(x, y)
        # Outside/inside classification
        outside_rad = max(r - self.radius, 0.0)
        outside_z = max(abs(z) - self.half_h, 0.0)
        outside_dist = math.hypot(outside_rad, outside_z)

        if outside_dist > 0.0:
            # Compute nearest point on the cylinder surface
            if r > 1e-12:
                nx, ny = x / r, y / r
            else:
                nx, ny = 1.0, 0.0
            clamped_z = max(-self.half_h, min(self.half_h, z))
            nearest_local = np.array(
                [self.radius * nx, self.radius * ny, clamped_z])
            dvec_local = pl - nearest_local
            n_local = dvec_local / (np.linalg.norm(dvec_local) + 1e-12)
            signed = outside_dist
        else:
            # Inside: pick the smaller penetration to lateral or caps
            pr = self.radius - r
            pz = self.half_h - abs(z)
            if pr <= pz:
                # Closer to lateral surface
                if r > 1e-12:
                    n_local = np.array([x / r, y / r, 0.0])
                else:
                    n_local = np.array([1.0, 0.0, 0.0])
                signed = -pr
            else:
                # Closer to top/bottom cap
                n_local = np.array([0.0, 0.0, 1.0 if z >= 0.0 else -1.0])
                signed = -pz

        n_world = self._local_to_world_vec(n_local)
        n_world /= (np.linalg.norm(n_world) + 1e-12)
        return float(signed), n_world


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
        self._cyls: List["CylinderObstacle"] = []

    def add_sphere(self, cx, cy, cz, radius, influence_distance=1.0):
        self._spheres.append(SphereObstacle(
            cx, cy, cz, radius, influence_distance))

    def add_box(self, cx, cy, cz, sx, sy, sz, yaw=0.0, pitch=0.0, roll=0.0, influence_distance=1.0):
        self._obbs.append(OBBObstacle(cx, cy, cz, sx/2.0, sy /
                          2.0, sz/2.0, yaw, pitch, roll, influence_distance))

    def add_cylinder(self, cx, cy, cz, radius, height, yaw=0.0, pitch=0.0, roll=0.0, influence_distance=1.0):
        """
        Add a finite cylinder (local axis along +Z) with center at (cx,cy,cz), radius, and total height.
        Orientation is specified by yaw/pitch/roll (Z-Y-X order), matching OBB conventions.
        """
        self._cyls.append(CylinderObstacle(cx, cy, cz, radius,
                          height/2.0, yaw, pitch, roll, influence_distance))

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
        for c in self._cyls:
            d, n_hat = c.distance_and_direction(p)
            d_inf = c.influence_distance
            if d < d_inf:
                d_eff = np.sign(d) * max(abs(d), 1e-6)
                coeff = self.repulsive_gain * \
                    (1.0/d_eff - 1.0/d_inf) * (1.0/(d_eff*d_eff))
                total += coeff * n_hat
        return total

    def total_force(self, p: np.ndarray, g: np.ndarray, v_curr: np.ndarray) -> np.ndarray:
        return self.attractive_force(p, g) + self.repulsive_force(p) - self.damping * v_curr

    def _apply_motion_limits(self, v_cmd: np.ndarray, v_prev: np.ndarray, dt: float) -> np.ndarray:
        """
          Mirror the C++ sequence:
            1) soft-saturate v_cmd by max speed
            2) rate-limit step by max accel * dt
            3) re-apply soft-saturation to ensure speed stays within the limit
          """
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

    def plan_path(self,
                  start: Tuple[float, float, float],
                  goal: Tuple[float, float, float],
                  dt: float = 0.02,
                  max_steps: int = 6000,
                  goal_tolerance: float = 0.05,
                  stagnation_window: int = 120,
                  stagnation_eps: float = 1e-4) -> Dict[str, Any]:
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
            v = self._apply_motion_limits(v_cmd, v_prev, dt)

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
                 save_path: str = None,
                 fps: int = 30,
                 dpi: int = 100,
                 writer: str = "pillow",
                 save_every: int = 5,
                 bitrate: int = 1800):
        # Build a frame index list that decimates frames.
        frames_idx = list(range(0, max_steps, max(1, int(save_every))))
        p = np.array(start, dtype=float)
        g = np.array(goal, dtype=float)
        v_prev = np.zeros(3, dtype=float)

        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_box_aspect([1, 1, 1])
        ax.set_title("3D Potential Field Simulation")
        ax.grid(False)

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
        u = np.linspace(0, 2*np.pi, 16)
        vgrid = np.linspace(0, np.pi, 8)
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

        def draw_cylinder(ax, cyl: "CylinderObstacle", inflate: float = 0.0, ls='-'):
            R = cyl.radius + inflate
            H = cyl.half_h + inflate
            thetas = np.linspace(0, 2*np.pi, 24)
            zs = np.linspace(-H, H, 8)
            # Rotation and translation
            Rmat = cyl.R()
            center = np.array([cyl.cx, cyl.cy, cyl.cz])
            # Draw lateral rings
            for z in zs:
                circle_local = np.stack(
                    [R*np.cos(thetas), R*np.sin(thetas), np.full_like(thetas, z)], axis=1)
                circle_world = (Rmat @ circle_local.T).T + center
                ax.plot(circle_world[:, 0], circle_world[:, 1],
                        circle_world[:, 2], linestyle=ls, linewidth=0.8)
            # Draw a few vertical lines at fixed angles
            for th in thetas[::6]:
                p1_local = np.array([R*np.cos(th), R*np.sin(th), -H])
                p2_local = np.array([R*np.cos(th), R*np.sin(th),  H])
                p12 = (Rmat @ np.stack([p1_local, p2_local]).T).T + center
                ax.plot(p12[:, 0], p12[:, 1], p12[:, 2],
                        linestyle=ls, linewidth=0.8)

        for c in self._cyls:
            draw_cylinder(ax, c, inflate=0.0, ls='-')
            draw_cylinder(ax, c, inflate=c.influence_distance, ls='--')

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
            v = self._apply_motion_limits(v_cmd, v_prev, dt)
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
            fig, update, init_func=init, frames=frames_idx, interval=dt*1000, blit=False
        )

        if save_path:
            try:
                if writer.lower() in ("ffmpeg", "ffmpeg_file", "ffmpeg_writer", "mp4", "h264"):
                    from matplotlib.animation import FFMpegWriter
                    w = FFMpegWriter(fps=fps, bitrate=bitrate)
                    anim.save(save_path, writer=w, dpi=dpi)
                elif writer.lower() == "pillow":
                    from matplotlib.animation import PillowWriter
                    anim.save(save_path, writer=PillowWriter(
                        fps=fps), dpi=dpi, bitrate=bitrate)
                else:
                    anim.save(save_path, writer=writer, fps=fps, dpi=dpi)
                print(f"Saved animation to {save_path}")
            except Exception as e:
                print(f"Failed to save animation to {save_path}: {e}")

        plt.show()


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


def obstacles_in_the_way():
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
    # Add a cylinder obstacle (axis along its local Z, rotated by yaw/pitch/roll)
    pf.add_cylinder(cx=1.0, cy=-1.0, cz=0.3, radius=0.45, height=1.2,
                    yaw=np.deg2rad(15.0), pitch=np.deg2rad(0.0), roll=np.deg2rad(0.0),
                    influence_distance=0.8)

    start = (-2.0, -1.5, -0.5)
    goal = (4.0,  1.0,  0.6)

    dt = 0.01
    tol = 0.1
    steps = 4000

    res = pf.plan_path(start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)
    print("Goal reached:", res["goal_reached"], "Duration:", float(res["t"][-1]), "s",
          "End:", (float(res["x"][-1]), float(res["y"][-1]), float(res["z"][-1])))
    plot_kinematics(
        "Kinematics vs Time (Obstacles In the Way)", res, show=True, save_path="ObstaclesInTheWay"
    )
    pf.simulate(
        start, goal, dt=dt, goal_tolerance=tol,
        max_steps=steps,
        # save_path="Demo3DSimulation.gif", writer="pillow", fps=15, dpi=90, save_every=6, bitrate=1800
    )


def no_obstacles():
    pf = PotentialField3D(
        attractive_gain=1.0,
        repulsive_gain=0.0,
        linear_gain=1.0,
        max_speed=2.0,
        max_accel=4.0,
        damping=0.0,
        beta_velocity=1.0
    )

    start = (-2.0, -1.5, -0.5)
    goal = (4.0,  1.0,  0.6)

    dt = 0.01
    tol = 0.1
    steps = 3000

    res = pf.plan_path(start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)
    print("Goal reached:", res["goal_reached"], "Duration:", float(res["t"][-1]), "s",
          "End:", (float(res["x"][-1]), float(res["y"][-1]), float(res["z"][-1])))
    # Plot kinematics
    plot_kinematics(
        "Kinematics vs Time (No Obstacles)", res, show=True, save_path="NoObstacles"
    )
    pf.simulate(
        start, goal, dt=dt, goal_tolerance=tol,
        max_steps=steps,
        # save_path="Demo3DSimulationNoObstacles.gif", writer="pillow", fps=15, dpi=90, save_every=6, bitrate=1800
    )


def narrow_gap_corridor():
    """
    Two long OBB 'walls' separated by a narrow gap (doorway).
    Tests: repulsion shaping near tight passages, speed/accel limits.
    """
    pf = PotentialField3D(
        attractive_gain=1.2,
        repulsive_gain=0.35,
        linear_gain=1.0,
        max_speed=1.2,
        max_accel=2.0,
        damping=0.1,
        beta_velocity=1.0,
    )

    # Two long walls with a narrow doorway between them around x ~ 0
    wall_len = 6.0
    wall_th = 0.3
    gap_y = 0.25  # half-gap (actual gap ~0.5 m)
    inf = 0.6

    # Left wall (upper segment), right wall (lower segment) — rotated slightly
    pf.add_box(cx=-0.5, cy=+gap_y+1.0, cz=0.0, sx=wall_len, sy=wall_th, sz=1.5,
               yaw=np.deg2rad(5.0), influence_distance=inf)
    pf.add_box(cx=-0.5, cy=-gap_y-1.0, cz=0.0, sx=wall_len, sy=wall_th, sz=1.5,
               yaw=np.deg2rad(-5.0), influence_distance=inf)

    start = (-3.0, 0.0, 0.0)
    goal = (3.0, 0.0, 0.0)

    dt = 0.02
    tol = 0.05
    steps = 3500
    res = pf.plan_path(start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)
    print("[narrow_gap_corridor] Goal reached:",
          res["goal_reached"], "Duration:", float(res["t"][-1]))
    plot_kinematics("Narrow Gap Corridor - Kinematics",
                    res, show=True, save_path="NarrowGapCorridor")
    pf.simulate(start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)


def u_trap_local_minima():
    """
    A U-shaped cul-de-sac (three OBBs). Pure potential fields often stall here.
    Try adjusting damping / attractive_gain to see escape behavior.
    """
    pf = PotentialField3D(
        attractive_gain=1.0,
        repulsive_gain=0.4,
        linear_gain=1.0,
        max_speed=1.0,
        max_accel=2.0,
        damping=0.2,          # more damping can help
        beta_velocity=1.0,
    )

    inf = 0.8
    # Bottom of U
    pf.add_box(cx=0.0, cy=-0.5, cz=0.0, sx=4.0, sy=0.3,
               sz=1.5, yaw=0.0, influence_distance=inf)
    # Sides of U
    pf.add_box(cx=-1.9, cy=0.7, cz=0.0, sx=0.3, sy=2.0,
               sz=1.5, yaw=0.0, influence_distance=inf)
    pf.add_box(cx=+1.9, cy=0.7, cz=0.0, sx=0.3, sy=2.0,
               sz=1.5, yaw=0.0, influence_distance=inf)

    start = (0.0, 0.0, 0.0)        # inside the U
    goal = (0.0, 2.5, 0.0)        # outside the U, above

    dt = 0.02
    tol = 0.05
    steps = 4000
    res = pf.plan_path(start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)
    print("[u_trap_local_minima] Goal reached:",
          res["goal_reached"], "Duration:", float(res["t"][-1]))
    plot_kinematics("U-Trap (Local Minima) - Kinematics",
                    res, show=True, save_path="UTrapLocalMinima")
    pf.simulate(start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)


def cluttered_spheres(seed: int = 7):
    """
    Random 'forest' of spheres with varied radii/influence.
    Tests: vector superposition, oscillations, soft saturation ergonomics.
    """
    rng = np.random.default_rng(seed)
    pf = PotentialField3D(
        attractive_gain=1.1,
        repulsive_gain=0.25,
        linear_gain=1.0,
        max_speed=1.3,
        max_accel=2.5,
        damping=0.12,
        beta_velocity=1.2,
    )

    # Place ~10 spheres in a region; keep a corridor open
    for _ in range(10):
        cx = rng.uniform(-1.5, 2.5)
        cy = rng.uniform(-1.0, 1.0)
        cz = rng.uniform(-0.4, 0.4)
        r = rng.uniform(0.25, 0.55)
        inf = rng.uniform(0.6, 1.0)
        # Keep center corridor near y ~ 0 slightly clearer
        if abs(cy) < 0.2 and rng.random() < 0.7:
            cy += rng.choice([-1, 1]) * rng.uniform(0.3, 0.6)
        pf.add_sphere(cx, cy, cz, r, influence_distance=inf)

    start = (-2.0, 0.0, 0.0)
    goal = (3.0, 0.0, 0.0)

    dt = 0.02
    tol = 0.06
    steps = 4000
    res = pf.plan_path(start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)
    print("[cluttered_spheres] Goal reached:",
          res["goal_reached"], "Duration:", float(res["t"][-1]))
    plot_kinematics("Cluttered Spheres - Kinematics",
                    res, show=True, save_path="ClutteredSpheres")
    pf.simulate(start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)


def start_inside_obstacle():
    """
    Start point is inside a sphere; should be pushed out along outward normal.
    Good for verifying signed distance handling and clamp near singularity.
    """
    pf = PotentialField3D(
        attractive_gain=1.0,
        repulsive_gain=0.35,
        linear_gain=1.0,
        max_speed=1.2,
        max_accel=2.2,
        damping=0.1,
        beta_velocity=1.0,
    )
    pf.add_sphere(cx=0.0, cy=0.0, cz=0.0, radius=0.7, influence_distance=1.0)

    start = (0.0, 0.0, 0.0)     # inside the sphere
    goal = (2.0, 0.0, 0.0)

    dt = 0.02
    tol = 0.05
    steps = 3000
    res = pf.plan_path(start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)
    print("[start_inside_obstacle] Goal reached:",
          res["goal_reached"], "Duration:", float(res["t"][-1]))
    plot_kinematics("Start Inside Obstacle - Kinematics",
                    res, show=True, save_path="StartInsideObstacle")
    pf.simulate(start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)


def high_speed_low_accel():
    """
    Very high speed limit but tight acceleration limit — showcases the rate limiter (HUD 'ON' a lot).
    No obstacles; straight shot to goal.
    """
    pf = PotentialField3D(
        attractive_gain=1.5,
        repulsive_gain=0.0,
        linear_gain=1.2,
        max_speed=4.0,    # high speed ceiling
        max_accel=0.5,    # small accel
        damping=0.05,
        beta_velocity=1.0,
    )

    start = (-3.0, 0.0, 0.0)
    goal = (3.0, 0.0, 0.0)

    dt = 0.02
    tol = 0.05
    steps = 2500
    res = pf.plan_path(start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)
    print("[high_speed_low_accel] Goal reached:",
          res["goal_reached"], "Duration:", float(res["t"][-1]))
    plot_kinematics("High Speed, Low Accel - Kinematics",
                    res, show=True, save_path="HighSpeedLowAccel")
    pf.simulate(start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)


def _min_path_clearance(pf: PotentialField3D, xs, ys, zs) -> float:
    """Return the minimum signed clearance (distance from obstacle boundary) along the path."""
    pts = np.vstack([xs, ys, zs]).T
    min_d = +np.inf
    # Measure against all obstacles using their signed distance helpers (positive outside, negative inside).
    for p in pts:
        dmin = +np.inf
        for s in pf._spheres:
            d, _ = s.distance_and_direction(p)
            dmin = min(dmin, d)
        for b in pf._obbs:
            d, _ = b.distance_and_direction(p)
            dmin = min(dmin, d)
        min_d = min(min_d, dmin)
    return float(min_d)


def _draw_obstacles(ax, pf: PotentialField3D):
    # Spheres + influence
    u = np.linspace(0, 2*np.pi, 18)
    v = np.linspace(0, np.pi, 10)
    for s in pf._spheres:
        xs = s.radius*np.outer(np.cos(u), np.sin(v)) + s.cx
        ys = s.radius*np.outer(np.sin(u), np.sin(v)) + s.cy
        zs = s.radius*np.outer(np.ones_like(u), np.cos(v)) + s.cz
        ax.plot_wireframe(xs, ys, zs, linewidth=0.5)
        Rinf = s.radius + s.influence_distance
        xi = Rinf*np.outer(np.cos(u), np.sin(v)) + s.cx
        yi = Rinf*np.outer(np.sin(u), np.sin(v)) + s.cy
        zi = Rinf*np.outer(np.ones_like(u), np.cos(v)) + s.cz
        ax.plot_wireframe(xi, yi, zi, linewidth=0.4, linestyle='--')

    # OBBs + influence edges
    def draw_obb(obb: OBBObstacle, inflate=0.0, ls='-'):
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
        edges = [(0, 1), (0, 2), (0, 4), (7, 6), (7, 5), (7, 3),
                 (1, 3), (1, 5), (2, 3), (2, 6), (4, 5), (4, 6)]
        for i, j in edges:
            ax.plot([W[i, 0], W[j, 0]], [W[i, 1], W[j, 1]], [
                    W[i, 2], W[j, 2]], linestyle=ls, linewidth=0.8)

    for b in pf._obbs:
        draw_obb(b, inflate=0.0, ls='-')
        draw_obb(b, inflate=b.influence_distance, ls='--')


def damping_and_constraints_prevent_collision():
    """
    Showcases how damping + motion constraints can prevent collisions in a tight turn
    near an obstacle when acceleration limits would otherwise cause overshoot.
    """
    # --- Shared environment (one big sphere + a short wall) ---
    env = PotentialField3D(
        attractive_gain=1.2,   # same field gains both runs
        repulsive_gain=0.25,
        linear_gain=1.0,
        max_speed=1.0,         # placeholders; per-run instances override these
        max_accel=1.0,
        damping=0.0,
        beta_velocity=1.0,
    )
    # Obstacle layout
    env.add_sphere(cx=0.5, cy=0.0, cz=0.0, radius=0.65, influence_distance=0.9)
    # A short wall to “catch” the cut if the turn is too tight
    env.add_box(cx=1.2, cy=-0.1, cz=0.0, sx=0.2, sy=1.2, sz=1.0,
                yaw=np.deg2rad(10.0), influence_distance=0.6)

    start = (-1.8, -0.8, 0.0)
    goal = (2.0,  0.9, 0.0)

    dt, tol, steps = 0.02, 0.06, 3500

    # --- Run A: Risky — no damping, high speed limit, tight accel (hard to turn in time) ---
    pf_risky = PotentialField3D(
        attractive_gain=env.attractive_gain,
        repulsive_gain=env.repulsive_gain,
        linear_gain=env.linear_gain,
        max_speed=3.5,          # fast
        max_accel=0.5,          # sluggish acceleration
        damping=0.0,            # no viscous damping
        beta_velocity=1.0,
    )
    # Copy obstacles
    pf_risky._spheres = env._spheres.copy()
    pf_risky._obbs = env._obbs.copy()

    resA = pf_risky.plan_path(
        start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)
    clearanceA = _min_path_clearance(pf_risky, resA["x"], resA["y"], resA["z"])

    # --- Run B: Safe — add damping, lower speed cap, higher accel (more responsive, smoother) ---
    pf_safe = PotentialField3D(
        attractive_gain=env.attractive_gain,
        repulsive_gain=env.repulsive_gain,
        linear_gain=env.linear_gain,
        max_speed=1.2,          # slower top speed
        max_accel=2.5,          # can turn/adjust faster
        damping=0.35,           # viscous damping to quash overshoot
        beta_velocity=1.0,
    )
    pf_safe._spheres = env._spheres.copy()
    pf_safe._obbs = env._obbs.copy()

    resB = pf_safe.plan_path(
        start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)
    clearanceB = _min_path_clearance(pf_safe, resB["x"], resB["y"], resB["z"])

    print("[damping/constraints demo]")
    print(f"  Risky: goal_reached={resA['goal_reached']}, min_clearance={clearanceA:.3f} m "
          f"({'COLLISION' if clearanceA < 0 else 'OK'})")
    print(f"  Safe : goal_reached={resB['goal_reached']}, min_clearance={clearanceB:.3f} m "
          f"({'COLLISION' if clearanceB < 0 else 'OK'})")

    # --- Visualization: overlay both paths with the same obstacles ---
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Damping & Motion Constraints Prevent Collisions")
    ax.set_box_aspect([1, 1, 1])

    # Bounds
    xs = [start[0], goal[0]] + [s.cx for s in env._spheres] + \
        [b.cx for b in env._obbs]
    ys = [start[1], goal[1]] + [s.cy for s in env._spheres] + \
        [b.cy for b in env._obbs]
    zs = [start[2], goal[2]] + [s.cz for s in env._spheres] + \
        [b.cz for b in env._obbs]
    ax.set_xlim(min(xs)-2, max(xs)+2)
    ax.set_ylim(min(ys)-2, max(ys)+2)
    ax.set_zlim(min(zs)-2, max(zs)+2)
    ax.grid(False)

    # Obstacles
    _draw_obstacles(ax, env)

    # Paths
    ax.plot(resA["x"], resA["y"], resA["z"], linewidth=2.0,
            label=f"Risky (min d={clearanceA:.2f} m)")
    ax.plot(resB["x"], resB["y"], resB["z"], linewidth=2.0,
            linestyle="--", label=f"Safe (min d={clearanceB:.2f} m)")
    ax.scatter([start[0]], [start[1]], [start[2]], marker='o')
    ax.scatter([goal[0]],  [goal[1]],  [goal[2]],  marker='^')

    ax.legend(loc="upper left")
    plt.show()

    # Optional: plot speed profiles to see limiter/damping effect
    def _speed(res): return np.sqrt(res["vx"]**2 + res["vy"]**2 + res["vz"]**2)
    fig2, ax2 = plt.subplots(figsize=(8, 3.5))
    ax2.plot(resA["t"], _speed(resA), label="Risky speed")
    ax2.plot(resB["t"], _speed(resB), label="Safe speed", linestyle="--")
    ax2.set_title("Speed vs Time")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("|v| [m/s]")
    ax2.grid(True, linestyle=':')
    ax2.legend()
    plt.show()


if __name__ == "__main__":
    # obstacles_in_the_way()
    # no_obstacles()
    # narrow_gap_corridor()
    # u_trap_local_minima()
    # cluttered_spheres()
    # start_inside_obstacle()
    # high_speed_low_accel()
    damping_and_constraints_prevent_collision()
