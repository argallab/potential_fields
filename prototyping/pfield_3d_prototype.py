import math
import os
import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt
from typing import List, Tuple, Dict, Any
from matplotlib import animation
import scipy


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


# ------------------------------
# Reusable potential-field helpers
# Centralize lightweight utilities so visualization scripts can import instead
# of re-implementing formulas.
# ------------------------------

def attractive_force_magnitude(
    distance: np.ndarray | float,
    k_att: float = 1.0,
    tol: float = 1e-3,
) -> np.ndarray | float:
    """
    Magnitude of translational attractive force: |F_att| = k_att * d (zero inside tol).
    Accepts scalar or numpy array.
    """
    if isinstance(distance, (int, float)):
        d = max(float(distance), 0.0)
        return 0.0 if d <= tol else k_att * d
    d_arr = np.maximum(np.asarray(distance, dtype=float), 0.0)
    mag = k_att * d_arr
    mag[d_arr <= tol] = 0.0
    return mag


def attractive_moment_magnitude(
    angle_rad: np.ndarray | float,
    k_rot: float = 0.7,
    threshold: float = 0.02,
) -> np.ndarray | float:
    """
    Magnitude of rotational attractive moment: |M_att| = k_rot * theta (zero inside threshold).
    Accepts scalar or numpy array.
    """
    if isinstance(angle_rad, (int, float)):
        th = max(float(angle_rad), 0.0)
        return 0.0 if th <= threshold else k_rot * th
    th_arr = np.maximum(np.asarray(angle_rad, dtype=float), 0.0)
    mag = k_rot * th_arr
    mag[th_arr <= threshold] = 0.0
    return mag


def repulsive_force_magnitude(
    distance: np.ndarray | float,
    k_rep: float,
    influence_distance: float,
    eps: float = 1e-6,
) -> np.ndarray | float:
    """
    Khatib-style repulsive magnitude inside influence distance q:
      |F_rep| = k_rep * (1/d - 1/q) * (1/d^2), for 0 < d < q; else 0.
    Safe near surfaces with epsilon clipping.
    Accepts scalar or numpy array.
    """
    q = max(float(influence_distance), eps)
    if isinstance(distance, (int, float)):
        d = max(float(distance), eps)
        if d >= q:
            return 0.0
        inv_d = 1.0 / d
        inv_q = 1.0 / q
        return k_rep * (inv_d - inv_q) * (inv_d * inv_d)
    d_arr = np.clip(np.asarray(distance, dtype=float), eps, None)
    mag = np.zeros_like(d_arr)
    mask = d_arr < q
    dm = d_arr[mask]
    inv_d = 1.0 / dm
    inv_q = 1.0 / q
    mag[mask] = k_rep * (inv_d - inv_q) * (inv_d * inv_d)
    return mag


def combine_forces_1d(
    distance_from_goal: float,
    obstacle_center: float,
    k_att: float,
    k_rep: float,
    influence_q: float,
    min_distance: float = 1e-3,
) -> tuple[float, float, float]:
    """
    1D slice helper along +x: attractive toward goal at x=0, repulsive from an obstacle at x=obstacle_center.
    Returns (F_attr, F_rep, F_total). Signs follow 1D directions.
    """
    d = float(distance_from_goal)
    # Attractive toward goal (negative direction along +x)
    F_attr = -k_att * d
    # Repulsive acts away from obstacle center
    dist_obs = abs(d - obstacle_center)
    F_rep_mag = float(repulsive_force_magnitude(
        max(dist_obs, min_distance), k_rep=k_rep, influence_distance=influence_q, eps=min_distance
    ))
    if F_rep_mag > 0.0:
        sign = -1.0 if d > obstacle_center else 1.0
        F_rep = sign * F_rep_mag
    else:
        F_rep = 0.0
    return F_attr, F_rep, F_attr + F_rep


def apply_motion_constraints_1d(
    v_des: float,
    v_prev: float,
    dt: float,
    vmax: float,
    amax: float,
) -> float:
    """
    Mirror simple 1D motion limits used in visualization:
      1) velocity hard cap
      2) acceleration cap relative to previous velocity
      3) re-apply velocity cap
    """
    v_cap = float(np.clip(v_des, -vmax, vmax))
    if dt <= 0.0:
        return v_cap
    dv = v_cap - v_prev
    dv_max = amax * dt
    if abs(dv) > dv_max and dv_max > 0.0:
        v_cap = v_prev + math.copysign(dv_max, dv)
    return float(np.clip(v_cap, -vmax, vmax))


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
                 beta_velocity: float = 1.0):
        self.attractive_gain = float(attractive_gain)
        self.repulsive_gain = float(repulsive_gain)
        self.linear_gain = float(linear_gain)
        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)
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

    def total_force(self, p: np.ndarray, g: np.ndarray) -> np.ndarray:
        return self.attractive_force(p, g) + self.repulsive_force(p)

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
            F = self.total_force(p, g)
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

    def draw_obstacles(self, ax):
        u = np.linspace(0, 2*np.pi, 18)
        v = np.linspace(0, np.pi, 10)
        # Spheres + influence

        def draw_sphere(s: SphereObstacle, inflate=0.0, ls='-'):
            xs = (s.radius + inflate)*np.outer(np.cos(u), np.sin(v)) + s.cx
            ys = (s.radius + inflate)*np.outer(np.sin(u), np.sin(v)) + s.cy
            zs = (s.radius + inflate) * \
                np.outer(np.ones_like(u), np.cos(v)) + s.cz
            ax.plot_wireframe(xs, ys, zs, linewidth=0.5, linestyle=ls)
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
        # Cylinders + influence edges

        def draw_cylinder(cyl: CylinderObstacle, inflate: float = 0.0, ls='-'):
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
        for b in self._obbs:
            draw_obb(b, inflate=0.0, ls='-')
            draw_obb(b, inflate=b.influence_distance, ls='--')
        for c in self._cyls:
            draw_cylinder(c, inflate=0.0, ls='-')
            draw_cylinder(c, inflate=c.influence_distance, ls='--')
        for s in self._spheres:
            draw_sphere(s, inflate=0.0, ls='-')
            draw_sphere(s, inflate=s.influence_distance, ls='--')

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
            F = self.total_force(p, g)
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
                # Ensure output directory exists
                out_dir = os.path.dirname(save_path)
                if out_dir:
                    os.makedirs(out_dir, exist_ok=True)

                if writer.lower() in ("ffmpeg", "ffmpeg_file", "ffmpeg_writer", "mp4", "h264"):
                    from matplotlib.animation import FFMpegWriter
                    w = FFMpegWriter(fps=fps, bitrate=bitrate)
                    # When passing a MovieWriter instance, don't pass fps/bitrate to save()
                    anim.save(save_path, writer=w, dpi=dpi)
                elif writer.lower() == "pillow":
                    from matplotlib.animation import PillowWriter
                    w = PillowWriter(fps=fps)
                    # PillowWriter doesn't use bitrate; don't pass fps/bitrate to save() with an instance
                    anim.save(save_path, writer=w, dpi=dpi)
                else:
                    # For string writer names (e.g., 'imagemagick'), passing fps is valid
                    anim.save(save_path, writer=writer, fps=fps, dpi=dpi)
                print(f"Saved animation to {save_path}")
            except Exception as e:
                print(f"Failed to save animation to {save_path}: {e}")

        plt.show()

    def min_path_clearance(self, xs, ys, zs) -> float:
        """Return the minimum signed clearance (distance from obstacle boundary) along the path."""
        pts = np.vstack([xs, ys, zs]).T
        min_d = +np.inf
        # Measure against all obstacles using their signed distance helpers (positive outside, negative inside).
        for p in pts:
            dmin = +np.inf
            for s in self._spheres:
                d, _ = s.distance_and_direction(p)
                dmin = min(dmin, d)
            for b in self._obbs:
                d, _ = b.distance_and_direction(p)
                dmin = min(dmin, d)
            for c in self._cyls:
                d, _ = c.distance_and_direction(p)
                dmin = min(dmin, d)
            min_d = min(min_d, dmin)
        return float(min_d)


def plot_kinematics(title: str,
                    res: Dict[str, Any],
                    show: bool = True,
                    save_path: str | None = None,
                    description: str = ""):
    """
    Args:
        title: plot title
        res: dictionary of time-series results from planner.simulate() or planner.plan_path()
        show: whether to display the plot interactively
        save_path: optional path to save the plot image (e.g., PNG)
        description: optional text description to include below the title

    Plot a 4x2 grid of time-series:
      (row 1, col 1) position components (x,y,z) + dashed |p|
      (row 1, col 2) distance-to-goal (if goal given) else |p|
      (row 2, col 1) linear velocity components (vx,vy,vz) + dashed |v|
      (row 2, col 2) min_clearance_m if present
      (row 3, col 1) linear acceleration components (ax,ay,az) + dashed |a|
      (row 3, col 2) joint positions vs time (if present)
      (row 4, col 1) angular velocity components (wx,wy,wz) + dashed |w| (if present)
      (row 4, col 2) joint velocities vs time (if present)

    Expected keys in res:
      - "t": time array [N]
      - "x", "y", "z": position components [N]
      - "vx", "vy", "vz": linear velocity components [N]
      - optionally: "wx","wy","wz" (or CSV-style "ee_wx" etc.) and "min_clearance_m".
      - optionally: "joint_positions" [N x num_joints], "joint_velocities" [N x num_joints]
    """

    t = np.asarray(res.get("t", []), dtype=float)
    x = np.asarray(res.get("x", []), dtype=float)
    y = np.asarray(res.get("y", []), dtype=float)
    z = np.asarray(res.get("z", []), dtype=float)
    vx = np.asarray(res.get("vx", []), dtype=float)
    vy = np.asarray(res.get("vy", []), dtype=float)
    vz = np.asarray(res.get("vz", []), dtype=float)

    # Optional series
    wx = res.get("wx")
    wy = res.get("wy")
    wz = res.get("wz")
    if wx is None and ("ee_wx" in res):
        wx = res.get("ee_wx")
        wy = res.get("ee_wy")
        wz = res.get("ee_wz")
    wx = None if wx is None else np.asarray(wx, dtype=float)
    # Orientation (quaternions) if available
    qx = res.get("qx", res.get("ee_qx"))
    qy = res.get("qy", res.get("ee_qy"))
    qz = res.get("qz", res.get("ee_qz"))
    qw = res.get("qw", res.get("ee_qw"))
    has_quat_series = (
        qx is not None and qy is not None and qz is not None and qw is not None)
    if has_quat_series:
        qx = np.asarray(qx, dtype=float)
        qy = np.asarray(qy, dtype=float)
        qz = np.asarray(qz, dtype=float)
        qw = np.asarray(qw, dtype=float)
    wy = None if wy is None else np.asarray(wy, dtype=float)
    wz = None if wz is None else np.asarray(wz, dtype=float)
    min_clear = res.get("min_clearance_m", res.get("min_clearance"))
    min_clear = None if min_clear is None else np.asarray(
        min_clear, dtype=float)

    # Joint data
    joint_pos = res.get("joint_positions")
    joint_vel = res.get("joint_velocities")
    if joint_pos is not None:
        joint_pos = np.asarray(joint_pos, dtype=float)
    if joint_vel is not None:
        joint_vel = np.asarray(joint_vel, dtype=float)

    if t.size == 0:
        raise ValueError("Result dictionary is missing time array 't'.")

    # Compute accelerations via gradient over time (handles non-uniform dt)
    ax = np.gradient(vx, t)
    ay = np.gradient(vy, t)
    az = np.gradient(vz, t)

    pos_mag = np.sqrt(x*x + y*y + z*z)
    vel_mag = np.sqrt(vx*vx + vy*vy + vz*vz)
    acc_mag = np.sqrt(ax*ax + ay*ay + az*az)

    # Build 4x2 grid
    fig, axes = plt.subplots(4, 2, sharex=True, figsize=(12, 12))
    fig.suptitle(f"{title}", fontsize=16)

    # Row 1, Col 1: Position components (+|p|)
    ax11 = axes[0, 0]
    ax11.plot(t, x, label='x')
    ax11.plot(t, y, label='y')
    ax11.plot(t, z, label='z')
    ax11.plot(t, pos_mag, '--', color='k', linewidth=1.2, label='|p|')
    ax11.set_ylabel('Position [m]')
    ax11.grid(True, linestyle=':')
    ax11.legend(loc='best', fontsize=9)

    # Row 1, Col 2: Distance to goal (if provided) else |p|
    ax12 = axes[0, 1]
    goal_pos = res.get('goal_pos')
    if goal_pos is not None and goal_pos.size > 0:
        g = np.asarray(goal_pos, dtype=float).reshape(3,)
        d_goal = np.sqrt((x - g[0])**2 + (y - g[1])**2 + (z - g[2])**2)
        line_d, = ax12.plot(t, d_goal, color='C3', label='Dist→Goal')
        ax12.set_ylabel('Dist→Goal [m]')

        pos_tol = res.get('goal_tolerance_m')
        if pos_tol is not None:
            ax12.axhline(y=pos_tol, color=line_d.get_color(),
                         linestyle='--', linewidth=1.2, label='Pos. Tolerance')

        # Initialize angular distance array
        angle_to_goal = np.zeros_like(d_goal)
        goal_q = res.get("goal_q")
        if has_quat_series and goal_q is not None and goal_q.size > 0:
            # Create Rotation objects from the time series of quaternions
            path_quats = scipy.spatial.transform.Rotation.from_quat(
                np.vstack([qx, qy, qz, qw]).T
            )
            goal_quat_rot = scipy.spatial.transform.Rotation.from_quat(goal_q)
            # Compute angular distance for each point in the path
            # The magnitude of the rotation vector of the relative rotation is the angle
            angle_to_goal = (path_quats.inv() * goal_quat_rot).as_rotvec()
            angle_to_goal = np.linalg.norm(angle_to_goal, axis=1)

            # Plot angular distance on a secondary y-axis
            ax12_twin = ax12.twinx()
            line_a, = ax12_twin.plot(
                t, angle_to_goal, color='C4', linestyle='-', label='Angle→Goal')
            ax12_twin.set_ylabel('Angle→Goal [rad]', color='C4')
            ax12_twin.tick_params(axis='y', labelcolor='C4')

            ang_tol = res.get('angular_tolerance_rad')
            if ang_tol is not None:
                ax12_twin.axhline(y=ang_tol, color=line_a.get_color(
                ), linestyle=':', linewidth=1.2, label='Ang. Tolerance')

            # Combine legends from both y-axes
            lines = [line_d, line_a]
            # Add the tolerance lines to the legend
            lines.extend([l for l in ax12.get_lines()
                         if l.get_linestyle() == '--'])
            lines.extend([l for l in ax12_twin.get_lines()
                         if l.get_linestyle() == ':'])
            ax12.legend(lines, [l.get_label()
                        for l in lines], loc='upper right', fontsize=9)

        # Find the first index where BOTH translational and rotational tolerances are met
        pos_tol = res.get("goal_tolerance_m")
        ang_tol = res.get("angular_tolerance_rad")

        if pos_tol is not None and ang_tol is not None:
            pos_ok = d_goal <= pos_tol
            rot_ok = angle_to_goal <= ang_tol
            both_ok = np.logical_and(pos_ok, rot_ok)

            if np.any(both_ok):
                first_converged_idx = np.where(both_ok)[0][0]
                ax12.axvline(t[first_converged_idx], color='C2', linestyle='--',
                             linewidth=1.5, label=f"Converged at t={t[first_converged_idx]:.2f}s")
                # Update the main legend to include the convergence line
                lines.append(ax12.get_lines()[-1])  # Add the axvline
                ax12.legend(lines, [l.get_label()
                            for l in lines], loc='upper right', fontsize=9)
            else:
                # If convergence was never met, add text to the plot
                messages = []
                if not np.any(pos_ok):
                    messages.append("Position Tolerance not met")
                if not np.any(rot_ok):
                    messages.append("Angular Tolerance not met")

                if messages:
                    message_text = "\n".join(messages)
                    ax12.text(0.95, 0.05, message_text,
                              transform=ax12.transAxes,
                              fontsize=9, color='red',
                              ha='right', va='bottom',
                              bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='red', alpha=0.8))
        elif 'ax12_twin' not in locals():
            ax12.legend(loc='best', fontsize=9)

    else:
        ax12.plot(t, pos_mag, color='C7', label='|p|')
        ax12.set_ylabel('|p| [m]')
        ax12.legend(loc='best', fontsize=9)
    ax12.grid(True, linestyle=':')

    # Row 2, Col 1: Linear velocity components (+|v|)
    ax21 = axes[1, 0]
    ax21.plot(t, vx, label='vx')
    ax21.plot(t, vy, label='vy')
    ax21.plot(t, vz, label='vz')
    ax21.plot(t, vel_mag, '--', color='k', linewidth=1.2, label='|v|')
    ax21.set_ylabel('Velocity [m/s]')
    ax21.grid(True, linestyle=':')
    ax21.legend(loc='best', fontsize=9)

    # Row 2, Col 2: Min clearance (if present)
    ax22 = axes[1, 1]
    if min_clear is not None:
        ax22.plot(t, min_clear, color='C2', label='min_clearance')
        ax22.legend(loc='best', fontsize=9)
    else:
        ax22.text(0.5, 0.5, 'No min_clearance provided', transform=ax22.transAxes,
                  ha='center', va='center', fontsize=10, color='gray')
    ax22.set_ylabel('Clearance [m]')
    ax22.grid(True, linestyle=':')

    # Row 3, Col 1: Linear acceleration components (+|a|)
    ax31 = axes[2, 0]
    ax31.plot(t, ax, label='ax')
    ax31.plot(t, ay, label='ay')
    ax31.plot(t, az, label='az')
    ax31.plot(t, acc_mag, '--', color='k', linewidth=1.2, label='|a|')
    ax31.set_ylabel('Accel [m/s^2]')
    ax31.grid(True, linestyle=':')
    ax31.legend(loc='best', fontsize=9)

    # Row 3, Col 2: Joint Positions
    ax32 = axes[2, 1]
    if joint_pos is not None:
        num_joints = joint_pos.shape[1]
        for j in range(num_joints):
            ax32.plot(t, joint_pos[:, j], label=f'q{j+1}')
        ax32.set_ylabel('Joint Pos [rad]')
        ax32.legend(loc='best', fontsize=8, ncol=2)
    else:
        ax32.text(0.5, 0.5, 'No joint positions provided', transform=ax32.transAxes,
                  ha='center', va='center', fontsize=10, color='gray')
    ax32.grid(True, linestyle=':')

    # Row 4, Col 1: Angular velocity components (+|w|) if present
    ax41 = axes[3, 0]
    if wx is not None and wy is not None and wz is not None:
        ax41.plot(t, wx, label='wx')
        ax41.plot(t, wy, label='wy')
        ax41.plot(t, wz, label='wz')
        wmag = np.sqrt(wx*wx + wy*wy + wz*wz)
        ax41.plot(t, wmag, '--', color='k', linewidth=1.2, label='|w|')
        ax41.set_ylabel('Angular vel [rad/s]')
        ax41.legend(loc='best', fontsize=9)
    else:
        ax41.text(0.5, 0.5, 'No angular velocity provided', transform=ax41.transAxes,
                  ha='center', va='center', fontsize=10, color='gray')
        ax41.set_ylabel('Angular vel [rad/s]')
    ax41.grid(True, linestyle=':')
    ax41.set_xlabel('Time [s]')

    # Row 4, Col 2: Joint Velocities
    ax42 = axes[3, 1]
    if joint_vel is not None:
        num_joints = joint_vel.shape[1]
        for j in range(num_joints):
            ax42.plot(t, joint_vel[:, j], label=f'dq{j+1}')
        ax42.set_ylabel('Joint Vel [rad/s]')
        ax42.legend(loc='best', fontsize=8, ncol=2)
    else:
        ax42.text(0.5, 0.5, 'No joint velocities provided', transform=ax42.transAxes,
                  ha='center', va='center', fontsize=10, color='gray')
    ax42.grid(True, linestyle=':')
    ax42.set_xlabel('Time [s]')

    # Create a text box for the description if provided
    if description:
        fig.text(0.5, 0.92, description, ha='center', va='bottom', fontsize=10)

    fig.tight_layout(rect=[0, 0, 1, 0.96])

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved kinematics plot to {save_path}.png")
    if show:
        plt.show()
    else:
        plt.close(fig)
    return fig, axes
